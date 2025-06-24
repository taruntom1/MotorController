#include "TaskManager.h"
#include "WheelContainer.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <utility> // for std::to_underlying

TaskManager::TaskManager(TaskManagerConfig config, WheelContainer &wheelContainer)
    : wheel_container_(wheelContainer),
      control_task_delay_ticks(pdMS_TO_TICKS(1000 / config.control_loop_frequency)),
      odo_broadcast_task_delay_ticks(pdMS_TO_TICKS(1000 / config.odo_broadcast_frequency))
{
    // Create task state queue
    task_state_queue_ = xQueueCreate(TASK_STATE_QUEUE_LENGTH, sizeof(TaskStateCommand));
    if (task_state_queue_ == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create task state queue");
        // Consider throwing an exception or setting an error flag
        return;
    }

    // Create wheel manager task with error checking
    BaseType_t result = xTaskCreate(wheelManageTaskEntry, "WheelManager",
                                    WHEEL_MANAGE_TASK_STACK_SIZE, this,
                                    WHEEL_MANAGE_TASK_PRIORITY, &wheel_manage_task_handle);
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create wheel manager task");
        // Clean up queue if task creation fails
        if (task_state_queue_ != nullptr)
        {
            vQueueDelete(task_state_queue_);
            task_state_queue_ = nullptr;
        }
    }
}

TaskManager::~TaskManager()
{
    // Stop tasks gracefully before deleting them
    if (control_task_handle != nullptr)
    {
        controlLoopTaskAction(TaskAction::Stop);
    }

    if (odo_broadcast_task_handle != nullptr)
    {
        odoBroadcastTaskAction(TaskAction::Stop);
    }

    // Delete wheel manager task
    if (wheel_manage_task_handle != nullptr)
    {
        vTaskDelete(wheel_manage_task_handle);
        wheel_manage_task_handle = nullptr;
    }

    // Clean up task state queue
    if (task_state_queue_ != nullptr)
    {
        vQueueDelete(task_state_queue_);
        task_state_queue_ = nullptr;
    }
}

void TaskManager::wheelManageTaskEntry(void *pvParameters)
{
    static_cast<TaskManager *>(pvParameters)->wheelManageTask();
}

void TaskManager::suspendResumeAndProcessHelper(const std::function<void()> &processFunc)
{
    using enum TaskAction;
    controlLoopTaskAction(Suspend);
    odoBroadcastTaskAction(Suspend);
    processFunc(); 
    controlLoopTaskAction(Resume);
    odoBroadcastTaskAction(Resume);
}

void TaskManager::wheelManageTask()
{
    while (true)
    {
        uint32_t ulNotificationValue;
        xTaskNotifyWait(0, ULONG_MAX, &ulNotificationValue, portMAX_DELAY);

        if (ulNotificationValue & std::to_underlying(task_manager_notifications::PROCESS_TASK_STATE_QUEUE))
        {
            processTaskStateQueue();
        }

        if (ulNotificationValue & std::to_underlying(task_manager_notifications::NUM_WHEEL_UPDATE))
        {
            suspendResumeAndProcessHelper([this]()
                                          { wheel_container_.processPendingWheelCount(); });
        }

        if (ulNotificationValue & std::to_underlying(task_manager_notifications::WHEEL_UPDATE))
        {
            suspendResumeAndProcessHelper([this]()
                                          { wheel_container_.processWheelDataQueue(); });
        }

        if (ulNotificationValue & std::to_underlying(task_manager_notifications::CONTROL_MODE_UPDATE))
        {
            suspendResumeAndProcessHelper([this]()
                                          { wheel_container_.processControlModeQueue(); });
        }
    }
}

void TaskManager::controlTaskEntry(void *pvParameters)
{
    static_cast<TaskManager *>(pvParameters)->controlTask();
}

void TaskManager::controlTask()
{
    TickType_t last_wake_time = xTaskGetTickCount();
    while (true)
    {
        wheel_container_.executeControlLoop();
        vTaskDelayUntil(&last_wake_time, control_task_delay_ticks.load(std::memory_order_acquire));

        if (!control_loop_run.load(std::memory_order_acquire))
        {
            notifyTaskManager(task_manager_notifications::CONTROL_LOOP_SUSPENDED);
            vTaskSuspend(nullptr);
        }
    }
}

void TaskManager::odoBroadcastTaskEntry(void *pvParameters)
{
    static_cast<TaskManager *>(pvParameters)->odoBroadcastTask();
}

void TaskManager::odoBroadcastTask()
{
    odoBroadcastData.second.reserve(wheel_container_.getWheelCount());
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true)
    {
        odoBroadcastData.first = esp_timer_get_time();
        odoBroadcastData.second = wheel_container_.collectOdometry();

        assert(odoBroadcastCallback && "odoBroadcastCallback must be set before calling");
        odoBroadcastCallback(odoBroadcastData);
        vTaskDelayUntil(&last_wake_time, odo_broadcast_task_delay_ticks.load(std::memory_order_acquire));
        if (!odo_broadcast_run.load(std::memory_order_acquire))
        {
            notifyTaskManager(task_manager_notifications::ODO_BROADCAST_SUSPENDED);
            vTaskSuspend(nullptr);
        }
    }
}

void TaskManager::updateOdoBroadcastFrequency(frequency_t frequency)
{
    if (frequency > 0)
    {
        odo_broadcast_task_delay_ticks.store(pdMS_TO_TICKS(xPortGetTickRateHz() / frequency),
                                             std::memory_order_release);
        ESP_LOGI(TAG, "Odo broadcast frequency updated to %d Hz", frequency);
    }
    else
    {
        ESP_LOGW(TAG, "Invalid odo broadcast frequency: %d Hz", frequency);
    }
}

void TaskManager::updateControlLoopFrequency(frequency_t frequency)
{
    if (frequency > 0)
    {
        control_task_delay_ticks.store(pdMS_TO_TICKS(xPortGetTickRateHz() / frequency),
                                       std::memory_order_release);
        wheel_container_.updateControlLoopFrequency(frequency);
        ESP_LOGI(TAG, "Control loop frequency updated to %d Hz", frequency);
    }
    else
    {
        ESP_LOGW(TAG, "Invalid control loop frequency: %d Hz", frequency);
    }
}

void TaskManager::controlLoopTaskActionNonBlocking(TaskAction action)
{
    enqueueTaskStateCommand(TaskType::ControlLoop, action);
}

void TaskManager::odoBroadcastTaskActionNonBlocking(TaskAction action)
{
    enqueueTaskStateCommand(TaskType::OdoBroadcast, action);
}

// Helper method for common task creation logic
bool TaskManager::createTaskHelper(TaskFunction_t taskFunction,
                                   const char *taskName,
                                   UBaseType_t stackSize,
                                   UBaseType_t priority,
                                   TaskHandle_t &taskHandle,
                                   TaskState &taskState,
                                   std::atomic<bool> &runFlag)
{
    if (taskHandle == nullptr)
    {
        runFlag.store(true, std::memory_order_release);
        BaseType_t result = xTaskCreate(taskFunction, taskName, stackSize, this, priority, &taskHandle);
        if (result == pdPASS)
        {
            taskState = TaskState::Running;
            return true;
        }
        else
        {
            runFlag.store(false, std::memory_order_release);
            taskHandle = nullptr;
            ESP_LOGE(TAG, "Failed to create %s (stack: %u, priority: %u), error: %d",
                     taskName, stackSize, priority, result);
            return false;
        }
    }
    return false;
}

bool TaskManager::createControlTask()
{
    return createTaskHelper(controlTaskEntry, "Control Task",
                            CONTROL_TASK_STACK_SIZE, CONTROL_TASK_PRIORITY,
                            control_task_handle, control_task_state_,
                            control_loop_run);
}

bool TaskManager::createOdoBroadcastTask()
{
    return createTaskHelper(odoBroadcastTaskEntry, "Odo Broadcast Task",
                            ODO_BROADCAST_TASK_STACK_SIZE, ODO_BROADCAST_TASK_PRIORITY,
                            odo_broadcast_task_handle, odo_broadcast_task_state_,
                            odo_broadcast_run);
}

bool TaskManager::controlLoopTaskAction(TaskAction action)
{
    return handleTaskAction(action, control_task_handle, control_task_state_, [this]()
                            { return createControlTask(); }, control_loop_run, [this]()
                            { return suspendAndWaitForControlLoopSuspend(); });
}

bool TaskManager::odoBroadcastTaskAction(TaskAction action)
{
    return handleTaskAction(action, odo_broadcast_task_handle, odo_broadcast_task_state_, [this]()
                            { return createOdoBroadcastTask(); }, odo_broadcast_run, [this]()
                            { return suspendAndWaitForOdoBroadcastSuspend(); });
}

bool TaskManager::handleTaskAction(TaskAction action,
                                   TaskHandle_t &task_handle,
                                   TaskState &task_state,
                                   std::function<bool()> task_creator,
                                   std::atomic<bool> &run_flag,
                                   std::function<bool()> suspend_handler)
{
    const char *task_name = getTaskName(task_handle);
    const char *action_str = taskActionToString(action);
    const char *current_state_str = taskStateToString(task_state);

    // Early return for invalid combinations
    if (task_handle == nullptr && action != TaskAction::Start)
    {
        ESP_LOGW(TAG, "%s: Cannot %s non-existent task", task_name, action_str);
        return false;
    }

    using enum TaskAction;
    switch (action)
    {
    case Start:
        if (task_handle == nullptr || task_state == TaskState::Deleted)
        {
            bool created = task_creator();
            if (created)
            {
                task_state = TaskState::Running;
                ESP_LOGI(TAG, "%s: Started successfully (%s -> Running)", task_name, current_state_str);
            }
            else
            {
                ESP_LOGE(TAG, "%s: Failed to start from %s state", task_name, current_state_str);
            }
            return created;
        }
        ESP_LOGW(TAG, "%s: Already exists in %s state, cannot start", task_name, current_state_str);
        return false;

    case Stop:
        if (task_state != TaskState::Deleted)
        {
            if (suspend_handler())
            {
                vTaskDelete(task_handle);
                task_handle = nullptr;
                task_state = TaskState::Deleted;
                ESP_LOGI(TAG, "%s: Stopped successfully (%s -> Deleted)", task_name, current_state_str);
                return true;
            }
            ESP_LOGE(TAG, "%s: Failed to stop from %s state (suspend failed)", task_name, current_state_str);
        }
        else
        {
            ESP_LOGW(TAG, "%s: Already in Deleted state, cannot stop", task_name);
        }
        return false;

    case Suspend:
        if (task_state == TaskState::Running)
        {
            bool suspended = suspend_handler();
            if (suspended)
            {
                task_state = TaskState::Suspended;
                ESP_LOGI(TAG, "%s: Suspended successfully (Running -> Suspended)", task_name);
            }
            else
            {
                ESP_LOGE(TAG, "%s: Failed to suspend from Running state", task_name);
            }
            return suspended;
        }
        ESP_LOGW(TAG, "%s: Cannot suspend from %s state", task_name, current_state_str);
        return false;

    case Resume:
        if (task_state == TaskState::Suspended)
        {
            run_flag.store(true, std::memory_order_release);
            vTaskResume(task_handle);
            task_state = TaskState::Running;
            ESP_LOGI(TAG, "%s: Resumed successfully (Suspended -> Running)", task_name);
            return true;
        }
        ESP_LOGW(TAG, "%s: Cannot resume from %s state", task_name, current_state_str);
        return false;
    }

    return false;
}

bool TaskManager::suspendAndWaitForTaskSuspend(TaskHandle_t &task_handle,
                                               task_manager_notifications notification_type,
                                               std::atomic<bool> &run_flag,
                                               const char *timeout_log_tag)
{
    if (task_handle == nullptr)
        return true;

    if (eTaskState state = eTaskGetState(task_handle);
        state == eDeleted || state == eSuspended)
    {
        if (state == eDeleted)
            task_handle = nullptr;
        return true;
    }

    const auto expected_notification = std::to_underlying(notification_type);
    run_flag.store(false, std::memory_order_release); // Stop the loop

    uint32_t notification = 0;
    const auto timeout_ticks = pdMS_TO_TICKS(TASK_SUSPENSION_TIMEOUT_MS);

    while (!(notification & expected_notification))
    {
        if (xTaskNotifyWait(0, expected_notification, &notification, timeout_ticks) == pdFAIL)
        {
            ESP_LOGE(TAG, "Timeout waiting for %s suspension", timeout_log_tag);
            return false; // Timeout or failure
        }
    }

    return true;
}

bool TaskManager::suspendAndWaitForControlLoopSuspend()
{
    return suspendAndWaitForTaskSuspend(control_task_handle,
                                        task_manager_notifications::CONTROL_LOOP_SUSPENDED,
                                        control_loop_run, "control loop");
}

bool TaskManager::suspendAndWaitForOdoBroadcastSuspend()
{
    return suspendAndWaitForTaskSuspend(odo_broadcast_task_handle,
                                        task_manager_notifications::ODO_BROADCAST_SUSPENDED,
                                        odo_broadcast_run, "odometry broadcast");
}

bool TaskManager::enqueueTaskStateCommand(TaskType task_type, TaskAction action)
{
    if (task_state_queue_ == nullptr)
    {
        ESP_LOGE(TAG, "Task state queue not initialized");
        return false;
    }

    TaskStateCommand command = {task_type, action};

    // Use a small timeout instead of immediate return to handle brief queue congestion
    if (const auto queue_timeout = pdMS_TO_TICKS(10);
        xQueueSendToBack(task_state_queue_, &command, queue_timeout) != pdPASS)
    {
        ESP_LOGW(TAG, "Queue: Failed to enqueue %s %s command (queue full/timeout)",
                 taskTypeToString(task_type), taskActionToString(action));
        return false;
    }

    // Trigger queue processing
    notifyTaskManager(task_manager_notifications::PROCESS_TASK_STATE_QUEUE);
    return true;
}

void TaskManager::processTaskStateQueue()
{
    TaskStateCommand command;
    uint8_t processed_count = 0;
    const uint8_t max_commands_per_iteration = 5; // Prevent excessive processing in one go

    // Process pending commands in the queue (with limit to prevent starvation)
    while (processed_count < max_commands_per_iteration &&
           xQueueReceive(task_state_queue_, &command, pdMS_TO_TICKS(50)) == pdPASS)
    {
        using enum TaskType;
        switch (command.task_type)
        {
        case ControlLoop:
            controlLoopTaskAction(command.action);
            break;
        case OdoBroadcast:
            odoBroadcastTaskAction(command.action);
            break;
        default:
            ESP_LOGW(TAG, "ProcessQueue: Unknown task type %d", static_cast<int>(command.task_type));
            break;
        }

        processed_count++;
    }

    // If we hit the limit, trigger another processing cycle
    if (processed_count >= max_commands_per_iteration &&
        uxQueueMessagesWaiting(task_state_queue_) > 0)
    {
        notifyTaskManager(task_manager_notifications::PROCESS_TASK_STATE_QUEUE);
    }
}

// Helper functions for enum to string conversion
const char *TaskManager::taskActionToString(TaskAction action) const
{
    using enum TaskAction;
    switch (action)
    {
    case Start:
        return "Start";
    case Stop:
        return "Stop";
    case Suspend:
        return "Suspend";
    case Resume:
        return "Resume";
    default:
        return "Unknown";
    }
}

const char *TaskManager::taskStateToString(TaskState state) const
{
    using enum TaskState;
    switch (state)
    {
    case Running:
        return "Running";
    case Suspended:
        return "Suspended";
    case Deleted:
        return "Deleted";
    default:
        return "Unknown";
    }
}

const char *TaskManager::taskTypeToString(TaskType type) const
{
    using enum TaskType;
    switch (type)
    {
    case ControlLoop:
        return "ControlLoop";
    case OdoBroadcast:
        return "OdoBroadcast";
    default:
        return "Unknown";
    }
}

const char *TaskManager::getTaskName(TaskHandle_t &task_handle) const
{
    if (task_handle == control_task_handle)
        return "ControlTask";
    if (task_handle == odo_broadcast_task_handle)
        return "OdoBroadcastTask";
    return "UnknownTask";
}
