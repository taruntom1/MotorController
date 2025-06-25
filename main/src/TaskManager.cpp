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

    // Create mutexes for task suspension control
    control_loop_mutex = xSemaphoreCreateMutexStatic(&control_loop_mutex_buffer);
    odo_broadcast_mutex = xSemaphoreCreateMutexStatic(&odo_broadcast_mutex_buffer);

    if (control_loop_mutex == nullptr || odo_broadcast_mutex == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create task suspension mutexes");
        // Clean up queue if mutex creation fails
        if (task_state_queue_ != nullptr)
        {
            vQueueDelete(task_state_queue_);
            task_state_queue_ = nullptr;
        }
        return;
    }

    // Create wheel manager task with error checking
    wheel_manage_task_handle = xTaskCreateStatic(wheelManageTaskEntry, "WheelManager",
                                                 WHEEL_MANAGE_TASK_STACK_SIZE, this,
                                                 WHEEL_MANAGE_TASK_PRIORITY,
                                                 wheel_manage_task_stack,
                                                 &wheel_manage_task_tcb);
    if (wheel_manage_task_handle == nullptr)
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

    // Clean up mutexes
    if (control_loop_mutex != nullptr)
    {
        vSemaphoreDelete(control_loop_mutex);
        control_loop_mutex = nullptr;
    }

    if (odo_broadcast_mutex != nullptr)
    {
        vSemaphoreDelete(odo_broadcast_mutex);
        odo_broadcast_mutex = nullptr;
    }
}

void TaskManager::wheelManageTaskEntry(void *pvParameters)
{
    static_cast<TaskManager *>(pvParameters)->wheelManageTask();
}

void TaskManager::mutexProtectedProcessHelper(const std::function<void()> &processFunc)
{
    // Acquire both mutexes to ensure thread safety with running tasks
    if (xSemaphoreTake(control_loop_mutex, portMAX_DELAY) == pdTRUE)
    {
        if (xSemaphoreTake(odo_broadcast_mutex, portMAX_DELAY) == pdTRUE)
        {
            // Execute the processing function while holding both mutexes
            processFunc();

            // Release mutexes in reverse order
            xSemaphoreGive(odo_broadcast_mutex);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to acquire odo broadcast mutex for wheel processing");
        }
        xSemaphoreGive(control_loop_mutex);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to acquire control loop mutex for wheel processing");
    }
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
            mutexProtectedProcessHelper([this]()
                                        { wheel_container_.processPendingWheelCount(); });
        }

        if (ulNotificationValue & std::to_underlying(task_manager_notifications::WHEEL_UPDATE))
        {
            mutexProtectedProcessHelper([this]()
                                        { wheel_container_.processWheelDataQueue(); });
        }

        if (ulNotificationValue & std::to_underlying(task_manager_notifications::CONTROL_MODE_UPDATE))
        {
            mutexProtectedProcessHelper([this]()
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
        // Acquire mutex at the beginning of the loop iteration
        if (xSemaphoreTake(control_loop_mutex, portMAX_DELAY) == pdTRUE)
        {
            wheel_container_.executeControlLoop();

            // Release mutex after execution
            xSemaphoreGive(control_loop_mutex);

            vTaskDelayUntil(&last_wake_time, control_task_delay_ticks.load(std::memory_order_acquire));
        }
        else
        {
            ESP_LOGE(TAG, "Failed to acquire control loop mutex");
            vTaskDelay(pdMS_TO_TICKS(10)); // Small delay before retry
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
        // Acquire mutex at the beginning of the loop iteration
        if (xSemaphoreTake(odo_broadcast_mutex, portMAX_DELAY) == pdTRUE)
        {
            odoBroadcastData.first = esp_timer_get_time();
            odoBroadcastData.second = wheel_container_.collectOdometry();

            assert(odoBroadcastCallback && "odoBroadcastCallback must be set before calling");
            odoBroadcastCallback(odoBroadcastData);

            // Release mutex after execution
            xSemaphoreGive(odo_broadcast_mutex);

            vTaskDelayUntil(&last_wake_time, odo_broadcast_task_delay_ticks.load(std::memory_order_acquire));
        }
        else
        {
            ESP_LOGE(TAG, "Failed to acquire odo broadcast mutex");
            vTaskDelay(pdMS_TO_TICKS(10)); // Small delay before retry
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
                                   StackType_t *stackBuffer,
                                   StaticTask_t *taskBuffer)
{
    if (taskHandle == nullptr)
    {
        taskHandle = xTaskCreateStatic(taskFunction, taskName, stackSize, this, priority, stackBuffer, taskBuffer);
        if (taskHandle != nullptr)
        {
            taskState = TaskState::Running;
            return true;
        }
        else
        {
            taskHandle = nullptr;
            ESP_LOGE(TAG, "Failed to create %s (stack: %u, priority: %u)",
                     taskName, stackSize, priority);
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
                            control_task_stack, &control_task_tcb);
}

bool TaskManager::createOdoBroadcastTask()
{
    return createTaskHelper(odoBroadcastTaskEntry, "Odo Broadcast Task",
                            ODO_BROADCAST_TASK_STACK_SIZE, ODO_BROADCAST_TASK_PRIORITY,
                            odo_broadcast_task_handle, odo_broadcast_task_state_,
                            odo_broadcast_task_stack, &odo_broadcast_task_tcb);
}

bool TaskManager::controlLoopTaskAction(TaskAction action)
{
    return handleTaskAction(action, control_task_handle, control_task_state_, [this]()
                            { return createControlTask(); }, [this]()
                            { return suspendAndWaitForControlLoopSuspend(); });
}

bool TaskManager::odoBroadcastTaskAction(TaskAction action)
{
    return handleTaskAction(action, odo_broadcast_task_handle, odo_broadcast_task_state_, [this]()
                            { return createOdoBroadcastTask(); }, [this]()
                            { return suspendAndWaitForOdoBroadcastSuspend(); });
}

bool TaskManager::handleTaskAction(TaskAction action,
                                   TaskHandle_t &task_handle,
                                   TaskState &task_state,
                                   std::function<bool()> task_creator,
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

bool TaskManager::suspendAndWaitForTaskSuspend(SemaphoreHandle_t mutex,
                                               TaskHandle_t task_handle,
                                               const char *timeout_log_tag)
{
    if (mutex == nullptr || task_handle == nullptr)
    {
        ESP_LOGE(TAG, "Mutex or task handle is null for %s", timeout_log_tag);
        return false;
    }

    // Try to acquire the mutex with timeout - this ensures the task is not in critical section
    const auto timeout_ticks = pdMS_TO_TICKS(TASK_SUSPENSION_TIMEOUT_MS);

    if (xSemaphoreTake(mutex, timeout_ticks) == pdTRUE)
    {
        // Task is not in critical section, safe to suspend
        vTaskSuspend(task_handle);

        // Release the mutex so task can resume when needed
        xSemaphoreGive(mutex);

        ESP_LOGI(TAG, "%s: Task suspended successfully", timeout_log_tag);
        return true;
    }
    else
    {
        ESP_LOGE(TAG, "Timeout waiting for %s to exit critical section", timeout_log_tag);
        return false;
    }
}

bool TaskManager::suspendAndWaitForControlLoopSuspend()
{
    return suspendAndWaitForTaskSuspend(control_loop_mutex,
                                        control_task_handle,
                                        "control loop");
}

bool TaskManager::suspendAndWaitForOdoBroadcastSuspend()
{
    return suspendAndWaitForTaskSuspend(odo_broadcast_mutex,
                                        odo_broadcast_task_handle,
                                        "odometry broadcast");
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
