#include "TaskManager.h"
#include "WheelContainer.h"
#include "Task.h"
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
        return;
    }

    // Create mutexes for task suspension control
    control_loop_mutex = xSemaphoreCreateMutexStatic(&control_loop_mutex_buffer);
    odo_broadcast_mutex = xSemaphoreCreateMutexStatic(&odo_broadcast_mutex_buffer);

    if (control_loop_mutex == nullptr || odo_broadcast_mutex == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create task suspension mutexes");
        if (task_state_queue_ != nullptr)
        {
            vQueueDelete(task_state_queue_);
            task_state_queue_ = nullptr;
        }
        return;
    }

    // Create wheel manager task using Task wrapper
    Task::Config wheel_config{
        .name = "WheelManager",
        .stack_size = WHEEL_MANAGE_TASK_STACK_SIZE,
        .priority = WHEEL_MANAGE_TASK_PRIORITY,
        .core_id = tskNO_AFFINITY,
        .use_static_allocation = true};

    wheel_manage_task_ = std::make_unique<Task>(
        wheel_config,
        [this]()
        { this->wheelManageTask(); },
        wheel_manage_task_stack,
        &wheel_manage_task_tcb);

    if (!wheel_manage_task_->start())
    {
        ESP_LOGE(TAG, "Failed to start wheel manager task");
        wheel_manage_task_.reset();
        if (task_state_queue_ != nullptr)
        {
            vQueueDelete(task_state_queue_);
            task_state_queue_ = nullptr;
        }
    }
}

TaskManager::~TaskManager()
{
    // Task wrappers will automatically clean up themselves
    // Just clean up other resources
    control_task_.reset();
    odo_broadcast_task_.reset();
    wheel_manage_task_.reset();

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

bool TaskManager::createControlTask()
{
    if (control_task_ != nullptr)
    {
        ESP_LOGW(TAG, "Control task already exists");
        return false;
    }

    Task::Config config{
        .name = "ControlTask",
        .stack_size = CONTROL_TASK_STACK_SIZE,
        .priority = CONTROL_TASK_PRIORITY,
        .core_id = tskNO_AFFINITY,
        .use_static_allocation = true};

    control_task_ = std::make_unique<Task>(
        config,
        [this]()
        { this->controlTask(); },
        control_task_stack,
        &control_task_tcb);

    // Set suspension mutex for coordinated shutdown
    control_task_->setSuspensionMutex(control_loop_mutex);

    return control_task_->start();
}

bool TaskManager::createOdoBroadcastTask()
{
    if (odo_broadcast_task_ != nullptr)
    {
        ESP_LOGW(TAG, "Odo broadcast task already exists");
        return false;
    }

    Task::Config config{
        .name = "OdoBroadcastTask",
        .stack_size = ODO_BROADCAST_TASK_STACK_SIZE,
        .priority = ODO_BROADCAST_TASK_PRIORITY,
        .core_id = tskNO_AFFINITY,
        .use_static_allocation = true};

    odo_broadcast_task_ = std::make_unique<Task>(
        config,
        [this]()
        { this->odoBroadcastTask(); },
        odo_broadcast_task_stack,
        &odo_broadcast_task_tcb);

    // Set suspension mutex for coordinated shutdown
    odo_broadcast_task_->setSuspensionMutex(odo_broadcast_mutex);

    return odo_broadcast_task_->start();
}

bool TaskManager::controlLoopTaskAction(TaskAction action)
{
    return handleTaskAction(action, control_task_, [this]() -> std::unique_ptr<Task>
                            {
        if (createControlTask()) {
            return std::move(control_task_);
        }
        return nullptr; });
}

bool TaskManager::odoBroadcastTaskAction(TaskAction action)
{
    return handleTaskAction(action, odo_broadcast_task_, [this]() -> std::unique_ptr<Task>
                            {
        if (createOdoBroadcastTask()) {
            return std::move(odo_broadcast_task_);
        }
        return nullptr; });
}

bool TaskManager::handleTaskAction(TaskAction action,
                                   std::unique_ptr<Task> &task_wrapper,
                                   std::function<std::unique_ptr<Task>()> task_creator)
{
    using enum TaskAction;
    switch (action)
    {
    case Start:
        if (task_wrapper == nullptr)
        {
            task_wrapper = task_creator();
        }
        return task_wrapper != nullptr;

    case Stop:
        return task_wrapper ? task_wrapper->stop() : true;

    case Suspend:
        return task_wrapper ? task_wrapper->suspend() : false;

    case Resume:
        return task_wrapper ? task_wrapper->resume() : false;
    }

    return false;
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
        ESP_LOGW(TAG, "Queue: Failed to enqueue command (queue full/timeout)");
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