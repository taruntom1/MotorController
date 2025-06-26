#include "core/TaskManager.h"
#include "control/WheelContainer.h"
#include "core/Task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <utility> // for std::to_underlying

/*
 * TaskManager implementation with simplified design:
 * - Consolidated frequency update logic with template helper
 * - Simplified task action handling without complex lambda creators
 * - Streamlined queue processing without artificial limits
 * - Template-based task creation to reduce code duplication
 */

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

void TaskManager::updateTaskFrequency(frequency_t frequency, 
                                     std::atomic<TickType_t>& delay_ticks, 
                                     const char* task_name,
                                     std::function<void(frequency_t)> additional_action)
{
    if (frequency > 0)
    {
        delay_ticks.store(pdMS_TO_TICKS(xPortGetTickRateHz() / frequency), std::memory_order_release);
        if (additional_action)
        {
            additional_action(frequency);
        }
        ESP_LOGI(TAG, "%s frequency updated to %d Hz", task_name, frequency);
    }
    else
    {
        ESP_LOGW(TAG, "Invalid %s frequency: %d Hz", task_name, frequency);
    }
}

void TaskManager::updateOdoBroadcastFrequency(frequency_t frequency)
{
    updateTaskFrequency(frequency, odo_broadcast_task_delay_ticks, "Odo broadcast", nullptr);
}

void TaskManager::updateControlLoopFrequency(frequency_t frequency)
{
    updateTaskFrequency(frequency, control_task_delay_ticks, "Control loop", 
                       [this](frequency_t freq) { wheel_container_.updateControlLoopFrequency(freq); });
}

void TaskManager::controlLoopTaskActionNonBlocking(TaskAction action)
{
    enqueueTaskStateCommand(TaskType::ControlLoop, action);
}

void TaskManager::odoBroadcastTaskActionNonBlocking(TaskAction action)
{
    enqueueTaskStateCommand(TaskType::OdoBroadcast, action);
}

template<typename TaskFunc>
bool TaskManager::createTask(std::unique_ptr<Task>& task_ptr, 
                           const char* name,
                           UBaseType_t stack_size,
                           UBaseType_t priority,
                           TaskFunc&& task_function,
                           StackType_t* stack_buffer,
                           StaticTask_t* tcb_buffer,
                           SemaphoreHandle_t suspension_mutex)
{
    if (task_ptr != nullptr)
    {
        ESP_LOGW(TAG, "%s task already exists", name);
        return false;
    }

    Task::Config config{
        .name = name,
        .stack_size = stack_size,
        .priority = priority,
        .core_id = tskNO_AFFINITY,
        .use_static_allocation = true};

    task_ptr = std::make_unique<Task>(
        config,
        std::forward<TaskFunc>(task_function),
        stack_buffer,
        tcb_buffer);

    // Set suspension mutex for coordinated shutdown
    if (suspension_mutex)
    {
        task_ptr->setSuspensionMutex(suspension_mutex);
    }

    return task_ptr->start();
}

bool TaskManager::createControlTask()
{
    return createTask(control_task_, 
                     "ControlTask",
                     CONTROL_TASK_STACK_SIZE,
                     CONTROL_TASK_PRIORITY,
                     [this]() { this->controlTask(); },
                     control_task_stack,
                     &control_task_tcb,
                     control_loop_mutex);
}

bool TaskManager::createOdoBroadcastTask()
{
    return createTask(odo_broadcast_task_,
                     "OdoBroadcastTask", 
                     ODO_BROADCAST_TASK_STACK_SIZE,
                     ODO_BROADCAST_TASK_PRIORITY,
                     [this]() { this->odoBroadcastTask(); },
                     odo_broadcast_task_stack,
                     &odo_broadcast_task_tcb,
                     odo_broadcast_mutex);
}

bool TaskManager::controlLoopTaskAction(TaskAction action)
{
    using enum TaskAction;
    switch (action)
    {
    case Start:
        if (control_task_ == nullptr)
        {
            createControlTask();
        }
        return control_task_ != nullptr;

    case Stop:
        return control_task_ ? control_task_->stop() : true;

    case Suspend:
        return control_task_ ? control_task_->suspend() : false;

    case Resume:
        return control_task_ ? control_task_->resume() : false;
    }
    return false;
}

bool TaskManager::odoBroadcastTaskAction(TaskAction action)
{
    using enum TaskAction;
    switch (action)
    {
    case Start:
        if (odo_broadcast_task_ == nullptr)
        {
            createOdoBroadcastTask();
        }
        return odo_broadcast_task_ != nullptr;

    case Stop:
        return odo_broadcast_task_ ? odo_broadcast_task_->stop() : true;

    case Suspend:
        return odo_broadcast_task_ ? odo_broadcast_task_->suspend() : false;

    case Resume:
        return odo_broadcast_task_ ? odo_broadcast_task_->resume() : false;
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
    
    // Process all pending commands in the queue
    while (xQueueReceive(task_state_queue_, &command, 0) == pdPASS)
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
    }
}