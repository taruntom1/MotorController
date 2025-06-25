#include "Task.h"
#include <cassert>

Task::Task(const Config& config, TaskFunction task_function)
    : config_(config), 
      task_function_(std::move(task_function)),
      use_static_allocation_(false),
      static_stack_buffer_(nullptr),
      static_task_buffer_(nullptr)
{
    assert(task_function_ && "Task function cannot be null");
    assert(config_.stack_size > 0 && "Stack size must be greater than 0");
    assert(!config_.name.empty() && "Task name cannot be empty");
}

Task::Task(const Config& config, TaskFunction task_function, 
           StackType_t* stack_buffer, StaticTask_t* task_buffer)
    : config_(config), 
      task_function_(std::move(task_function)),
      use_static_allocation_(true),
      static_stack_buffer_(stack_buffer),
      static_task_buffer_(task_buffer)
{
    assert(task_function_ && "Task function cannot be null");
    assert(config_.stack_size > 0 && "Stack size must be greater than 0");
    assert(!config_.name.empty() && "Task name cannot be empty");
    assert(stack_buffer && "Stack buffer cannot be null for static allocation");
    assert(task_buffer && "Task buffer cannot be null for static allocation");
}

Task::~Task()
{
    // Ensure task is stopped before destruction
    if (getState() != TaskState::Stopped)
    {
        ESP_LOGW(TAG, "Destroying active '%s' - force stop", config_.name.c_str());
        stop(2000); // Give more time for cleanup in destructor
    }
}

bool Task::start()
{
    TaskState expected = TaskState::Created;
    if (!state_.compare_exchange_strong(expected, TaskState::Running, std::memory_order_acq_rel))
    {
        if (expected == TaskState::Stopped)
        {
            // Allow restart from stopped state
            setState(TaskState::Running);
        }
        else
        {
            ESP_LOGW(TAG, "Task '%s' cannot start from %s state", 
                     config_.name.c_str(), stateToString(expected));
            return false;
        }
    }

    TaskHandle_t handle = nullptr;
    
    if (use_static_allocation_)
    {
        handle = xTaskCreateStatic(
            taskEntry,
            config_.name.c_str(),
            config_.stack_size,
            this,
            config_.priority,
            static_stack_buffer_,
            static_task_buffer_
        );
    }
    else
    {
        BaseType_t result;
        if (config_.core_id == tskNO_AFFINITY)
        {
            result = xTaskCreate(
                taskEntry,
                config_.name.c_str(),
                config_.stack_size,
                this,
                config_.priority,
                &handle
            );
        }
        else
        {
            result = xTaskCreatePinnedToCore(
                taskEntry,
                config_.name.c_str(),
                config_.stack_size,
                this,
                config_.priority,
                &handle,
                config_.core_id
            );
        }
        
        if (result != pdPASS)
        {
            handle = nullptr;
        }
    }

    if (handle != nullptr)
    {
        task_handle_.store(handle, std::memory_order_release);
        ESP_LOGI(TAG, "Started '%s' (prio:%u, stack:%u)", config_.name.c_str(), config_.priority, config_.stack_size);
        return true;
    }
    else
    {
        setState(TaskState::Stopped);
        ESP_LOGE(TAG, "Create failed: '%s'", config_.name.c_str());
        return false;
    }
}

bool Task::stop(uint32_t timeout_ms)
{
    TaskHandle_t handle = task_handle_.load(std::memory_order_acquire);
    TaskState current_state = getState();
    
    if (handle == nullptr || current_state == TaskState::Stopped)
    {
        setState(TaskState::Stopped);
        return true;
    }

    // First try to suspend gracefully if task is running
    if (current_state == TaskState::Running && !waitForSafeSuspension(timeout_ms))
    {
        ESP_LOGW(TAG, "Suspend timeout, force delete '%s'", config_.name.c_str());
    }

    // Delete the FreeRTOS task
    vTaskDelete(handle);
    task_handle_.store(nullptr, std::memory_order_release);
    setState(TaskState::Stopped);
    
    ESP_LOGI(TAG, "Stopped '%s'", config_.name.c_str());
    return true;
}

bool Task::suspend(uint32_t timeout_ms)
{
    TaskState expected = TaskState::Running;
    if (!state_.compare_exchange_strong(expected, TaskState::Suspended, std::memory_order_acq_rel))
    {
        ESP_LOGW(TAG, "Cannot suspend '%s' from %s", config_.name.c_str(), stateToString(expected));
        return false;
    }

    if (waitForSafeSuspension(timeout_ms))
    {
        TaskHandle_t handle = task_handle_.load(std::memory_order_acquire);
        if (handle != nullptr)
        {
            vTaskSuspend(handle);
            ESP_LOGI(TAG, "Suspended '%s'", config_.name.c_str());
            return true;
        }
    }

    // Revert state if suspension failed
    setState(TaskState::Running);
    ESP_LOGE(TAG, "Suspend failed: '%s'", config_.name.c_str());
    return false;
}

bool Task::resume()
{
    TaskState expected = TaskState::Suspended;
    if (!state_.compare_exchange_strong(expected, TaskState::Running, std::memory_order_acq_rel))
    {
        ESP_LOGW(TAG, "Cannot resume '%s' from %s", config_.name.c_str(), stateToString(expected));
        return false;
    }

    TaskHandle_t handle = task_handle_.load(std::memory_order_acquire);
    if (handle != nullptr)
    {
        vTaskResume(handle);
        ESP_LOGI(TAG, "Resumed '%s'", config_.name.c_str());
        return true;
    }

    // Revert state if resume failed
    setState(TaskState::Suspended);
    ESP_LOGE(TAG, "Resume failed: '%s'", config_.name.c_str());
    return false;
}

void Task::taskEntry(void* pvParameters)
{
    Task* task = static_cast<Task*>(pvParameters);
    assert(task && "Task parameter cannot be null");
    
    task->taskWrapper();
}

void Task::taskWrapper()
{
    ESP_LOGI(TAG, "Exec '%s'", config_.name.c_str());
    
    try
    {
        // Execute the user-provided task function
        task_function_();
    }
    catch (...)
    {
        ESP_LOGE(TAG, "Unhandled exception '%s'", config_.name.c_str());
    }
    
    // Task function has completed - mark as stopped
    task_handle_.store(nullptr, std::memory_order_release);
    setState(TaskState::Stopped);
    
    ESP_LOGI(TAG, "Done '%s'", config_.name.c_str());
    
    // Delete self - this should be the last thing we do
    vTaskDelete(nullptr);
}

bool Task::waitForSafeSuspension(uint32_t timeout_ms)
{
    if (suspension_mutex_ == nullptr)
    {
        // No coordination mutex - just give a small delay for current operation to complete
        vTaskDelay(pdMS_TO_TICKS(10));
        return true;
    }

    // Try to acquire the mutex to ensure task is not in critical section
    const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    
    if (xSemaphoreTake(suspension_mutex_, timeout_ticks) == pdTRUE)
    {
        // Task is not in critical section, safe to proceed
        xSemaphoreGive(suspension_mutex_);
        return true;
    }
    else
    {
        ESP_LOGW(TAG, "Suspend wait timeout '%s'", config_.name.c_str());
        return false;
    }
}

const char* Task::stateToString(TaskState state) const
{
    switch (state)
    {
        case TaskState::Created:   return "Created";
        case TaskState::Running:   return "Running";
        case TaskState::Suspended: return "Suspended";
        case TaskState::Stopped:   return "Stopped";
        default:                   return "Unknown";
    }
}

void Task::setState(TaskState new_state)
{
    state_.store(new_state, std::memory_order_release);
}

