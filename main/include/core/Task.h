#pragma once

#include <functional>
#include <memory>
#include <atomic>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

enum class TaskState
{
    Created,    // Task object created but FreeRTOS task not started
    Running,    // FreeRTOS task is running
    Suspended,  // FreeRTOS task is suspended
    Stopped     // FreeRTOS task has been deleted
};

/**
 * @brief RAII wrapper for FreeRTOS tasks with lifecycle management
 * 
 * This class encapsulates a FreeRTOS task as an object, providing:
 * - Automatic resource cleanup via RAII
 * - Thread-safe state management
 * - Graceful task suspension and resumption
 * - Static or dynamic memory allocation options
 */
class Task
{
public:
    using TaskFunction = std::function<void()>;
    
    struct Config
    {
        std::string name;
        UBaseType_t stack_size;
        UBaseType_t priority;
        BaseType_t core_id = tskNO_AFFINITY;  // Run on any core by default
        bool use_static_allocation = false;   // Use dynamic allocation by default
    };

    /**
     * @brief Construct a new Task object
     * @param config Task configuration
     * @param task_function Function to execute in the task
     */
    Task(const Config& config, TaskFunction task_function);
    
    /**
     * @brief Construct a new Task object with static allocation
     * @param config Task configuration (use_static_allocation will be set to true)
     * @param task_function Function to execute in the task
     * @param stack_buffer Pre-allocated stack buffer
     * @param task_buffer Pre-allocated task control block
     */
    Task(const Config& config, TaskFunction task_function, 
         StackType_t* stack_buffer, StaticTask_t* task_buffer);

    // Non-copyable and non-movable for safety
    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;
    Task(Task&&) = delete;
    Task& operator=(Task&&) = delete;

    /**
     * @brief Destructor - automatically stops the task
     */
    ~Task();

    /**
     * @brief Start the FreeRTOS task
     * @return true if task started successfully, false otherwise
     */
    bool start();

    /**
     * @brief Stop the FreeRTOS task (delete it)
     * @param timeout_ms Maximum time to wait for graceful shutdown
     * @return true if task stopped successfully, false otherwise
     */
    bool stop(uint32_t timeout_ms = 1000);

    /**
     * @brief Suspend the FreeRTOS task
     * @param timeout_ms Maximum time to wait for task to reach safe suspend point
     * @return true if task suspended successfully, false otherwise
     */
    bool suspend(uint32_t timeout_ms = 500);

    /**
     * @brief Resume the FreeRTOS task
     * @return true if task resumed successfully, false otherwise
     */
    bool resume();

    /**
     * @brief Get current task state
     * @return Current TaskState
     */
    TaskState getState() const { return state_.load(std::memory_order_acquire); }

    /**
     * @brief Get FreeRTOS task handle
     * @return TaskHandle_t or nullptr if task not running
     */
    TaskHandle_t getHandle() const { return task_handle_.load(std::memory_order_acquire); }

    /**
     * @brief Get task name
     * @return Task name string
     */
    const std::string& getName() const { return config_.name; }

    /**
     * @brief Check if task is running
     * @return true if task is in Running state
     */
    bool isRunning() const { return getState() == TaskState::Running; }

    /**
     * @brief Check if task is suspended
     * @return true if task is in Suspended state
     */
    bool isSuspended() const { return getState() == TaskState::Suspended; }

    /**
     * @brief Set a mutex for coordination during suspension
     * This mutex will be used to ensure safe suspension points
     * @param mutex Mutex handle to use for coordination
     */
    void setSuspensionMutex(SemaphoreHandle_t mutex) { suspension_mutex_ = mutex; }

private:
    static constexpr const char* TAG = "Task";
    
    Config config_;
    TaskFunction task_function_;
    std::atomic<TaskHandle_t> task_handle_{nullptr};
    std::atomic<TaskState> state_{TaskState::Created};
    
    // Static allocation support
    bool use_static_allocation_;
    StackType_t* static_stack_buffer_;
    StaticTask_t* static_task_buffer_;
    
    // For coordinated suspension
    SemaphoreHandle_t suspension_mutex_ = nullptr;
    
    // Internal task entry point
    static void taskEntry(void* pvParameters);
    
    // Internal task wrapper
    void taskWrapper();
    
    // Helper methods
    bool waitForSafeSuspension(uint32_t timeout_ms);
    const char* stateToString(TaskState state) const;
    void setState(TaskState new_state);
};