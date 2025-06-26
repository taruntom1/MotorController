#pragma once

#include <atomic>
#include <functional>
#include <memory>

#include "utils/MyStructs.h"
#include "core/Task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// Forward declaration
class WheelContainer;

struct TaskManagerConfig
{
    frequency_t control_loop_frequency;
    frequency_t odo_broadcast_frequency;
};

enum class TaskAction
{
    Start,
    Stop,
    Suspend,
    Resume
};

enum class TaskType
{
    ControlLoop,
    OdoBroadcast
};

struct TaskStateCommand
{
    TaskType task_type;
    TaskAction action;
};

enum class task_manager_notifications : uint32_t
{
    // Wheel updates
    NUM_WHEEL_UPDATE = (1 << 0),
    WHEEL_UPDATE = (1 << 1),
    CONTROL_MODE_UPDATE = (1 << 2),

    // Task state queue processing
    PROCESS_TASK_STATE_QUEUE = (1 << 3),
};

/**
 * @brief Manages the three FreeRTOS tasks for wheel management, control loop, and odometry broadcast
 */
class TaskManager
{
public:
    TaskManager(TaskManagerConfig config, WheelContainer &wheelContainer);
    ~TaskManager();

    // Task actions : Blocking (direct execution)
    bool controlLoopTaskAction(TaskAction action);
    bool odoBroadcastTaskAction(TaskAction action);
    // Task actions : Non-blocking (queued execution via wheel manager task)
    void controlLoopTaskActionNonBlocking(TaskAction action);
    void odoBroadcastTaskActionNonBlocking(TaskAction action);

    // Frequency updates
    void updateOdoBroadcastFrequency(frequency_t frequency);
    void updateControlLoopFrequency(frequency_t frequency);

    // Notification interface
    void notifyWheelUpdate() { notifyTaskManager(task_manager_notifications::WHEEL_UPDATE); }
    void notifyWheelCountUpdate() { notifyTaskManager(task_manager_notifications::NUM_WHEEL_UPDATE); }
    void notifyControlModeUpdate() { notifyTaskManager(task_manager_notifications::CONTROL_MODE_UPDATE); }

    void setOdoBroadcastCallback(std::function<void(const std::pair<timestamp_t, std::vector<odometry_t>> &)> cb)
    {
        odoBroadcastCallback = std::move(cb);
    }

private:
    void updateTaskFrequency(frequency_t frequency, 
                           std::atomic<TickType_t>& delay_ticks, 
                           const char* task_name,
                           std::function<void(frequency_t)> additional_action = nullptr);

    static constexpr const char *TAG = "TaskManager";
    static constexpr UBaseType_t WHEEL_MANAGE_TASK_STACK_SIZE = 4096;
    static constexpr UBaseType_t WHEEL_MANAGE_TASK_PRIORITY = 6;
    static constexpr UBaseType_t CONTROL_TASK_STACK_SIZE = 3072;
    static constexpr UBaseType_t CONTROL_TASK_PRIORITY = 5;
    static constexpr UBaseType_t ODO_BROADCAST_TASK_STACK_SIZE = 2048;
    static constexpr UBaseType_t ODO_BROADCAST_TASK_PRIORITY = 4;
    static constexpr UBaseType_t TASK_STATE_QUEUE_LENGTH = 10;

    WheelContainer &wheel_container_;

    // Task state queue
    QueueHandle_t task_state_queue_;

    // Task wrapper objects
    std::unique_ptr<Task> wheel_manage_task_;
    std::unique_ptr<Task> control_task_;
    std::unique_ptr<Task> odo_broadcast_task_;

    // Static memory buffers for tasks
    StaticTask_t wheel_manage_task_tcb;
    StaticTask_t control_task_tcb;
    StaticTask_t odo_broadcast_task_tcb;

    StackType_t wheel_manage_task_stack[WHEEL_MANAGE_TASK_STACK_SIZE];
    StackType_t control_task_stack[CONTROL_TASK_STACK_SIZE];
    StackType_t odo_broadcast_task_stack[ODO_BROADCAST_TASK_STACK_SIZE];

    // Task loop delays
    std::atomic<TickType_t> control_task_delay_ticks;
    std::atomic<TickType_t> odo_broadcast_task_delay_ticks;

    // Odometry broadcast mem cache
    std::pair<timestamp_t, std::vector<odometry_t>> odoBroadcastData;

    // Task suspension control (mutex-based)
    SemaphoreHandle_t control_loop_mutex;
    SemaphoreHandle_t odo_broadcast_mutex;
    StaticSemaphore_t control_loop_mutex_buffer;
    StaticSemaphore_t odo_broadcast_mutex_buffer;

    // Tasks and related functions
    void wheelManageTask();
    void controlTask();
    void odoBroadcastTask();

    // Callbacks
    std::function<void(const std::pair<timestamp_t, std::vector<odometry_t>> &)> odoBroadcastCallback;

    bool createControlTask();
    bool createOdoBroadcastTask();
    
    template<typename TaskFunc>
    bool createTask(std::unique_ptr<Task>& task_ptr, 
                   const char* name,
                   UBaseType_t stack_size,
                   UBaseType_t priority,
                   TaskFunc&& task_function,
                   StackType_t* stack_buffer,
                   StaticTask_t* tcb_buffer,
                   SemaphoreHandle_t suspension_mutex = nullptr);

    void processTaskStateQueue();

    void notifyTaskManager(task_manager_notifications notification)
    {
        if (wheel_manage_task_ && wheel_manage_task_->getHandle())
        {
            xTaskNotify(wheel_manage_task_->getHandle(), static_cast<uint32_t>(notification), eSetBits);
        }
    }

    bool enqueueTaskStateCommand(TaskType task_type, TaskAction action);

    // Helper for mutex-protected processing without full task suspension
    void mutexProtectedProcessHelper(const std::function<void()> &processFunc);
};
