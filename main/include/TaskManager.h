#pragma once

#include <atomic>
#include <functional>

#include "MyStructs.h"
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

enum class TaskState
{
    Running,
    Suspended,
    Deleted
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

    // Task actions : Blocking
    bool controlLoopTaskAction(TaskAction action);
    bool odoBroadcastTaskAction(TaskAction action);
    // Task actions : Non-blocking
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
    static constexpr const char *TAG = "TaskManager";
    static constexpr UBaseType_t WHEEL_MANAGE_TASK_STACK_SIZE = 4096;
    static constexpr UBaseType_t WHEEL_MANAGE_TASK_PRIORITY = 6;
    static constexpr UBaseType_t CONTROL_TASK_STACK_SIZE = 3072;
    static constexpr UBaseType_t CONTROL_TASK_PRIORITY = 5;
    static constexpr UBaseType_t ODO_BROADCAST_TASK_STACK_SIZE = 2048;
    static constexpr UBaseType_t ODO_BROADCAST_TASK_PRIORITY = 4;
    static constexpr TickType_t TASK_SUSPENSION_TIMEOUT_MS = 500;
    static constexpr UBaseType_t TASK_STATE_QUEUE_LENGTH = 10;

    WheelContainer &wheel_container_;

    // Task state queue
    QueueHandle_t task_state_queue_;

    // Task handles
    TaskHandle_t wheel_manage_task_handle = nullptr;
    TaskHandle_t control_task_handle = nullptr;
    TaskHandle_t odo_broadcast_task_handle = nullptr;

    // Static memory buffers for tasks
    StaticTask_t wheel_manage_task_tcb;
    StaticTask_t control_task_tcb;
    StaticTask_t odo_broadcast_task_tcb;
    
    StackType_t wheel_manage_task_stack[WHEEL_MANAGE_TASK_STACK_SIZE];
    StackType_t control_task_stack[CONTROL_TASK_STACK_SIZE];
    StackType_t odo_broadcast_task_stack[ODO_BROADCAST_TASK_STACK_SIZE];

    TaskState control_task_state_ = TaskState::Deleted;
    TaskState odo_broadcast_task_state_ = TaskState::Deleted;

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
    static void wheelManageTaskEntry(void *pvParameters);
    [[noreturn]] void wheelManageTask();

    static void controlTaskEntry(void *pvParameters);
    [[noreturn]] void controlTask();

    static void odoBroadcastTaskEntry(void *pvParameters);
    [[noreturn]] void odoBroadcastTask();

    // Callbacks
    std::function<void(const std::pair<timestamp_t, std::vector<odometry_t>> &)> odoBroadcastCallback;

    bool createControlTask();
    bool createOdoBroadcastTask(); // Helper method for common task creation logic
    bool createTaskHelper(TaskFunction_t taskFunction,
                          const char *taskName,
                          UBaseType_t stackSize,
                          UBaseType_t priority,
                          TaskHandle_t &taskHandle,
                          TaskState &taskState,
                          StackType_t *stackBuffer,
                          StaticTask_t *taskBuffer);

    bool handleTaskAction(TaskAction action,
                          TaskHandle_t &task_handle,
                          TaskState &task_state,
                          std::function<bool()> task_creator,
                          std::function<bool()> suspend_handler);
    void processTaskStateQueue();

    bool suspendAndWaitForControlLoopSuspend();
    bool suspendAndWaitForOdoBroadcastSuspend();
    bool suspendAndWaitForTaskSuspend(SemaphoreHandle_t mutex, 
                                      TaskHandle_t task_handle,
                                      const char *timeout_log_tag);

    void notifyTaskManager(task_manager_notifications notification)
    {
        xTaskNotify(wheel_manage_task_handle, static_cast<uint32_t>(notification), eSetBits);
    } // Helper methods for logging
    const char *taskActionToString(TaskAction action) const;
    const char *taskStateToString(TaskState state) const;
    const char *taskTypeToString(TaskType type) const;
    const char *getTaskName(TaskHandle_t &task_handle) const;

    bool enqueueTaskStateCommand(TaskType task_type, TaskAction action);

    // Helper for mutex-protected processing without full task suspension
    void mutexProtectedProcessHelper(const std::function<void()> &processFunc);
};
