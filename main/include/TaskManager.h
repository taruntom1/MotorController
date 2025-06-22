#pragma once

#include <atomic>
#include <functional>

#include "MyStructs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Forward declaration
class WheelContainer;

struct TaskManagerConfig
{
    frequency_t control_loop_frequency;
    frequency_t odo_broadcast_frequency;
};

enum class TaskAction {
    Start,
    Stop,
    Suspend,
    Resume
};

enum class TaskState {
    Running,
    Suspended,
    Deleted
};

enum class task_manager_notifications : uint32_t {
    // Wheel updates
    NUM_WHEEL_UPDATE = (1 << 0),
    WHEEL_UPDATE = (1 << 1),
    CONTROL_MODE_UPDATE = (1 << 2),

    // Control loop notifications
    START_CONTROL_LOOP = (1 << 3),
    STOP_CONTROL_LOOP = (1 << 4),
    SUSPEND_CONTROL_LOOP = (1 << 5),
    RESUME_CONTROL_LOOP = (1 << 6),
    CONTROL_LOOP_SUSPENDED = (1 << 7),

    // Odometer broadcast notifications
    START_ODO_BROADCAST = (1 << 8),
    STOP_ODO_BROADCAST = (1 << 9),
    SUSPEND_ODO_BROADCAST = (1 << 10),
    RESUME_ODO_BROADCAST = (1 << 11),
    ODO_BROADCAST_SUSPENDED = (1 << 12),
};

/**
 * @brief Manages the three FreeRTOS tasks for wheel management, control loop, and odometry broadcast
 */
class TaskManager
{
public:
    TaskManager(TaskManagerConfig config, WheelContainer& wheelContainer);
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

    // Callbacks
    std::function<void(const std::pair<timestamp_t, std::vector<odometry_t>> &)> odoBroadcastCallback;
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

    WheelContainer& wheel_container_;

    // Task handles
    TaskHandle_t wheel_manage_task_handle = nullptr;
    TaskHandle_t control_task_handle = nullptr;
    TaskHandle_t odo_broadcast_task_handle = nullptr;

    TaskState control_task_state_ = TaskState::Deleted;
    TaskState odo_broadcast_task_state_ = TaskState::Deleted;

    // Task loop delays
    std::atomic<TickType_t> control_task_delay_ticks;
    std::atomic<TickType_t> odo_broadcast_task_delay_ticks;

    // Odometry broadcast mem cache
    std::pair<timestamp_t, std::vector<odometry_t>> odoBroadcastData;

    // State flags
    std::atomic<bool> control_loop_run{false};
    std::atomic<bool> odo_broadcast_run{false};

    // Tasks and related functions
    static void wheelManageTaskEntry(void *pvParameters);
    void wheelManageTask();

    static void controlTaskEntry(void *pvParameters);
    void controlTask();

    static void odoBroadcastTaskEntry(void *pvParameters);
    void odoBroadcastTask();

    bool createControlTask();
    bool createOdoBroadcastTask();

    bool handleTaskAction(TaskAction action,
                          TaskHandle_t &task_handle,
                          TaskState &task_state,
                          std::function<bool()> task_creator,
                          std::atomic<bool> &run_flag,
                          std::function<bool()> suspend_handler);
    void handleTaskActionNotification(uint32_t notification);

    bool suspendAndWaitForControlLoopSuspend();
    bool suspendAndWaitForOdoBroadcastSuspend();

    void notifyTaskManager(task_manager_notifications notification) {
        xTaskNotify(wheel_manage_task_handle, static_cast<uint32_t>(notification), eSetBits);
    }
};
