#pragma once

#include <vector>
#include <utility>
#include <atomic>

#include "Wheel.h"
#include "ControlInterface.h"
#include "MyStructs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

struct WheelManagerConfig
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

enum class wheel_manager_notifications : uint32_t {
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
 * @brief Manages dynamic creation and reinitialization of Wheel objects based on task notifications.
 */
class WheelManager
{
public:
    WheelManager(WheelManagerConfig config);
    ~WheelManager();

    void updateWheelCount(uint8_t count);
    void updateWheel(const wheel_data_t &wheel);
    void updateControlMode(uint8_t id, ControlMode mode);
    void updateOdoBroadcastFrequency(frequency_t frequency);
    void updateControlLoopFrequency(frequency_t frequency);
    void updatePIDConstants(uint8_t id, PIDType type, pid_constants_t constants);
    void updateSetpoints(const std::vector<float> &setpoints);

    // Task actions : Blocking
    bool controlLoopTaskAction(TaskAction action);
    bool odoBroadcastTaskAction(TaskAction action);
    // Task actions : Non-blocking
    void controlLoopTaskActionNonBlocking(TaskAction action);
    void odoBroadcastTaskActionNonBlocking(TaskAction action);

    // callbacks
    std::function<void(const std::pair<timestamp_t, std::vector<odometry_t>> &)> odoBroadcastCallback;
    void setOdoBroadcastCallback(std::function<void(const std::pair<timestamp_t, std::vector<odometry_t>> &)> cb)
    {
        odoBroadcastCallback = std::move(cb);
    }

private:
    static constexpr char *TAG = "WheelManager";

    // Task handles
    TaskHandle_t wheel_manage_task_handle = nullptr;
    TaskHandle_t control_task_handle = nullptr;
    TaskHandle_t odo_broadcast_task_handle = nullptr;

    TaskState control_task_state_ = TaskState::Deleted;
    TaskState odo_broadcast_task_state_ = TaskState::Deleted;

    // Task loop delays
    std::atomic<TickType_t> control_task_delay_ticks;
    std::atomic<TickType_t> odo_broadcast_task_delay_ticks;

    std::vector<std::optional<Wheel>> wheels_;
    uint8_t wheel_count_{0};

    // Queue handles
    QueueHandle_t wheel_data_queue = nullptr;
    QueueHandle_t control_mode_queue = nullptr;

    // Odobroadcast mem cache
    std::pair<timestamp_t, std::vector<odometry_t>> odoBroadcastData;

    // state flags
    std::atomic<bool> control_loop_run{false};
    std::atomic<bool> odo_broadcast_run{false};

    // Wheel details
    uint16_t needsOdoUpdate_ = 0; ///< bit mask of wheels that need odometry update

    void changeWheelCount();
    void changeRequestedWheels();
    void changeControlMode();

    // Tasks and related functions
    static void wheelManageTaskEntry(void *pvParameters);
    void WheelManageTask();

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

    void notifyWheelManager(wheel_manager_notifications notification) {
        xTaskNotify(wheel_manage_task_handle, static_cast<uint32_t>(notification), eSetBits);
    }
};
