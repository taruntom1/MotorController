#pragma once

#include <vector>
#include <utility>
#include <atomic>

#include "Wheel.h"
#include "ControlInterface.h"
#include "MyStructs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

enum class wheel_manager_notifications : uint32_t
{
    NUM_WHEEL_UPDATE = (1 << 0),
    WHEEL_UPDATE = (1 << 1),
    CONTROL_MODE_UPDATE = (1 << 2),
    CONTROL_LOOP_SUSPENDED = (1 << 3),
    ODO_BROADCAST_SUSPENDED = (1 << 4),
    START_ODO_BROADCAST = (1 << 5),
    STOP_ODO_BROADCAST = (1 << 6),
    START_CONTROL_LOOP = (1 << 7),
    STOP_CONTROL_LOOP = (1 << 8)
};

/**
 * @brief Manages dynamic creation and reinitialization of Wheel objects based on task notifications.
 */
class WheelManager
{
public:
    WheelManager();

    void Run();

    static void wheelManageTaskEntry(void *pvParameters);
    void WheelManageTask();

    static void controlTaskEntry(void *pvParameters);
    void controlTask();

    static void odoBroadcastTaskEntry(void *pvParameters);
    void odoBroadcastTask();

    void updateWheelCount(uint8_t count);
    void updateWheel(Wheel &wheel);
    void updateControlMode(uint8_t id, ControlMode mode);
    void updateOdoBroadcast(bool enabled);
    void updatePIDConstants(uint8_t id, PIDType type, pid_constants_t constants);
    void updateSetpoints(std::vector<float> &setpoints);

    // callbacks
    std::function<void(const std::vector<std::pair<timestamp_t, odometry_t>> &)> odoBroadcastCallback;
    void setOdoBroadcastCallback(std::function<void(const std::vector<std::pair<timestamp_t, odometry_t>> &)> cb)
    {
        odoBroadcastCallback = std::move(cb);
    }

private:
    static constexpr char *TAG = "WheelManager";

    // Task handles
    TaskHandle_t wheel_manage_task_handle;
    TaskHandle_t control_task_handle;
    TaskHandle_t odo_broadcast_task_handle;

    std::vector<Wheel> wheels_;
    uint8_t wheel_count_{0};

    // Queue handles
    QueueHandle_t wheel_data_queue;
    QueueHandle_t control_mode_queue;

    // state flags
    std::atomic<bool> control_loop_run{false};
    std::atomic<bool> odo_broadcast_run{false};

    // Wheel details
    uint16_t needsOdoUpdate_ = 0; ///< bit mask of wheels that need odometry update

    void changeWheelCount();
    void changeRequestedWheels();
    void changeControlMode();

    void startOdoBroadcast();
    void startControlLoop();
    bool stopControlLoop();
    bool stopOdoBroadcast();

    bool suspendAndWaitForControlLoopSuspend();
    bool suspendAndWaitForOdoBroadcastSuspend();
};
