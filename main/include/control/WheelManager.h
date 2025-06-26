#pragma once

#include <memory>
#include <functional>

#include "utils/MyStructs.h"
#include "control/WheelContainer.h"
#include "core/TaskManager.h"

// Forward declarations

enum class TaskAction;

struct WheelManagerConfig
{
    frequency_t control_loop_frequency;
    frequency_t odo_broadcast_frequency;
};

/**
 * @brief Facade class that combines WheelContainer and TaskManager to maintain the original interface
 */
class WheelManager
{
public:
    WheelManager(WheelManagerConfig config);
    ~WheelManager() = default;

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
    void setOdoBroadcastCallback(std::function<void(const std::pair<timestamp_t, std::vector<odometry_t>> &)> cb);

private:
    WheelContainer wheel_container_;
    TaskManager task_manager_;
};
