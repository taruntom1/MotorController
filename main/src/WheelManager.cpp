#include "WheelManager.h"
#include "WheelContainer.h"
#include "TaskManager.h"

WheelManager::WheelManager(WheelManagerConfig config)
    : wheel_container_(),
      task_manager_({config.control_loop_frequency, config.odo_broadcast_frequency}, wheel_container_)
{
}

void WheelManager::updateWheelCount(uint8_t count)
{
    wheel_container_.setPendingWheelCount(count);
    task_manager_.notifyWheelCountUpdate();
}

void WheelManager::updateWheel(const wheel_data_t &wheel)
{
    wheel_container_.queueWheelData(wheel);
    task_manager_.notifyWheelUpdate();
}

void WheelManager::updateControlMode(uint8_t id, ControlMode mode)
{
    wheel_container_.queueControlMode(id, mode);
    task_manager_.notifyControlModeUpdate();
}

void WheelManager::updateOdoBroadcastFrequency(frequency_t frequency)
{
    task_manager_.updateOdoBroadcastFrequency(frequency);
}

void WheelManager::updateControlLoopFrequency(frequency_t frequency)
{
    task_manager_.updateControlLoopFrequency(frequency);
}

void WheelManager::updatePIDConstants(uint8_t id, PIDType type, pid_constants_t constants)
{
    wheel_container_.updatePIDConstants(id, type, constants);
}

void WheelManager::updateSetpoints(const std::vector<float> &setpoints)
{
    wheel_container_.updateSetpoints(setpoints);
}

bool WheelManager::controlLoopTaskAction(TaskAction action)
{
    return task_manager_.controlLoopTaskAction(action);
}

bool WheelManager::odoBroadcastTaskAction(TaskAction action)
{
    return task_manager_.odoBroadcastTaskAction(action);
}

void WheelManager::controlLoopTaskActionNonBlocking(TaskAction action)
{
    task_manager_.controlLoopTaskActionNonBlocking(action);
}

void WheelManager::odoBroadcastTaskActionNonBlocking(TaskAction action)
{
    task_manager_.odoBroadcastTaskActionNonBlocking(action);
}

void WheelManager::setOdoBroadcastCallback(std::function<void(const std::pair<timestamp_t, std::vector<odometry_t>> &)> cb)
{
    task_manager_.setOdoBroadcastCallback(std::move(cb));
}
