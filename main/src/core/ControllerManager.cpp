#include "core/ControllerManager.h"

ControllerManager& ControllerManager::getInstance()
{
    static ControllerManager instance;
    return instance;
}

ControllerManager::ControllerManager()
    : wheel_manager_({1, 1}),
      control_interface_({576000, UART_NUM_0, 2048, 44, 43, 0xAA, 100})
{
    connectCallbacks();

    wheel_manager_.controlLoopTaskActionNonBlocking(TaskAction::Start);
}

void ControllerManager::setControllerProperties(const controller_properties_t &controller_properties)
{
    wheel_manager_.updateWheelCount(controller_properties.numMotors);
    wheel_manager_.updateControlLoopFrequency(controller_properties.updateFrequencies.control_run_frequency);
    wheel_manager_.updateOdoBroadcastFrequency(controller_properties.updateFrequencies.odoBroadcastFrequency);
}

void ControllerManager::connectCallbacks()
{
    // From contorl_interface_ to ControllerManager
    control_interface_.setControllerPropertiesCallback([this](const controller_properties_t &properties)
                                                       { this->setControllerProperties(properties); });

    // From wheel_manager_ to control_interface_
    wheel_manager_.setOdoBroadcastCallback([this](const std::pair<timestamp_t, std::vector<odometry_t>> &data)
                                           { this->control_interface_.SendOdoData(data); });

    // From control_interface_ to wheel_manager_
    control_interface_.setWheelDataCallback(
        [this](const wheel_data_t &data)
        {
            wheel_manager_.updateWheel(data);
        });

    control_interface_.setWheelControlModeCallback(
        [this](uint8_t id, ControlMode mode)
        {
            wheel_manager_.updateControlMode(id, mode);
        });

    control_interface_.setOdoBroadcastCallbackNonBlocking(
        [this](TaskAction action)
        {
            wheel_manager_.odoBroadcastTaskActionNonBlocking(action);
        });
    control_interface_.setOdoBroadcastCallbackBlocking(
        [this](TaskAction action)
        {
            wheel_manager_.odoBroadcastTaskAction(action);
        });
    control_interface_.setPIDConstantsCallback(
        [this](uint8_t id, PIDType type, pid_constants_t constants)
        {
            wheel_manager_.updatePIDConstants(id, type, constants);
        });

    control_interface_.setWheelSetpointCallback(
        [this](const std::vector<float> &setpoints)
        {
            wheel_manager_.updateSetpoints(setpoints);
        });
}
