#include "ControlInterface.h"
#include "esp_log.h"
#include <string.h>

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG // Set local log level for this file

static const char *TAG = "ControlInterface";

ControlInterface::ControlInterface(protocol_config config)
    : protocol(config)
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Initializing control interface");
    protocol.begin();

    protocol.onCommandReceived = [this](Command command)
    {
        this->CallFunction(command);
    };

    cacheVctr.reserve(100);
}

void ControlInterface::Ping()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Ping sent");
    protocol.SendCommand(Command::PING);
}

void ControlInterface::start()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Starting motor controller");
    ControllerRunCallback(true);
    protocol.SendCommand(Command::READ_SUCCESS);
}
void ControlInterface::stop()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Stopping motor controller");
    ControllerRunCallback(false);
    protocol.SendCommand(Command::READ_SUCCESS);
}

bool ControlInterface::GetControllerProperties()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Reading controller properties");
    std::vector<uint8_t> data = protocol.ReadData(controller_properties_t::size, 1000);
    if (unlikely(data.empty()))
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN, TAG, "Controller properties read failed");
        protocol.SendCommand(Command::READ_FAILURE);
        return false;
    }

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Controller properties read successfully");
    size_t offset = 0;
    controller_properties_t properties;
    properties.from_bytes(data, offset);

    wheel_count = properties.numMotors;
    odo_broadcast_flags.resize(wheel_count);

    ControllerPropertiesCallback(properties);
    protocol.SendCommand(Command::READ_SUCCESS);
    return true;
}

bool ControlInterface::GetWheelData()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Getting motor data...");
    constexpr size_t dataSize = sizeof(uint8_t) + wheel_data_t::size;
    size_t offset = 1;

    std::vector<uint8_t> buffer = protocol.ReadData(dataSize, 1000);
    if (unlikely(buffer.empty()))
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN, TAG, "Motor data read failed");
        protocol.SendCommand(Command::READ_FAILURE);
        return false;
    }

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Motor ID and Motor Data received");
    uint8_t wheel_id = buffer[0];
    if (unlikely(wheel_id >= wheel_count))
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Motor ID out of range");
        protocol.SendCommand(Command::READ_FAILURE);
        return false;
    }

    wheel_data_t wheelData;
    wheelData.from_bytes(buffer, offset);

    assert(wheelData.motor_id == wheel_id);

    WheelDataCallback(wheelData);

    odo_broadcast_flags.at(wheel_id) = odo_broadcast_flag;
    refreshBroadcastStatus();

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Motor data read successful for motor ID %d", wheel_id);
    protocol.SendCommand(Command::READ_SUCCESS);
    return true;
}

void ControlInterface::stopAllBroadcast()
{
    OdoBroadcastCallbackBlocking(TaskAction::Suspend);
    protocol.SendCommand(Command::READ_SUCCESS);
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "All broadcasts stopped");
}

void ControlInterface::restoreAllBroadcast()
{
    refreshBroadcastStatus();
    // if (odo_broadcast_flag.angle || odo_broadcast_flag.speed || odo_broadcast_flag.pwm_value)
    OdoBroadcastCallbackBlocking(TaskAction::Resume);
    protocol.SendCommand(Command::READ_SUCCESS);

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "All broadcasts restored");
}

void ControlInterface::refreshBroadcastStatus()
{
    for (auto &flag : odo_broadcast_flags)
    {
        odo_broadcast_flag |= flag;
    }
}

bool ControlInterface::GetOdoBroadcastStatus()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Reading odo broadcast status");
    constexpr size_t buffer_size = sizeof(uint8_t) + odo_broadcast_flags_t::size;
    std::vector<uint8_t> buffer = protocol.ReadData(buffer_size, 1000);

    // Read motorID + broadcast flags in one read
    if (unlikely(buffer.empty()))
    {
        protocol.SendCommand(Command::READ_FAILURE);
        return false;
    }

    uint8_t motorID = buffer[0];
    if (unlikely(motorID >= wheel_count))
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN, TAG, "Motor ID out of range");
        protocol.SendCommand(Command::READ_FAILURE);
        return false;
    }

    size_t offset = 1;

    odo_broadcast_flags[motorID].from_bytes(buffer, offset);
    protocol.SendCommand(Command::READ_SUCCESS);

    // Update aggregated broadcast status
    odo_broadcast_flags_t &aggregatedStatus = odo_broadcast_flag;
    memset(&aggregatedStatus, 0, sizeof(odo_broadcast_flags_t));

    for (auto &odoBroadcastStatus : odo_broadcast_flags)
    {
        aggregatedStatus |= odoBroadcastStatus;
    }

    if (odo_broadcast_flag.angle || odo_broadcast_flag.speed || odo_broadcast_flag.pwm_value)
        OdoBroadcastCallbackNonBlocking(TaskAction::Start);
    else
        OdoBroadcastCallbackNonBlocking(TaskAction::Stop);

    return true;
}

bool ControlInterface::GetPIDConstants()
{
    constexpr size_t buffer_size = sizeof(uint8_t) * 2 + pid_constants_t::size;
    std::vector<uint8_t> buffer = protocol.ReadData(buffer_size, 1000);

    if (unlikely(buffer.empty()))
    {
        protocol.SendCommand(Command::READ_FAILURE);
        return false;
    }

    uint8_t &motorID = buffer[0];
    uint8_t &pidType = buffer[1];
    size_t offset = 2;

    // Validate motor ID
    if (unlikely((motorID >= wheel_count) || pidType >= 2))
    {
        protocol.SendCommand(Command::READ_FAILURE);
        return false;
    }

    pid_constants_t constants;
    constants.from_bytes(buffer, offset);

    // Store PID constants
    PIDConstantsCallback(motorID, static_cast<PIDType>(pidType), constants);

    protocol.SendCommand(Command::READ_SUCCESS);
    return true;
}

bool ControlInterface::GetWheelControlMode()
{
    constexpr size_t buffer_size = sizeof(uint8_t) + sizeof(ControlMode);
    std::vector<uint8_t> buffer(protocol.ReadData(buffer_size, 1000));
    if (unlikely(buffer.empty()))
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Failed to read motor ID and control mode");
        return false;
    }

    uint8_t &motorID = buffer[0];
    size_t offset = 1;
    if (unlikely(motorID >= wheel_count))
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Invalid motor ID %d", motorID);
        return false;
    }
    ControlMode control_mode;
    from_bytes(control_mode, buffer, offset);
    WheelControlModeCallback(motorID, control_mode);
    protocol.SendCommand(Command::READ_SUCCESS);
    return true;
}

bool ControlInterface::GetMotorSetpoints()
{
    std::vector<float> setpoints(wheel_count);
    if (protocol.ReadData(reinterpret_cast<uint8_t *>(setpoints.data()), sizeof(float) * wheel_count))
    {
        WheelSetpointCallback(setpoints);
        return true;
    }

    return false;
}

void ControlInterface::SendOdoData(const std::pair<timestamp_t, std::vector<odometry_t>> &odo_data)
{
    const bool speedBroadcast = odo_broadcast_flag.speed;
    const bool angleBroadcast = odo_broadcast_flag.angle;
    const bool pwmBroadcast = odo_broadcast_flag.pwm_value;

    if (!speedBroadcast && !angleBroadcast && !pwmBroadcast)
        return;

    constexpr uint8_t flag_index = 4;
    constexpr uint8_t header_and_command[] = {0xaa, 0xaa, 0xaa,
                                              static_cast<uint8_t>(Command::SEND_ODOMETRY)};
    constexpr size_t baseHeaderSize = sizeof(header_and_command) + 1;
    constexpr size_t timestampSize = sizeof(timestamp_t);
    const size_t angleBlockSize = angleBroadcast ? (sizeof(angle_t) * wheel_count) : 0;
    const size_t speedBlockSize = speedBroadcast ? (sizeof(angularvelocity_t) * wheel_count) : 0;
    const size_t pwmBlockSize = pwmBroadcast ? (sizeof(pwmvalue_t) * wheel_count) : 0;

    const size_t totalSize = angleBlockSize + speedBlockSize +
                             pwmBlockSize + timestampSize + baseHeaderSize;
    cacheVctr.resize(totalSize);

    uint8_t *ptr = cacheVctr.data();

    memcpy(ptr, header_and_command, sizeof(header_and_command));
    ptr[flag_index] = (angleBroadcast << 0) | (speedBroadcast << 1) | (pwmBroadcast << 2);

    ptr += baseHeaderSize;
    memcpy(ptr, &odo_data.first, timestampSize);
    ptr += timestampSize;

    for (const auto &data : odo_data.second)
    {
        if (angleBroadcast)
        {
            memcpy(ptr, &data.angle, sizeof(float));
            ptr += sizeof(float);
        }
        if (speedBroadcast)
        {
            memcpy(ptr, &data.angular_velocity, sizeof(float));
            ptr += sizeof(float);
        }
        if (pwmBroadcast)
        {
            memcpy(ptr, &data.pwm_value, sizeof(float));
            ptr += sizeof(float);
        }
    }

    protocol.SendData(cacheVctr);
}

/* bool ControlInterface::SendControllerProperties()
{
    protocol.SendCommand(Command::GET_CONTROLLER_PROPERTIES);
    protocol.SendData(controllerData.controllerProperties.to_bytes());
    return true;
}

bool ControlInterface::SendWheelData()
{
    uint8_t motorID = 0;
    protocol.ReadData((uint8_t *)&motorID, sizeof(motorID));

    if (motorID >= controllerData.controllerProperties.numMotors)
    {
        protocol.SendCommand(Command::READ_FAILURE);
        return false;
    }

    protocol.SendCommand(Command::GET_MOTOR_DATA);
    protocol.SendData((uint8_t *)&motorID, sizeof(motorID));
    protocol.SendData(controllerData.wheelData[motorID].to_bytes());
    return true;
} */

void ControlInterface::CallFunction(Command commandType)
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Calling function for command type: %d", static_cast<uint8_t>(commandType));
    switch (commandType)
    {
    case Command::SYNC_TIME:
        handleTimeSyncRequest(&protocol);
        break;
    case Command::SET_MOTOR_CONTROL_MODES:
        GetWheelControlMode();
        break;
    case Command::SET_WHEEL_SETPOINT:
        GetMotorSetpoints();
        break;
    case Command::PING:
        Ping();
        break;
    case Command::START:
        start();
        break;
    case Command::STOP:
        stop();
        break;
    case Command::SET_CONTROLLER_PROPERTIES:
        if (GetControllerProperties())
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Controller properties set successfully");
        else
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Failed to set controller properties");
        break;

    case Command::SET_MOTOR_DATA:
        GetWheelData();
        break;

    case Command::STOP_ALL_BROADCAST:
        stopAllBroadcast();
        break;

    case Command::RESTORE_ALL_BROADCAST:
        restoreAllBroadcast();
        break;

    case Command::SET_PID_CONSTANTS:
        GetPIDConstants();
        break;

    case Command::SET_ODO_BROADCAST_STATUS:
        GetOdoBroadcastStatus();
        break;

    default:
        break;
    }
}
