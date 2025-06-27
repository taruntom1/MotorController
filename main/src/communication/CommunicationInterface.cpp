#include "communication/CommunicationInterface.h"
#include "esp_log.h"
#include "communication/TimeSyncServer.h"
#include <string.h>
#include <format>
#include <utility>

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG // Set local log level for this file

ControlInterface::ControlInterface(const protocol_config &config)
    : protocol(config)
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Initializing control interface");

    try
    {
        protocol.begin();
    }
    catch (const UARTException &e)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Failed to initialize UART protocol: %s", e.what());
        throw ControlInterfaceException("UART initialization failed: " + std::string(e.what()));
    }

    protocol.onCommandReceived = [this](Command command)
    {
        this->CallFunction(command);
    };

    cacheVctr.reserve(100);
}

// Helper method for safe data reading with exception conversion
template <typename Func>
std::vector<uint8_t> ControlInterface::safeReadData(size_t size, uint32_t timeout, const std::string &operation, Func &&converter)
{
    try
    {
        return protocol.ReadData(size, timeout);
    }
    catch (const UARTTimeoutException &e)
    {
        throw converter("Timeout " + operation + ": " + std::string(e.what()));
    }
    catch (const UARTException &e)
    {
        throw converter("UART error " + operation + ": " + std::string(e.what()));
    }
}

// Helper method for executing operations with automatic failure response
template <typename Func>
void ControlInterface::safeExecuteWithFailureResponse(const std::string &operation, Func &&func)
{
    try
    {
        func();
        sendProtocolCommand(Command::READ_SUCCESS, "send success confirmation");
    }
    catch (const ControlInterfaceException &e)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "%s failed: %s", operation.c_str(), e.what());
        try
        {
            protocol.SendCommand(Command::READ_FAILURE);
        }
        catch (const UARTException &uart_e)
        {
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Failed to send failure notification: %s", uart_e.what());
        }
        throw;
    }
}

// Helper method for sending protocol commands with exception handling
void ControlInterface::sendProtocolCommand(Command cmd, const std::string &operation)
{
    try
    {
        protocol.SendCommand(cmd);
    }
    catch (const UARTException &e)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN, TAG, "Failed to %s: %s", operation.c_str(), e.what());
        if (cmd != Command::READ_SUCCESS) [[unlikely]]
        {
            throw ControlInterfaceException("Failed to " + operation + ": " + std::string(e.what()));
        }
        // Don't throw for READ_SUCCESS failures as main operation succeeded
    }
}

// Helper method for sending protocol data with exception handling
void ControlInterface::sendProtocolData(const std::vector<uint8_t> &data, const std::string &operation)
{
    try
    {
        protocol.SendData(data);
    }
    catch (const UARTException &e)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Failed to %s: %s", operation.c_str(), e.what());
        throw ControlInterfaceException("Failed to " + operation + ": " + std::string(e.what()));
    }
}

void ControlInterface::Ping()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Ping sent");
    sendProtocolCommand(Command::PING, "send ping command");
}

void ControlInterface::start()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Starting motor controller");
    assert(ControllerRunCallback && "ControllerRunCallback must be set");
    ControllerRunCallback(true);
    sendProtocolCommand(Command::READ_SUCCESS, "send start confirmation");
}

void ControlInterface::stop()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Stopping motor controller");
    assert(ControllerRunCallback && "ControllerRunCallback must be set");
    ControllerRunCallback(false);
    sendProtocolCommand(Command::READ_SUCCESS, "send stop confirmation");
}

void ControlInterface::GetControllerProperties()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Reading controller properties");

    safeExecuteWithFailureResponse("Reading controller properties", [this]()
                                   {
        auto data = safeReadData(controller_properties_t::size, 1000, "reading controller properties", 
                                [](const std::string& msg) { return DataReadException(msg); });

        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Controller properties read successfully");
        size_t offset = 0;
        controller_properties_t properties;
        properties.from_bytes(data, offset);

        wheel_count = properties.numMotors;
        odo_broadcast_flags.resize(wheel_count);
        motor_setpoints.resize(wheel_count);

        assert(ControllerPropertiesCallback && "ControllerPropertiesCallback must be set");
        ControllerPropertiesCallback(properties); });
}

void ControlInterface::GetWheelData()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Getting motor data...");

    safeExecuteWithFailureResponse("Reading motor data", [this]()
                                   {
        constexpr size_t dataSize = sizeof(uint8_t) + wheel_data_t::size;
        auto buffer = safeReadData(dataSize, 1000, "reading motor data",
                                  [](const std::string& msg) { return DataReadException(msg); });

        ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Motor ID and Motor Data received");
        uint8_t wheel_id = buffer[0];
        if (wheel_id >= wheel_count) [[unlikely]]
        {
            throw InvalidMotorIdException(wheel_id, wheel_count);
        }

        size_t offset = 1;
        wheel_data_t wheelData;
        wheelData.from_bytes(buffer, offset);

        odo_broadcast_flags.at(wheel_id) = wheelData.odoBroadcastStatus;
        refreshBroadcastStatus();

        assert(wheelData.motor_id == wheel_id);
        assert(WheelDataCallback && "WheelDataCallback must be set");
        WheelDataCallback(wheelData);

        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Motor data read successful for motor ID %d", wheel_id); });
}

void ControlInterface::stopAllBroadcast()
{
    assert(OdoBroadcastCallbackBlocking && "OdoBroadcastCallbackBlocking must be set");
    OdoBroadcastCallbackBlocking(TaskAction::Suspend);
    sendProtocolCommand(Command::READ_SUCCESS, "send stop broadcast confirmation");
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "All broadcasts stopped");
}

void ControlInterface::restoreAllBroadcast()
{
    refreshBroadcastStatus();
    assert(OdoBroadcastCallbackBlocking && "OdoBroadcastCallbackBlocking must be set");
    OdoBroadcastCallbackBlocking(TaskAction::Resume);
    sendProtocolCommand(Command::READ_SUCCESS, "send restore broadcast confirmation");
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "All broadcasts restored");
}

void ControlInterface::refreshBroadcastStatus()
{
    // Zero initialize aggregate flag
    odo_broadcast_flag = {};

    for (const auto &flag : odo_broadcast_flags)
    {
        odo_broadcast_flag |= flag;
    }

    const TaskAction action = (odo_broadcast_flag.angle ||
                               odo_broadcast_flag.speed ||
                               odo_broadcast_flag.pwm_value)
                                  ? TaskAction::Start
                                  : TaskAction::Stop;

    assert(OdoBroadcastCallbackNonBlocking && "OdoBroadcastCallbackNonBlocking must be set");
    OdoBroadcastCallbackNonBlocking(action);
}

void ControlInterface::GetOdoBroadcastStatus()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Reading odo broadcast status");

    safeExecuteWithFailureResponse("Reading odo broadcast status", [this]()
                                   {
        constexpr size_t buffer_size = sizeof(uint8_t) + odo_broadcast_flags_t::size;
        auto buffer = safeReadData(buffer_size, 1000, "reading odo broadcast status",
                                  [](const std::string& msg) { return DataReadException(msg); });

        const uint8_t motorID = buffer[0];
        if (odo_broadcast_flags.size() <= motorID) [[unlikely]]
        {
            throw InvalidMotorIdException(motorID, wheel_count);
        }
        
        size_t offset = 1;
        odo_broadcast_flags[motorID].from_bytes(buffer, offset);
        refreshBroadcastStatus(); });
}

void ControlInterface::GetPIDConstants()
{
    safeExecuteWithFailureResponse("Reading PID constants", [this]()
                                   {
        constexpr size_t buffer_size = sizeof(uint8_t) * 2 + pid_constants_t::size;
        auto buffer = safeReadData(buffer_size, 1000, "reading PID constants",
                                  [](const std::string& msg) { return DataReadException(msg); });

        const uint8_t &motorID = buffer[0];
        const uint8_t &pidType = buffer[1];
        
        if (motorID >= wheel_count) [[unlikely]]
        {
            throw InvalidMotorIdException(motorID, wheel_count);
        }
        if (pidType >= 2) [[unlikely]]
        {
            throw InvalidPIDTypeException(pidType);
        }

        size_t offset = 2;
        pid_constants_t constants;
        constants.from_bytes(buffer, offset);

        assert(PIDConstantsCallback && "PIDConstantsCallback must be set");
        PIDConstantsCallback(motorID, static_cast<PIDType>(pidType), constants); });
}

void ControlInterface::GetWheelControlMode()
{
    safeExecuteWithFailureResponse("Reading wheel control mode", [this]()
                                   {
        constexpr size_t buffer_size = sizeof(uint8_t) + sizeof(ControlMode);
        auto buffer = safeReadData(buffer_size, 1000, "reading wheel control mode",
                                  [](const std::string& msg) { return DataReadException(msg); });

        const uint8_t &motorID = buffer[0];
        if (motorID >= wheel_count) [[unlikely]]
        {
            throw InvalidMotorIdException(motorID, wheel_count);
        }

        size_t offset = 1;
        ControlMode control_mode;
        from_bytes(control_mode, buffer, offset);
        
        assert(WheelControlModeCallback && "WheelControlModeCallback must be set");
        WheelControlModeCallback(motorID, control_mode); });
}

void ControlInterface::GetMotorSetpoints()
{
    try
    {
        bool success = false;
        try
        {
            success = protocol.ReadData(reinterpret_cast<uint8_t *>(motor_setpoints.data()), sizeof(float) * wheel_count);
        }
        catch (const UARTTimeoutException &e)
        {
            throw DataReadException("Timeout reading motor setpoints: " + std::string(e.what()));
        }
        catch (const UARTException &e)
        {
            throw DataReadException("UART error reading motor setpoints: " + std::string(e.what()));
        }

        if (!success) [[unlikely]]
        {
            throw DataReadException("Failed to read motor setpoints");
        }

        assert(WheelSetpointCallback && "WheelSetpointCallback must be set");
        WheelSetpointCallback(motor_setpoints);
    }
    catch (const ControlInterfaceException &e)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Motor setpoints read failed: %s", e.what());
        throw;
    }
}

void ControlInterface::SendOdoData(const std::pair<timestamp_t, std::vector<odometry_t>> &odo_data)
{
    const bool speedBroadcast = odo_broadcast_flag.speed;
    const bool angleBroadcast = odo_broadcast_flag.angle;
    const bool pwmBroadcast = odo_broadcast_flag.pwm_value;

    if (!speedBroadcast && !angleBroadcast && !pwmBroadcast) [[unlikely]]
        return;

    constexpr uint8_t flag_index = 4;
    constexpr uint8_t header_and_command[] = {0xaa, 0xaa, 0xaa,
                                              std::to_underlying(Command::SEND_ODOMETRY)};
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
    sendProtocolData(cacheVctr, "send odometry data");
}

void ControlInterface::SendIMUData(const std::pair<timestamp_t, imu_data_t> &imu_data)
{
    constexpr uint8_t flag_index = 4;
    constexpr uint8_t header_and_command[] = {0xaa, 0xaa, 0xaa,
                                              std::to_underlying(Command::SEND_IMU_DATA)};
    constexpr size_t baseHeaderSize = sizeof(header_and_command) + 1;
    constexpr size_t timestampSize = sizeof(timestamp_t);
    constexpr size_t imuBlockSize = imu_data_t::size;
    constexpr size_t totalSize = baseHeaderSize + timestampSize + imuBlockSize;
    imuCacheVctr.resize(totalSize);

    uint8_t *ptr = imuCacheVctr.data();
    memcpy(ptr, header_and_command, sizeof(header_and_command));
    ptr[flag_index] = 0; // No flags for IMU data, set to 0
    ptr += baseHeaderSize;
    memcpy(ptr, &imu_data.first, timestampSize);
    ptr += timestampSize;
    auto imu_bytes = imu_data.second.to_bytes();
    memcpy(ptr, imu_bytes.data(), imuBlockSize);
    sendProtocolData(imuCacheVctr, "send imu data");
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

    try
    {
        switch (commandType)
        {
            using enum Command;
        case SYNC_TIME:
            handleTimeSyncRequest(&protocol);
            break;
        case SET_MOTOR_CONTROL_MODES:
            GetWheelControlMode();
            break;
        case SET_WHEEL_SETPOINT:
            GetMotorSetpoints();
            break;
        case PING:
            Ping();
            break;
        case START:
            start();
            break;
        case STOP:
            stop();
            break;
        case SET_CONTROLLER_PROPERTIES:
            GetControllerProperties();
            break;
        case SET_MOTOR_DATA:
            GetWheelData();
            break;

        case STOP_ALL_BROADCAST:
            stopAllBroadcast();
            break;

        case RESTORE_ALL_BROADCAST:
            restoreAllBroadcast();
            break;

        case SET_PID_CONSTANTS:
            GetPIDConstants();
            break;

        case SET_ODO_BROADCAST_STATUS:
            GetOdoBroadcastStatus();
            break;

        default:
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN, TAG, "Unknown command received: %d", static_cast<int>(commandType));
            break;
        }
    }
    catch (const ControlInterfaceException &e)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Command execution failed for command %d: %s",
                            static_cast<int>(commandType), e.what());
        // Exception is already handled by individual methods, just log here
    }
}
