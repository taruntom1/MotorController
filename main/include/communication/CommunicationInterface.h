#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "communication/UART_Protocol.h"
#include "utils/MyStructs.h"
#include "communication/Commands.h"
#include "communication/TimeSyncServer.h"
#include "core/TaskManager.h"

#include <stdexcept>
#include <string>
#include <format>

enum class TaskAction;

// Custom exception classes for ControlInterface
class ControlInterfaceException : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

class DataReadException : public ControlInterfaceException
{
public:
    explicit DataReadException(const std::string &message) : ControlInterfaceException("Data read failed: " + message) {}
};

class InvalidMotorIdException : public ControlInterfaceException
{
public:
    explicit InvalidMotorIdException(uint8_t motorId, uint8_t maxId)
        : ControlInterfaceException(std::format("Invalid motor ID {}, max allowed: {}", motorId, maxId - 1)) {}
};

class InvalidPIDTypeException : public ControlInterfaceException
{
public:
    explicit InvalidPIDTypeException(uint8_t pidType)
        : ControlInterfaceException(std::format("Invalid PID type: {}", pidType)) {}
};

class ControlInterface
{
public:
    explicit ControlInterface(const protocol_config &config);

    void SendOdoData(const std::pair<timestamp_t, std::vector<odometry_t>> &odo_data);
    void SendIMUData(const std::pair<timestamp_t, imu_data_t> &imu_data);

    // Callback connectors
    void setControllerRunCallback(std::function<void(bool)> cb)
    {
        ControllerRunCallback = std::move(cb);
    }
    void setControllerPropertiesCallback(std::function<void(controller_properties_t &)> cb)
    {
        ControllerPropertiesCallback = std::move(cb);
    }
    void setWheelDataCallback(std::function<void(const wheel_data_t &)> cb)
    {
        WheelDataCallback = std::move(cb);
    }
    void setPIDConstantsCallback(std::function<void(uint8_t, PIDType, pid_constants_t)> cb)
    {
        PIDConstantsCallback = std::move(cb);
    }
    void setWheelControlModeCallback(std::function<void(uint8_t, ControlMode)> cb)
    {
        WheelControlModeCallback = std::move(cb);
    }
    void setWheelSetpointCallback(std::function<void(const std::vector<float> &)> cb)
    {
        WheelSetpointCallback = std::move(cb);
    }
    void setOdoBroadcastCallbackNonBlocking(std::function<void(TaskAction)> cb)
    {
        OdoBroadcastCallbackNonBlocking = std::move(cb);
    }
    void setOdoBroadcastCallbackBlocking(std::function<void(TaskAction)> cb)
    {
        OdoBroadcastCallbackBlocking = std::move(cb);
    }

private:
    static constexpr const char *TAG = "ControlInterface";
    UARTProtocol protocol;

    uint8_t wheel_count = 0;
    std::vector<odo_broadcast_flags_t> odo_broadcast_flags;
    odo_broadcast_flags_t odo_broadcast_flag;

    std::vector<uint8_t> cacheVctr;     // to prevent allocation for each call
    std::vector<float> motor_setpoints; // cache for motor setpoints
    std::vector<uint8_t> imuCacheVctr; // separate cache for IMU data

    // Helper methods for exception handling
    template<typename Func>
    std::vector<uint8_t> safeReadData(size_t size, uint32_t timeout, const std::string& operation, Func&& converter);

    template<typename Func>
    void safeExecuteWithFailureResponse(const std::string& operation, Func&& func);

    void sendProtocolCommand(Command cmd, const std::string& operation);
    void sendProtocolData(const std::vector<uint8_t>& data, const std::string& operation);

    void Ping();
    void start();
    void stop();
    void GetControllerProperties();
    // bool SendControllerProperties();
    void GetWheelData();
    // bool SendWheelData();
    void GetPIDConstants();
    void GetWheelControlMode();
    void GetMotorSetpoints();
    void stopAllBroadcast();
    void restoreAllBroadcast();
    void refreshBroadcastStatus();
    void GetOdoBroadcastStatus();
    void CallFunction(Command commandType);

    // Callback functions for different
    std::function<void(bool)> ControllerRunCallback;
    std::function<void(controller_properties_t &)> ControllerPropertiesCallback;
    std::function<void(const wheel_data_t &)> WheelDataCallback;
    std::function<void(uint8_t, PIDType, pid_constants_t)> PIDConstantsCallback;
    std::function<void(uint8_t, ControlMode)> WheelControlModeCallback;
    std::function<void(const std::vector<float> &)> WheelSetpointCallback;
    std::function<void(TaskAction)> OdoBroadcastCallbackNonBlocking;
    std::function<void(TaskAction)> OdoBroadcastCallbackBlocking;
};

#endif // CONTROL_INTERFACE_H
