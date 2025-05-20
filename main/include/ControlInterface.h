#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "UART_Protocol.h"
#include "MyStructs.h"
#include "Commands.h"
#include "TimeSyncServer.h"
#include "WheelManager.h"

enum class TaskAction;

class ControlInterface
{
public:
    ControlInterface(protocol_config config);

    void SendOdoData(const std::pair<timestamp_t, std::vector<odometry_t>> &odo_data);

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
    UARTProtocol protocol;

    uint8_t wheel_count = 0;
    std::vector<odo_broadcast_flags_t> odo_broadcast_flags;
    odo_broadcast_flags_t odo_broadcast_flag;

    std::vector<uint8_t> cacheVctr; // to prevent allocation for each call

    void Ping();
    void start();
    void stop();
    bool GetControllerProperties();
    // bool SendControllerProperties();
    bool GetWheelData();
    // bool SendWheelData();
    bool GetPIDConstants();
    bool GetWheelControlMode();
    bool GetMotorSetpoints();
    void stopAllBroadcast();
    void restoreAllBroadcast();
    void refreshBroadcastStatus();
    bool GetOdoBroadcastStatus();
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
