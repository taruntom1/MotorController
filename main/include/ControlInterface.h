#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "UART_Protocol.h"
#include "MyStructs.h"
#include "Commands.h"
#include "TimeSyncServer.h"

class ControlInterface
{
public:
    ControlInterface(protocol_config &config);

    void SendOdoData();

private:
    UARTProtocol protocol;
    uint16_t runLoopDelay;

    void Ping();
    void start();
    void stop();
    bool GetControllerProperties();
    bool SendControllerProperties();
    bool GetMotorData();
    bool SendMotorData();
    bool GetPIDConstants();
    bool GetPWMLimits();
    bool GetMotorControlMode();
    bool GetMotorPWMs();
    bool GetMotorSpeedSetpoints();
    bool GetMotorAngleSetpoints();
    void stopAllBroadcast();
    void restoreAllBroadcast();
    bool GetOdoBroadcastStatus();
    void CallFunction(Command commandType);

    // Callback functions for different commands
    std::

};

#endif // CONTROL_INTERFACE_H
