#pragma once

#include "WheelManager.h"
#include "ControlInterface.h"
#include "MyStructs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class ControllerManager
{
public:
    ControllerManager();

    void setControllerProperties(const controller_properties_t &controller_properties);

private:
    WheelManager wheel_manager_;
    ControlInterface control_interface_;

    void connectCallbacks();
};