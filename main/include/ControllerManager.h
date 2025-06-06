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
    static void taskEntry(void *pvParameters);

private:
    WheelManager wheel_manager_;
    ControlInterface control_interface_;

    void Task();
};