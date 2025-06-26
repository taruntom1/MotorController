#pragma once

#include "control/WheelManager.h"
#include "communication/CommunicationInterface.h"
#include "utils/MyStructs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class ControllerManager
{
public:
    // Singleton pattern
    static ControllerManager& getInstance();
    
    // Delete copy constructor and assignment operator to prevent copies
    ControllerManager(const ControllerManager&) = delete;
    ControllerManager& operator=(const ControllerManager&) = delete;

    void setControllerProperties(const controller_properties_t &controller_properties);

private:
    ControllerManager();

    WheelManager wheel_manager_;
    ControlInterface control_interface_;

    void connectCallbacks();
};