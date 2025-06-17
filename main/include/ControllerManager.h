#pragma once

#include "WheelManager.h"
#include "ControlInterface.h"
#include "MyStructs.h"
#include "MPU6050Reader.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class ControllerManager
{
public:
    ControllerManager();
    ~ControllerManager();

    void setControllerProperties(const controller_properties_t &controller_properties);
    void createIMU(const imu_config_t &config);
    void deleteIMU();

private:
    WheelManager wheel_manager_;
    ControlInterface control_interface_;
    MPU6050Reader *mpu6050_reader_ = nullptr;

    void connectCallbacks();
};