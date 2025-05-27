#include "ControlInterface.h"
#include "esp_log.h"
#include "print.h"
#include <string.h>

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG // Set local log level for this file

static const char *TAG = "ControlInterface";

ControlInterface::ControlInterface(protocol_config &config, controller_data_t &controllerData, TaskHandles &taskHandles)
    : protocol(config), controllerData(controllerData), taskHandles(taskHandles)
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Initializing control interface");
    protocol.begin();
    initMotorData();

    // Initialize the run loop delay
    runLoopDelay = 1000 / controllerData.controllerProperties.updateFrequencies.interfaceRun;

    protocol.onCommandReceived = [this](uint8_t command)
    {
        this->CallFunction(command);
    };

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Starting Control interface run task");
    xTaskCreate([](void *param)
                { static_cast<ControlInterface *>(param)->Run(); },
                "Communication Interface", 4096, this, 5, &taskHandles.interface_task_handles.Run);
}

void ControlInterface::Ping()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Ping sent");
    protocol.SendCommand(static_cast<uint8_t>(Command::PING));
}

void ControlInterface::start()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Starting motor controller");
    controllerData.controllerProperties.run = true;
    protocol.SendCommand(static_cast<uint8_t>(Command::READ_SUCCESS));
}
void ControlInterface::stop()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Stopping motor controller");
    controllerData.controllerProperties.run = false;
    protocol.SendCommand(static_cast<uint8_t>(Command::READ_SUCCESS));
}

bool ControlInterface::GetControllerProperties()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Reading controller properties");
    controller_properties_t controllerPropertiesTemp;
    if (protocol.ReadData((uint8_t *)&controllerPropertiesTemp, sizeof(controller_properties_t), 1000))
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Controller properties read successfully");

        controllerData.controllerProperties = controllerPropertiesTemp;
        if (initMotorData())
        {
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Motor data initialized successfully");
            protocol.SendCommand(static_cast<uint8_t>(Command::READ_SUCCESS));
            return true;
        }
        else
        {
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Motor data initialization failed");
            protocol.SendCommand(static_cast<uint8_t>(Command::READ_FAILURE));
            return false;
        }
    }
    else
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Controller properties read failed");
        protocol.SendCommand(static_cast<uint8_t>(Command::READ_FAILURE));
        return false;
    }
}

bool ControlInterface::initMotorData()
{
    controllerData.wheelData.clear();

    if (controllerData.controllerProperties.numMotors > 0 && controllerData.controllerProperties.numMotors <= 10)
    {
        controllerData.wheelData.resize(controllerData.controllerProperties.numMotors);
    }
    else
    {
        return false;
    }
    xTaskNotify(taskHandles.wheel_manager, (1 << 10), eSetBits);
    return true;
}

bool ControlInterface::GetMotorData()
{
    wheel_data_t temp;
    uint8_t motorID = 0;
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Getting motor data...");
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Getting motor ID...");

    if (protocol.ReadData(&motorID, sizeof(motorID)))
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Motor ID read successful");
        if (motorID >= controllerData.controllerProperties.numMotors)
        {
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Motor ID out of range");
            protocol.SendCommand(static_cast<uint8_t>(Command::READ_FAILURE));
            return false;
        }
    }
    else
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Motor ID read failed");
        protocol.SendCommand(static_cast<uint8_t>(Command::READ_FAILURE));
        return false;
    }

    if (protocol.ReadData((uint8_t *)&temp, sizeof(wheel_data_t), 1000))
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Motor data read successful for motor ID %d", motorID);
        controllerData.wheelData[motorID] = temp;
        protocol.SendCommand(static_cast<uint8_t>(Command::READ_SUCCESS));

        // Sending notification to motor manager task to construct Wheels
        xTaskNotify(taskHandles.wheel_manager, (1 << motorID), eSetBits);

        return true;
    }
    else
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Motor data read failed for motor ID %d", motorID);
        protocol.SendCommand(static_cast<uint8_t>(Command::READ_FAILURE));
        return false;
    }
}

void ControlInterface::stopAllBroadcast()
{
    controllerData.controllerProperties.odoBroadcastStatus = odo_broadcast_flags_t{0, 0, 0};
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "All broadcasts stopped");
}

void ControlInterface::restoreAllBroadcast()
{
    for (auto wheel_data : controllerData.wheelData)
    {
        controllerData.controllerProperties.odoBroadcastStatus.angle |= wheel_data.odoBroadcastStatus.angle;
        controllerData.controllerProperties.odoBroadcastStatus.speed |= wheel_data.odoBroadcastStatus.speed;
        controllerData.controllerProperties.odoBroadcastStatus.pwm_value |= wheel_data.odoBroadcastStatus.pwm_value;
    }
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "All broadcasts restored");
}

bool ControlInterface::GetOdoBroadcastStatus()
{
    // Buffer size: motorID + odo_broadcast_flags_t
    constexpr size_t buffer_size = sizeof(uint8_t) + sizeof(odo_broadcast_flags_t);
    uint8_t buffer[buffer_size];

    // Read motorID + broadcast flags in one read
    if (!protocol.ReadData(buffer, buffer_size, 1000))
    {
        protocol.SendCommand(static_cast<uint8_t>(Command::READ_FAILURE));
        return false;
    }

    uint8_t motorID = buffer[0];
    if (motorID >= controllerData.wheelData.size())
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN, TAG, "Motor ID out of range while reading broadcast status");
        protocol.SendCommand(static_cast<uint8_t>(Command::READ_FAILURE));
        return false;
    }

    // Copy broadcast flags data from buffer into temp struct using std::copy
    odo_broadcast_flags_t temp;
    std::copy(buffer + 1, buffer + 1 + sizeof(odo_broadcast_flags_t), reinterpret_cast<uint8_t *>(&temp));

    controllerData.wheelData[motorID].odoBroadcastStatus = temp;
    protocol.SendCommand(static_cast<uint8_t>(Command::READ_SUCCESS));
    xTaskNotify(taskHandles.wheel_task_handles[motorID].wheel_run_task_handle, ODO_BROADCAST_STATUS_UPDATE, eSetBits);

    // Update aggregated broadcast status
    odo_broadcast_flags_t &aggregatedStatus = controllerData.controllerProperties.odoBroadcastStatus;
    memset(&aggregatedStatus, 0, sizeof(odo_broadcast_flags_t));

    for (int i = 0; i < controllerData.controllerProperties.numMotors; i++)
    {
        aggregatedStatus.angle |= controllerData.wheelData[i].odoBroadcastStatus.angle;
        aggregatedStatus.speed |= controllerData.wheelData[i].odoBroadcastStatus.speed;
        aggregatedStatus.pwm_value |= controllerData.wheelData[i].odoBroadcastStatus.pwm_value;
    }

    return true;
}

bool ControlInterface::GetPIDConstants()
{
    uint8_t buffer[sizeof(uint8_t) * 2 + sizeof(pid_constants_t)];

    // Read all required data in a single call
    if (!protocol.ReadData(buffer, sizeof(buffer), 1000))
    {
        protocol.SendCommand(static_cast<uint8_t>(Command::READ_FAILURE));
        return false;
    }

    uint8_t motorID = buffer[0];
    uint8_t pidType = buffer[1];
    pid_constants_t temp;
    std::copy(buffer + 2, buffer + 2 + sizeof(pid_constants_t), reinterpret_cast<uint8_t *>(&temp));

    // Validate motor ID
    if (motorID >= controllerData.wheelData.size())
    {
        protocol.SendCommand(static_cast<uint8_t>(Command::READ_FAILURE));
        return false;
    }

    // Store PID constants
    switch (pidType)
    {
    case 0:
        controllerData.wheelData[motorID].anglePIDConstants = temp;
        break;
    case 1:
        controllerData.wheelData[motorID].speedPIDConstants = temp;
        break;
    default:
        protocol.SendCommand(static_cast<uint8_t>(Command::READ_FAILURE));
        return false;
    }

    xTaskNotify(taskHandles.wheel_task_handles[motorID].wheel_run_task_handle, PID_CONSTANTS_UPDATE, eSetBits);
    protocol.SendCommand(static_cast<uint8_t>(Command::READ_SUCCESS));
    return true;
}

bool ControlInterface::GetMotorControlMode()
{
    uint8_t buffer[1 + sizeof(ControlMode)] = {0};
    if (!protocol.ReadData(buffer, sizeof(buffer)))
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Failed to read motor ID and control mode");
        return false;
    }

    uint8_t motorID = buffer[0];
    ControlMode temp = *reinterpret_cast<ControlMode *>(&buffer[1]);

    if (motorID >= controllerData.wheelData.size())
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Invalid motor ID %d", motorID);
        return false;
    }

    controllerData.wheelData[motorID].control_mode = temp;
    xTaskNotify(taskHandles.wheel_task_handles[motorID].wheel_run_task_handle, CONTROL_MODE_UPDATE, eSetBits);

    protocol.SendCommand(static_cast<uint8_t>(Command::READ_SUCCESS));
    return true;
}

bool ControlInterface::GetMotorSpeedSetpoints()
{
    const uint8_t motorCount = controllerData.controllerProperties.numMotors;
    angularvelocity_t motorRPMs[motorCount];

    if (!protocol.ReadData(reinterpret_cast<uint8_t *>(motorRPMs), sizeof(motorRPMs)))
    {
        return false;
    }

    for (uint8_t i = 0; i < motorCount; i++)
    {
        controllerData.wheelData[i].setpoint.rpm = motorRPMs[i];
    }

    return true;
}

bool ControlInterface::GetMotorAngleSetpoints()
{
    const uint8_t motorCount = controllerData.controllerProperties.numMotors;
    angle_t motorAngles[motorCount];

    if (!protocol.ReadData(reinterpret_cast<uint8_t *>(motorAngles), sizeof(motorAngles)))
    {
        return false;
    }

    for (uint8_t i = 0; i < motorCount; i++)
    {
        controllerData.wheelData[i].setpoint.angle = motorAngles[i];
    }
    return true;
}

bool ControlInterface::GetMotorPWMs()
{
    const uint8_t motorCount = controllerData.controllerProperties.numMotors;
    pwmvalue_t pwmValues[motorCount];

    if (!protocol.ReadData(reinterpret_cast<uint8_t *>(pwmValues), sizeof(pwmValues)))
    {
        return false;
    }

    for (uint8_t i = 0; i < motorCount; i++)
    {
        controllerData.wheelData[i].pwmValue = pwmValues[i];
    }

    return true;
}


void ControlInterface::SendOdoData()
{
    const bool speedBroadcast = controllerData.controllerProperties.odoBroadcastStatus.speed;
    const bool angleBroadcast = controllerData.controllerProperties.odoBroadcastStatus.angle;
    const bool pwmBroadcast = controllerData.controllerProperties.odoBroadcastStatus.pwm_value;

    if (!speedBroadcast && !angleBroadcast && !pwmBroadcast)
        return;

    const uint8_t header[] = {0xaa, 0xaa, 0xaa};
    const size_t numMotors = controllerData.controllerProperties.numMotors;

    const size_t angleBlockSize = angleBroadcast ? (4 + sizeof(angle_t) * numMotors) : 0;
    const size_t speedBlockSize = speedBroadcast ? (4 + sizeof(angularvelocity_t) * numMotors) : 0;
    const size_t pwmBlockSize = pwmBroadcast ? (4 + sizeof(pwmvalue_t) * numMotors) : 0;

    const size_t totalSize = angleBlockSize + speedBlockSize + pwmBlockSize;
    uint8_t data[totalSize];

    uint8_t *ptr = data;

    auto write_block = [&](bool enabled, Command cmd, auto get_value, size_t typeSize)
    {
        if (!enabled)
            return;

        memcpy(ptr, header, sizeof(header));
        ptr[3] = static_cast<uint8_t>(cmd);
        ptr += 4;

        for (const auto &wheel : controllerData.wheelData)
        {
            auto value = get_value(wheel);
            memcpy(ptr, &value, typeSize);
            ptr += typeSize;
        }
    };

    write_block(angleBroadcast, Command::SEND_ODO_ANGLES, [](const auto &w)
                { return w.odometryData.angle; }, sizeof(angle_t));

    write_block(speedBroadcast, Command::SEND_ODO_SPEEDS, [](const auto &w)
                { return w.odometryData.rpm; }, sizeof(angularvelocity_t));

    write_block(pwmBroadcast, Command::SEND_ODO_PWMS, [](const auto &w)
                { return w.pwmValue; }, sizeof(pwmvalue_t));

    protocol.SendData(data, totalSize);
}

bool ControlInterface::SendControllerProperties()
{
    protocol.SendCommand(static_cast<uint8_t>(Command::GET_CONTROLLER_PROPERTIES));
    protocol.SendData((uint8_t *)&controllerData.controllerProperties, sizeof(controller_properties_t));
    return true;
}

bool ControlInterface::SendMotorData()
{
    uint8_t motorID = 0;
    protocol.ReadData((uint8_t *)&motorID, sizeof(motorID));

    if (motorID >= controllerData.controllerProperties.numMotors)
    {
        protocol.SendCommand(static_cast<uint8_t>(Command::READ_FAILURE));
        return false;
    }

    protocol.SendCommand(static_cast<uint8_t>(Command::GET_MOTOR_DATA));
    protocol.SendData((uint8_t *)&motorID, sizeof(motorID));
    protocol.SendData((uint8_t *)&controllerData.wheelData[motorID], sizeof(wheel_data_t));
    return true;
}

void ControlInterface::Run()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Running control interface");
    for (;;)
    {
        SendOdoData();
        vTaskDelay(pdMS_TO_TICKS(runLoopDelay));
    }
}

void ControlInterface::CallFunction(uint8_t commandType)
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Calling function for command type: %d", commandType);
    switch (static_cast<Command>(commandType))
    {
    case Command::SET_MOTOR_CONTROL_MODES:
        GetMotorControlMode();
        break;
    case Command::SET_MOTOR_SPEED_SETPOINTS:
        GetMotorSpeedSetpoints();
        break;
    case Command::SET_MOTOR_ANGLE_SETPOINTS:
        GetMotorAngleSetpoints();
        break;
    case Command::SET_MOTOR_PWMS:
        GetMotorPWMs();
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
        GetMotorData();
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

    case Command::GET_MOTOR_DATA:
        if (SendMotorData())
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Motor data send successfully");
        else
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Failed to sent motor data");

        break;
    case Command::GET_CONTROLLER_PROPERTIES:
        SendControllerProperties();
        break;

    default:
        break;
    }
}
