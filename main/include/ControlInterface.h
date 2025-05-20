/**
 * @file ControlInterface.h
 * @brief Header file for the ControlInterface class, responsible for managing motor control and communication.
 */

#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H

#include "UART_Protocol.h"
#include "MyStructs.h"
#include "Commands.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


/**
 * @defgroup Class
 * @brief Main class for the communication interface
 * @{
 */

/**
 * @class ControlInterface
 * @brief Manages communication and control for motor operations using UART protocol.
 */
class ControlInterface
{
private:
    UARTProtocol protocol; ///< UART protocol instance for communication.

    controller_data_t &controllerData; ///< Reference to controller data structure.

    TaskHandles &taskHandles; ///< Reference to task handles for managing tasks.

    uint16_t runLoopDelay; ///< Delay between loop iterations for run method.

public:
    /**
     * @brief Constructor for ControlInterface.
     * @param config Configuration for the UART protocol.
     * @param controllerData Reference to the controller data structure.
     * @param taskHandles Reference to task handles for managing tasks.
     */
    ControlInterface(protocol_config &config, controller_data_t &controllerData, TaskHandles &taskHandles);

    /**
     * @brief Sends a ping command to verify communication.
     */
    void Ping();

    /**
     * @brief Initializes motor data.
     * @return True if successful, false otherwise.
     */
    bool initMotorData();

    /**
     * @brief Starts motor control operations.
     */
    void start();

    /**
     * @brief Stops motor control operations.
     */
    void stop();

    /**
     * @brief Recieves controller properties from the main device.
     * @return True if successful, false otherwise.
     */
    bool GetControllerProperties();

    /**
     * @brief Sends controller properties to the main device.
     * @return True if successful, false otherwise.
     */
    bool SendControllerProperties();

    /**
     * @brief Retrieves motor data from the device.
     * @return True if successful, false otherwise.
     */
    bool GetMotorData();

    /**
     * @brief Sends motor data to the device.
     * @return True if successful, false otherwise.
     */
    bool SendMotorData();

    /**
     * @brief Retrieves PID constants from the device.
     * @return True if successful, false otherwise.
     */
    bool GetPIDConstants();

    /**
     * @brief Retrieves PWM limits from the device.
     * @return True if successful, false otherwise.
     */
    bool GetPWMLimits();

    /**
     * @brief Retrieves motor control status from the device.
     * @return True if successful, false otherwise.
     *
     */
    bool GetMotorControlMode();

    /**
     * @brief Retrieves motor PWM values from the device.
     * @return True if successful, false otherwise.
     */
    bool GetMotorPWMs();

    /**
     * @brief Retrieves motor speed setpoints from the device.
     * @return True if successful, false otherwise.
     */
    bool GetMotorSpeedSetpoints();

    /**
     * @brief Retrieves motor angle setpoints from the device.
     * @return True if successful, false otherwise.
     */
    bool GetMotorAngleSetpoints();

    void stopAllBroadcast();
    void restoreAllBroadcast();

    /**
     * @brief Retrieves odometry broadcast status from the device.
     * @return True if successful, false otherwise.
     */
    bool GetOdoBroadcastStatus();

    /**
     * @brief Sends odometry speeds to the device.
     */
    void SendOdoSpeeds();

    /**
     * @brief Sends odometry angles to the device.
     */
    void SendOdoAngles();

    /**
     * @brief Sends PWM values to the device.
     */
    void SendOdoPWMs();

    /**
     * @brief Runs the main control loop.
     */
    void Run();

    /**
     * @brief Calls a specific function based on the command type.
     * @param commandType Command identifier to execute.
     */
    void CallFunction(uint8_t commandType);
};
/** @} */

#endif // CONTROL_INTERFACE_H
