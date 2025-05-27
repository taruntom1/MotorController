// In MyStructs.h
/**
 * @file MyStructs.h
 * @brief Header file for the structs used in the communication interface
 *  class, responsible for managing motor control and communication.
 */
#ifndef MYSTRUCTS_H
#define MYSTRUCTS_H

#include <stdint.h>
#include <stdfloat>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @defgroup Structs
 * @brief Structs for storing different properties and setpoints of the controllers
 * @{
 */

using angle_t = float;             // Angle in degrees
using angularvelocity_t = float; // Angular velocity in degrees per second
using pwmvalue_t = float;

/**
 * @struct odo_broadcast_flags_t
 * @brief Represents the status of odometry data broadcasting.
 */
struct odo_broadcast_flags_t
{
    bool angle = false;     ///< Indicates if angle data is being broadcast.
    bool speed = false;     ///< Indicates if speed data is being broadcast.
    bool pwm_value = false; ///< Indicates if PWM data is being broadcast.
};

/**
 * @struct pid_constants_t
 * @brief Contains PID constants for motor control.
 */
struct pid_constants_t
{
    float p = 0; ///< Proportional constant.
    float i = 0; ///< Integral constant.
    float d = 0; ///< Derivative constant.
};

/**
 * @struct limits_pwm_t
 * @brief Defines the PWM limits for motor control.
 */
struct limits_pwm_t
{
    int8_t min = 0; ///< Minimum PWM value.
    int8_t max = 0; ///< Maximum PWM value.
};

/**
 * @struct connections_wheel_t
 * @brief Represents the hardware connections for a motor.
 */
struct connections_wheel_t
{
    uint8_t dir;   ///< Direction pin.
    uint8_t pwm;   ///< PWM pin.
    uint8_t enc_a; ///< Encoder pin A.
    uint8_t enc_b; ///< Encoder pin B.
};

/**
 * @struct odometry_t
 * @brief Stores odometry information for a motor.
 */
struct odometry_t
{
    angle_t angle = 0;         ///< Current angle in degrees.
    angularvelocity_t rpm = 0; ///< Current speed in RPM.
};

/**
 * @struct setpoint_t
 * @brief Represents desired values for motor control.
 */
struct setpoint_t
{
    angle_t angle = 0;         ///< Desired angle in degrees.
    angularvelocity_t rpm = 0; ///< Desired speed in RPM.
};

/**
 * @struct UpdateFrequencyWheel
 * @brief different loop update frequency for each wheel
 */
struct wheel_update_frequencies_t
{
    uint16_t pwm = 50;       ///< PWM update frequency in Hz when on Direct PWM control.
    uint16_t angle_pid = 50; ///< Angle PID update frequency in Hz when on Position control mode.
    uint16_t speed_pid = 50; ///< Speed PID update frequency in Hz when on speed control mode.
};

/**
 * @brief ControlMode enum
 * @brief Enumerates different control modes for motor operation.
 */
enum class ControlMode : uint8_t
{
    OFF,                ///< Off
    PWM_DIRECT_CONTROL, ///< Direct PWM control mode.
    POSITION_CONTROL,   ///< Position control mode.
    SPEED_CONTROL       ///< Speed control mode.

};

/**
 * @struct wheel_data_t
 * @brief Encapsulates all data related to a motor.
 */
struct wheel_data_t
{
    uint8_t motor_id = 0;                                       ///< Unique identifier for the motor.
    ControlMode control_mode = ControlMode::PWM_DIRECT_CONTROL; ///< Control mode for the motor.
    pid_constants_t anglePIDConstants;                          ///< PID constants for angle control.
    pid_constants_t speedPIDConstants;                          ///< PID constants for speed control.
    limits_pwm_t pwmLimits;                                     ///< PWM limits.
    connections_wheel_t motorConnections;                       ///< Hardware connections.
    odometry_t odometryData;                                    ///< Odometry data.
    setpoint_t setpoint;                                        ///< Desired setpoints.
    odo_broadcast_flags_t odoBroadcastStatus;                   ///< Status of odometry broadcasting.
    wheel_update_frequencies_t updateFrequenciesWheel;          ///< Different update frequencies for wheel control.
    pwmvalue_t pwmValue = 0;                                    ///< Current PWM value.
};

/**
 * @struct LoopingFrequencys
 * @brief Defines different update frequencies for various components.
 */

struct update_frequencies_t
{
    uint16_t interfaceRun = 50;
};

/**
 * @struct controller_properties_t
 * @brief Defines properties of the motor controller.
 */
struct controller_properties_t
{
    bool run = false;                         ///< Indicates if the controller is active.
    uint8_t numMotors = 0;                    ///< Number of motors controlled.
    odo_broadcast_flags_t odoBroadcastStatus; ///< Status of odometry broadcasting.
    uint16_t odoBroadcastFrequency = 30;      ///< Frequency of odometry broadcasting.
    update_frequencies_t updateFrequencies;   ///< Different update frequencies for various components.
};

/**
 * @struct controller_data_t
 * @brief Contains all data for the motor controller.
 */
struct controller_data_t
{
    std::vector<wheel_data_t> wheelData;          ///< Pointer to an array of wheel_data_t structures.
    controller_properties_t controllerProperties; ///< Properties of the controller.
};

struct WheelTaskHandles
{
    TaskHandle_t wheel_run_task_handle;
    TaskHandle_t control_task_handle;
    TaskHandle_t odo_broadcast;
};

struct InterfaceTaskHandles
{
    TaskHandle_t Run;
};

struct TaskHandles
{
    TaskHandle_t main_task_handle;
    TaskHandle_t wheel_manager;
    InterfaceTaskHandles interface_task_handles;
    std::vector<WheelTaskHandles> wheel_task_handles;
};

/** @defgroup notifications Notifications
 *  @brief Defines notification messages for the motor controller.
 *  @{
 */

/** Wheel run task notifications */
#define CONTROL_MODE_UPDATE (1 << 0)
#define ODO_BROADCAST_STATUS_UPDATE (1 << 1)
#define PID_CONSTANTS_UPDATE (1 << 2)

/** @} */ // end of notifications

#endif // MYSTRUCTS_H
