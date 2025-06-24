/// @file MotorDriver.h
/// @brief Header file for the MotorDriver class.

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <driver/mcpwm_prelude.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <algorithm>

/// @struct MotorDriverConfig
/// @brief Configuration structure for MotorDriver.
struct MotorDriverConfig
{
    gpio_num_t directionPin; ///< GPIO pin for motor direction control.
    gpio_num_t pwmPin; ///< GPIO pin for PWM output.
    uint32_t clockFrequencyHz; ///< Clock frequency in Hz.
    uint32_t pwmResolution; ///< PWM resolution.
};

/// @class MotorDriver
/// @brief Class to control a motor using MCPWM on an ESP32.
class MotorDriver
{
private:
    mcpwm_timer_handle_t timer = nullptr; ///< Handle for MCPWM timer.
    mcpwm_oper_handle_t operator_ = nullptr; ///< Handle for MCPWM operator.
    mcpwm_cmpr_handle_t comparator = nullptr; ///< Handle for MCPWM comparator.
    mcpwm_gen_handle_t generator = nullptr; ///< Handle for MCPWM generator.

    MotorDriverConfig config; ///< Motor driver configuration (stored by value).

    const char *TAG = "MotorDriver"; ///< Tag for logging purposes.

public:
    /// @brief Constructor for MotorDriver.
    /// @param config Pointer to the motor driver configuration.
    explicit MotorDriver(const MotorDriverConfig &config);
    
    /// @brief Destructor for MotorDriver.
    ~MotorDriver();

    /// @brief Initializes the motor driver.
    /// @return True if initialization was successful, false otherwise.
    bool init();

    /// @brief Sets the motor speed.
    /// @param speed Motor speed in the range -1.0 to 1.0.
    void setSpeed(float speed);
};

#endif // MOTOR_DRIVER_H
