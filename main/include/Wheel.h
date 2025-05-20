#pragma once

//=== Standard Library ===//
#include <memory>
#include <atomic>

//=== ESP-IDF & FreeRTOS ===//
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

//=== Custom Includes ===//
#include "MyStructs.h"
#include "MotorDriver.h"
#include "EncoderPulseReader.h"
#include "PID.h"
#include "RollingMeanAccumulator.h"

//=== Structs ===//
struct LoopDelays
{
    uint32_t anglePID = 0;
    uint32_t speedPID = 0;
    uint32_t PWM = 0;
    uint32_t encoder = 0;
};

//=== Wheel Class ===//
class Wheel
{
public:
    // Constructor & Destructor
    Wheel(const wheel_data_t *wheel_data, TickType_t control_loop_delay_ticks);

    ~Wheel();

    // Move Semantics
    Wheel(Wheel &&other) noexcept;
    Wheel &operator=(Wheel &&other) noexcept;

    // Deleted Copy Semantics
    Wheel(const Wheel &) = delete;
    Wheel &operator=(const Wheel &) = delete;

    void updateWheelData(const wheel_data_t *wheel_data, TickType_t control_loop_delay_ticks);

    // Getters
    uint8_t GetWheelID() const { return wheel_id; }
    odometry_t getOdometry();

    void updateControlMode(ControlMode mode);
    void updateLoopDelay(TickType_t delay_ticks);
    void updatePIDConstants(PIDType type, pid_constants_t &pid_constants);
    void updateSetpoint(float setpoint);
    void updateControlLoop();

private:
    // Logger Tag
    const char *TAG = "Wheel";

    // Identification
    uint8_t wheel_id;

    // Wheel Data
    ControlMode control_mode = ControlMode::OFF;
    pid_constants_t anglePIDConstants;
    pid_constants_t speedPIDConstants;
    connections_wheel_t motorConnections;
    odo_broadcast_flags_t odoBroadcastStatus;
    std::atomic<angle_t> angle_odom{0};
    std::atomic<angularvelocity_t> angular_velocity_odom{0};
    std::atomic<float> setpoint_atomic{0};
    float radians_per_tick = 1.0f;
    uint16_t control_loop_delay_ms;
    std::atomic<pwmvalue_t> pwm_value_atomic{0};

    // Motor and Encoder
    MotorDriverConfig motor_config;
    std::unique_ptr<MotorDriver> motorDriver;
    RollingMeanAccumulator<float> rolling_mean;

    pcnt_config_t encoder_config;
    std::unique_ptr<EncoderPulseReader> encoder;

    // PID object
    std::unique_ptr<PID<float>> pid;

    // State flags
    std::atomic<bool> pid_property_update{false};

    // data
    float setpoint;
    angle_t angle = 0;
    angularvelocity_t angular_velocity = 0;
    pwmvalue_t pwm;

    // Initialization Methods

    void InitMotorDriver();
    void InitEncoder();

    // Private Methods
    void refreshOdometry();

    void initPWMDirect();
    void updatePWMDirectControl();
    void deInitPWMDirect();
    void initAnglePID();
    void updateAnglePIDControl();
    void deInitAnglePID();
    void initSpeedPID();
    void updateSpeedPIDControl();
    void deInitSpeedPID();
};
