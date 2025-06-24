#include "Wheel.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO // Set local log level for this file

Wheel::Wheel(const wheel_data_t *wheel_data, TickType_t control_loop_delay_ticks)
    : wheel_id(wheel_data->motor_id),
      control_mode(ControlMode::OFF),
      anglePIDConstants(wheel_data->anglePIDConstants),
      speedPIDConstants(wheel_data->speedPIDConstants),
      motorConnections(wheel_data->motorConnections),
      odoBroadcastStatus(wheel_data->odoBroadcastStatus),
      radians_per_tick(wheel_data->radians_per_tick),
      control_loop_delay_ms(pdTICKS_TO_MS(control_loop_delay_ticks)),
      rolling_mean(10)
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Initializing Wheel instance %d", wheel_id);

    InitMotorDriver();
    InitEncoder();
    updateControlMode(control_mode);
}

Wheel::~Wheel()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Destroying Wheel instance %d", wheel_id);
    updateControlMode(ControlMode::OFF);
}

Wheel::Wheel(Wheel &&other) noexcept
    : TAG(other.TAG),
      wheel_id(other.wheel_id),
      control_mode(other.control_mode),
      anglePIDConstants(other.anglePIDConstants),
      speedPIDConstants(other.speedPIDConstants),
      motorConnections(other.motorConnections),
      odoBroadcastStatus(other.odoBroadcastStatus),
      angle_odom(other.angle_odom.load()),
      angular_velocity_odom(other.angular_velocity_odom.load()),
      setpoint_atomic(other.setpoint_atomic.load()),
      radians_per_tick(other.radians_per_tick),
      control_loop_delay_ms(other.control_loop_delay_ms),
      pwm_value_atomic(other.pwm_value_atomic.load()),
      motor_config(other.motor_config),
      motorDriver(std::move(other.motorDriver)),
      rolling_mean(10),
      encoder_config(other.encoder_config),
      encoder(std::move(other.encoder)),
      pid(std::move(other.pid)),
      pid_property_update(other.pid_property_update.load()),
      setpoint(other.setpoint),
      angle(other.angle),
      angular_velocity(other.angular_velocity),
      pwm(other.pwm)
{
}

Wheel &Wheel::operator=(Wheel &&other) noexcept
{
    if (this != &other)
    {
        TAG = other.TAG;
        wheel_id = other.wheel_id;
        control_mode = other.control_mode;
        anglePIDConstants = other.anglePIDConstants;
        speedPIDConstants = other.speedPIDConstants;
        motorConnections = other.motorConnections;
        odoBroadcastStatus = other.odoBroadcastStatus;
        angle_odom.store(other.angle_odom.load());
        angular_velocity_odom.store(other.angular_velocity_odom.load());
        setpoint_atomic.store(other.setpoint_atomic.load());
        radians_per_tick = other.radians_per_tick;
        control_loop_delay_ms = other.control_loop_delay_ms;
        pwm_value_atomic.store(other.pwm_value_atomic.load());
        motor_config = other.motor_config;
        motorDriver = std::move(other.motorDriver);
        rolling_mean = std::move(other.rolling_mean);
        encoder_config = other.encoder_config;
        encoder = std::move(other.encoder);
        pid = std::move(other.pid);
        pid_property_update.store(other.pid_property_update.load());
        setpoint = other.setpoint;
        angle = other.angle;
        angular_velocity = other.angular_velocity;
        pwm = other.pwm;
    }
    return *this;
}

void Wheel::updateWheelData(const wheel_data_t *wheel_data, TickType_t control_loop_delay_ticks)
{
    wheel_id = wheel_data->motor_id;
    control_mode = wheel_data->control_mode;
    anglePIDConstants = wheel_data->anglePIDConstants;
    speedPIDConstants = wheel_data->speedPIDConstants;
    motorConnections = wheel_data->motorConnections;
    odoBroadcastStatus = wheel_data->odoBroadcastStatus;
    radians_per_tick = wheel_data->radians_per_tick;
    control_loop_delay_ms = pdTICKS_TO_MS(control_loop_delay_ticks);

    pid_property_update.store(true);
}

odometry_t Wheel::getOdometry()
{
    if (control_mode == ControlMode::OFF || control_mode == ControlMode::PWM_DIRECT_CONTROL)
    {
        refreshOdometry();
    }

    return {angle_odom.load(std::memory_order_acquire),
            angular_velocity_odom.load(std::memory_order_acquire),
            pwm_value_atomic.load(std::memory_order_acquire)};
}

void Wheel::updateControlMode(ControlMode mode)
{
    using enum ControlMode;
    if (control_mode == mode)
        return;

    switch (control_mode)
    {
    case OFF:
        break;
    case PWM_DIRECT_CONTROL:
        deInitPWMDirect();
        break;
    case POSITION_CONTROL:
        deInitAnglePID();
        break;
    case SPEED_CONTROL:
        deInitSpeedPID();
        break;
    default:
        break;
    }

    control_mode = mode;

    switch (control_mode)
    {
    case PWM_DIRECT_CONTROL:
        initPWMDirect();
        break;
    case POSITION_CONTROL:
        initAnglePID();
        break;
    case SPEED_CONTROL:
        initSpeedPID();
        break;
    default:
        break;
    }
}

void Wheel::updateLoopDelay(TickType_t delay_ticks)
{
    control_loop_delay_ms = pdTICKS_TO_MS(delay_ticks);
    pid_property_update.store(true, std::memory_order_release);
}

void Wheel::updateControlLoop()
{
    switch (control_mode)
    {
        using enum ControlMode;
    case PWM_DIRECT_CONTROL:
        updatePWMDirectControl();
        break;
    case POSITION_CONTROL:
        updateAnglePIDControl();
        break;
    case SPEED_CONTROL:
        updateSpeedPIDControl();
        break;
    default:
        break;
    }
}

void Wheel::refreshOdometry()
{
    encoder_ticks_t ticks;
    encoder_tickrate_t tickrate;
    encoder->get_tick_tickrate(ticks, tickrate);

    angle = static_cast<angle_t>(ticks) * radians_per_tick;
    rolling_mean.accumulate(static_cast<angularvelocity_t>(tickrate) * radians_per_tick);
    angular_velocity = rolling_mean.getRollingMean();

    angle_odom.store(angle, std::memory_order_release);
    angular_velocity_odom.store(angular_velocity, std::memory_order_release);
}

void Wheel::updateSetpoint(float new_setpoint)
{
    setpoint_atomic.store(new_setpoint, std::memory_order_release);
}

void Wheel::updatePIDConstants(PIDType type, const pid_constants_t &pid_constants)
{
    switch (type)
    {
    case PIDType::POSITION:
        anglePIDConstants = pid_constants;
        break;
    case PIDType::VELOCITY:
        speedPIDConstants = pid_constants;
        break;
    default:
        break;
    }
    pid_property_update.store(true, std::memory_order_release);
}

void Wheel::InitMotorDriver()
{
    motor_config = {
        .directionPin = (gpio_num_t)motorConnections.dir,
        .pwmPin = (gpio_num_t)motorConnections.pwm,
        .clockFrequencyHz = 1000000,
        .pwmResolution = 1000};

    motorDriver = std::make_unique<MotorDriver>(motor_config);
    motorDriver->init();

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "MotorDriver initialized for Wheel %d", wheel_id);
}

void Wheel::InitEncoder()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Initializing encoder configuration for Wheel %d", wheel_id);

    encoder_config = {
        .channel_config = {
            .edge_gpio_num = (gpio_num_t)motorConnections.enc_a,
            .level_gpio_num = (gpio_num_t)motorConnections.enc_b}};

    encoder = std::make_unique<EncoderPulseReader>(&encoder_config);

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Encoder initialized for Wheel %d", wheel_id);
}

void Wheel::initPWMDirect()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Wheel %d PWM Direct Control task started", wheel_id);
    encoder->start_pulse_counter();
    motorDriver->setSpeed(0);
    pwm = 0;
}

void Wheel::updatePWMDirectControl()
{
    pwmvalue_t pwm_value = setpoint_atomic.load(std::memory_order_acquire);
    pwm_value_atomic.store(pwm_value, std::memory_order_relaxed);
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Wheel %d speed set: %.2f", wheel_id, pwm_value);
    motorDriver->setSpeed(pwm_value);
}

void Wheel::deInitPWMDirect()
{
    motorDriver->setSpeed(0);
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Wheel %d PWM Direct Control task stoped", wheel_id);
    encoder->stop_pulse_counter();
    encoder->clear_pulse_count();
}

void Wheel::initAnglePID()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Wheel %d angle PID Control task started", wheel_id);

    encoder->start_pulse_counter();
    pid = std::make_unique<PID<float>>(&angle, &pwm,
                                       &setpoint,
                                       anglePIDConstants.p,
                                       anglePIDConstants.i,
                                       anglePIDConstants.d,
                                       PID<angle_t>::DIRECT,
                                       control_loop_delay_ms);
    motorDriver->setSpeed(0);
    pwm = 0;
    pid->SetMode(PID<float>::AUTOMATIC);
}

void Wheel::updateAnglePIDControl()
{
    setpoint = setpoint_atomic.load(std::memory_order_acquire);

    refreshOdometry();
    pid->Compute();

    pwm_value_atomic.store(pwm, std::memory_order_relaxed);
    motorDriver->setSpeed(pwm);

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, " %d pwm: %.2f RPM SP: %.2f INP %.2f",
                        wheel_id, pwm, setpoint,
                        angle);

    if (pid_property_update.load(std::memory_order_acquire)) [[unlikely]]
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Wheel %d updating PID constants", wheel_id);
        pid->SetTunings(anglePIDConstants.p,
                        anglePIDConstants.i,
                        anglePIDConstants.d);
        pid->SetSampleTime(control_loop_delay_ms);
        pid_property_update.store(false, std::memory_order_release);
    }
}

void Wheel::deInitAnglePID()
{
    motorDriver->setSpeed(0);
    pid.reset();
    encoder->stop_pulse_counter();
    encoder->clear_pulse_count();
}

void Wheel::initSpeedPID()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Wheel %d speed PID Control task started", wheel_id);
    encoder->start_pulse_counter();
    pid = std::make_unique<PID<float>>(&angular_velocity, &pwm,
                                       &setpoint,
                                       speedPIDConstants.p,
                                       speedPIDConstants.i,
                                       speedPIDConstants.d,
                                       PID<angularvelocity_t>::DIRECT,
                                       control_loop_delay_ms);

    motorDriver->setSpeed(0);
    pwm = 0;
    pid->SetMode(PID<float>::AUTOMATIC);
}

void Wheel::updateSpeedPIDControl()
{
    setpoint = setpoint_atomic.load(std::memory_order_acquire);

    refreshOdometry();
    pid->Compute();

    pwm_value_atomic.store(pwm, std::memory_order_relaxed);
    motorDriver->setSpeed(pwm);

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "%d pwm: %.2f RPM SP: %.2f odo: %.2f",
                        wheel_id, pwm, setpoint,
                        angular_velocity);

    if (pid_property_update.load(std::memory_order_acquire)) [[unlikely]]
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Wheel %d updating PID constants", wheel_id);
        pid->SetTunings(speedPIDConstants.p,
                        speedPIDConstants.i,
                        speedPIDConstants.d);
        pid->SetSampleTime(control_loop_delay_ms);
        pid_property_update.store(false, std::memory_order_release);
    }
}

void Wheel::deInitSpeedPID()
{
    motorDriver->setSpeed(0);
    pid.reset();
    encoder->stop_pulse_counter();
    encoder->clear_pulse_count();
}
