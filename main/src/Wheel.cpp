#include "Wheel.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO // Set local log level for this file

Wheel::Wheel(wheel_data_t *wheel_data)
    : wheel_id(wheel_data->motor_id),
      control_mode(wheel_data->control_mode),
      anglePIDConstants(wheel_data->anglePIDConstants),
      speedPIDConstants(wheel_data->speedPIDConstants),
      motorConnections(wheel_data->motorConnections),
      updateFrequenciesWheel(wheel_data->updateFrequenciesWheel),
      odoBroadcastStatus(wheel_data->odoBroadcastStatus),
      radians_per_tick(wheel_data->radians_per_tick)
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Initializing Wheel instance %d", wheel_id);

    InitMotorDriver();
    InitEncoder();
}

Wheel::~Wheel()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Destroying Wheel instance %d", wheel_id);
    updateControlMode(ControlMode::OFF);
}

Wheel::Wheel(Wheel &&other) noexcept
{
    TAG = other.TAG;
    wheel_id = other.wheel_id;
    motor_id = other.motor_id;
    control_mode = other.control_mode;
    anglePIDConstants = other.anglePIDConstants;
    speedPIDConstants = other.speedPIDConstants;
    motorConnections = other.motorConnections;
    updateFrequenciesWheel = other.updateFrequenciesWheel;
    odoBroadcastStatus = other.odoBroadcastStatus;
    angle_odom.store(other.angle_odom.load());
    angular_velocity_odom.store(other.angular_velocity_odom.load());
    setpoint_atomic.store(other.setpoint_atomic.load());
    radians_per_tick = other.radians_per_tick;
    pwm_value_atomic.store(other.pwm_value_atomic.load());

    motor_config = other.motor_config;
    motorDriver = std::move(other.motorDriver);

    encoder_config = other.encoder_config;
    encoder = std::move(other.encoder);

    pid = std::move(other.pid);

    pid_const_update.store(other.pid_const_update.load());

    setpoint = other.setpoint;
    angle = other.angle;
    angular_velocity = other.angular_velocity;
    pwm = other.pwm;
}

Wheel &Wheel::operator=(Wheel &&other) noexcept
{
    if (this != &other)
    {
        TAG = other.TAG;
        wheel_id = other.wheel_id;
        motor_id = other.motor_id;
        control_mode = other.control_mode;
        anglePIDConstants = other.anglePIDConstants;
        speedPIDConstants = other.speedPIDConstants;
        motorConnections = other.motorConnections;
        updateFrequenciesWheel = other.updateFrequenciesWheel;
        odoBroadcastStatus = other.odoBroadcastStatus;
        angle_odom.store(other.angle_odom.load());
        angular_velocity_odom.store(other.angular_velocity_odom.load());
        setpoint_atomic.store(other.setpoint_atomic.load());
        radians_per_tick = other.radians_per_tick;
        pwm_value_atomic.store(other.pwm_value_atomic.load());

        motor_config = other.motor_config;
        motorDriver = std::move(other.motorDriver);

        encoder_config = other.encoder_config;
        encoder = std::move(other.encoder);

        pid = std::move(other.pid);

        pid_const_update.store(other.pid_const_update.load());

        setpoint = other.setpoint;
        angle = other.angle;
        angular_velocity = other.angular_velocity;
        pwm = other.pwm;
    }
    return *this;
}

odometry_t Wheel::getOdometry()
{
    if (control_mode == ControlMode::OFF || control_mode == ControlMode::PWM_DIRECT_CONTROL)
    {
        refreshOdometry();
    }

    return odometry_t(angle_odom.load(std::memory_order_release),
                      angular_velocity_odom.load(std::memory_order_release));
}

void Wheel::updateControlMode(ControlMode mode)
{
    if (control_mode == mode)
        return;

    switch (control_mode)
    {
    case ControlMode::OFF:
        break;
    case ControlMode::PWM_DIRECT_CONTROL:
        deInitPWMDirect();
        break;
    case ControlMode::POSITION_CONTROL:
        deInitAnglePID();
        break;
    case ControlMode::SPEED_CONTROL:
        deInitSpeedPID();
        break;
    default:
        break;
    }

    control_mode = mode;

    switch (control_mode)
    {
    case ControlMode::PWM_DIRECT_CONTROL:
        initPWMDirect();
        break;
    case ControlMode::POSITION_CONTROL:
        initAnglePID();
        break;
    case ControlMode::SPEED_CONTROL:
        initSpeedPID();
        break;
    default:
        break;
    }
}

void Wheel::updateControlLoop()
{
    switch (control_mode)
    {
    case ControlMode::PWM_DIRECT_CONTROL:
        updatePWMDirectControl;
        break;
    case ControlMode::POSITION_CONTROL:
        updateAnglePIDControl();
        break;
    case ControlMode::SPEED_CONTROL:
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
    angular_velocity = static_cast<angularvelocity_t>(tickrate) * radians_per_tick;

    angle_odom.store(angle);
    angular_velocity_odom.store(angular_velocity);
}

void Wheel::updateSetpoint(float setpoint)
{
    setpoint_atomic.store(setpoint, std::memory_order_relaxed);
}

void Wheel::updatePIDConstants(PIDType type, pid_constants_t &pid_constants)
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
    pid_const_update.store(true, std::memory_order_acquire);
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
}

void Wheel::updatePWMDirectControl()
{
    pwmvalue_t pwm_value = setpoint_atomic.load();
    pwm_value_atomic.store(pwm_value);
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Wheel %d speed set: %.2f", wheel_id, pwm_value);
    motorDriver->setSpeed(pwm_value);
}

void Wheel::deInitPWMDirect()
{
    motorDriver->setSpeed(0);
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Wheel %d PWM Direct Control task stoped", wheel_id);
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
                                       1000 / updateFrequenciesWheel.angle_pid);
    pid->SetMode(PID<float>::AUTOMATIC);
}

void Wheel::updateAnglePIDControl()
{
    setpoint = setpoint_atomic.load(std::memory_order_release);

    refreshOdometry();
    pid->Compute();

    pwm_value_atomic.store(pwm);
    motorDriver->setSpeed(pwm);

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, " %d pwm: %.2f RPM SP: %.2f INP %.2f",
                        wheel_id, pwm, setpoint,
                        angle);

    if (unlikely(pid_const_update.load(std::memory_order_acquire)))
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Wheel %d updating PID constants", wheel_id);
        pid->SetTunings(anglePIDConstants.p,
                        anglePIDConstants.i,
                        anglePIDConstants.d);
        pid_const_update.store(false, std::memory_order_release);
    }
}

void Wheel::deInitAnglePID()
{
    motorDriver->setSpeed(0);
    pid.reset();
    encoder->stop_pulse_counter();
}

void Wheel::initSpeedPID()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Wheel %d speed PID Control task started", wheel_id);
    encoder->start_pulse_counter();
    auto pid = std::make_unique<PID<angularvelocity_t>>(&angular_velocity, &pwm,
                                                        &setpoint,
                                                        speedPIDConstants.p,
                                                        speedPIDConstants.i,
                                                        speedPIDConstants.d,
                                                        PID<angularvelocity_t>::DIRECT,
                                                        1000 / updateFrequenciesWheel.speed_pid);

    pid->SetMode(PID<angularvelocity_t>::AUTOMATIC);
}

void Wheel::updateSpeedPIDControl()
{
    setpoint = setpoint_atomic.load(std::memory_order_release);

    refreshOdometry();
    pid->Compute();

    pwm_value_atomic.store(pwm);
    motorDriver->setSpeed(pwm);

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "%d pwm: %.2f RPM SP: %.2f odo: %.2f",
                        wheel_id, pwm, setpoint,
                        angular_velocity);

    if (unlikely(pid_const_update.load(std::memory_order_acquire)))
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Wheel %d updating PID constants", wheel_id);
        pid->SetTunings(speedPIDConstants.p,
                        speedPIDConstants.i,
                        speedPIDConstants.d);
        pid_const_update.store(false, std::memory_order_release);
    }
}

void Wheel::deInitSpeedPID()
{
    motorDriver->setSpeed(0);
    pid.reset();
    encoder->stop_pulse_counter();
}
