#include "Wheel.h"

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG // Set local log level for this file

// Task stack sizes and priorities for Wheel tasks
constexpr uint32_t WHEEL_RUN_TASK_STACK_SIZE = 4000;
constexpr UBaseType_t WHEEL_RUN_TASK_PRIORITY = 7;
constexpr uint32_t CONTROL_TASK_STACK_SIZE = 4096;
constexpr UBaseType_t CONTROL_TASK_PRIORITY = 5;
constexpr uint32_t PWM_DIRECT_CONTROL_TASK_STACK_SIZE = 2500;
constexpr UBaseType_t PWM_DIRECT_CONTROL_TASK_PRIORITY = 5;
constexpr uint32_t ODO_BROADCAST_TASK_STACK_SIZE = 2500;
constexpr UBaseType_t ODO_BROADCAST_TASK_PRIORITY = 5;
// BITMASKS for notifications
#define CONTROL_TASK_DELETED (1 << 5)

Wheel::Wheel(controller_properties_t *controller_properties, wheel_data_t *wheel_data, WheelTaskHandles *task_handles)
    : wheel_id(wheel_data->motor_id), wheel_data(wheel_data), task_handles(task_handles)
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Initializing Wheel instance %d", wheel_id);

    this->loop_delays.anglePID = 1000 / this->wheel_data->updateFrequenciesWheel.angle_pid;
    this->loop_delays.speedPID = 1000 / this->wheel_data->updateFrequenciesWheel.speed_pid;
    this->loop_delays.PWM = 1000 / this->wheel_data->updateFrequenciesWheel.pwm;
    this->loop_delays.encoder = 1000 / controller_properties->odoBroadcastFrequency;

    // Initialize the motor driver
    this->motor_config = {
        .directionPin = (gpio_num_t)wheel_data->motorConnections.dir,
        .pwmPin = (gpio_num_t)wheel_data->motorConnections.pwm,
        .clockFrequencyHz = 1000000,
        .pwmResolution = 1000};

    motorDriver = std::make_unique<MotorDriver>(this->motor_config);
    motorDriver->init();
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "MotorDriver initialized for Wheel %d", wheel_id);

    // Initialize the encoder
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Initializing encoder configuration for Wheel %d", wheel_id);
    this->encoder_config = {
        .channel_config = {
            .edge_gpio_num = (gpio_num_t)wheel_data->motorConnections.enc_a,
            .level_gpio_num = (gpio_num_t)wheel_data->motorConnections.enc_b}};
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Creating EncoderPulseReader object for Wheel %d", wheel_id);
    encoder = std::make_unique<EncoderPulseReader>(&this->encoder_config);

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Encoder initialized for Wheel %d", wheel_id);

    this->task_handles->wheel_run_task_handle = nullptr;
    this->task_handles->odo_broadcast = nullptr;
    this->task_handles->control_task_handle = nullptr;

    // Start the wheel run task
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Starting Wheel run task for instance %d", wheel_id);
    createTask([](void *param)
               { static_cast<Wheel *>(param)->Run(); },
               "Wheel run task", &this->task_handles->wheel_run_task_handle, WHEEL_RUN_TASK_STACK_SIZE, WHEEL_RUN_TASK_PRIORITY);
}

Wheel::Wheel(Wheel &&other) noexcept
    : TAG(other.TAG),
      wheel_id(other.wheel_id),
      wheel_data(other.wheel_data),
      task_handles(other.task_handles),
      loop_delays(other.loop_delays),
      motor_config(other.motor_config),
      motorDriver(std::move(other.motorDriver)),
      encoder_config(other.encoder_config),
      encoder(std::move(other.encoder)),
      control_task_run(other.control_task_run)
{
    // Optional: reset other's raw pointers or flags if necessary
    other.wheel_data = nullptr;
    other.task_handles = nullptr;
    other.control_task_run = false;
}

Wheel &Wheel::operator=(Wheel &&other) noexcept
{
    if (this != &other)
    {
        TAG = other.TAG;
        wheel_id = other.wheel_id;
        wheel_data = other.wheel_data;
        task_handles = other.task_handles;
        loop_delays = other.loop_delays;
        motor_config = other.motor_config;
        motorDriver = std::move(other.motorDriver);
        encoder_config = other.encoder_config;
        encoder = std::move(other.encoder);
        control_task_run = other.control_task_run;

        // Optional: reset other's state
        other.wheel_data = nullptr;
        other.task_handles = nullptr;
        other.control_task_run = false;
    }
    return *this;
}

Wheel::~Wheel()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Destroying Wheel instance %d", wheel_id);

    if (task_handles->control_task_handle != nullptr)
    {
        control_task_run = false;
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Stopping Control task for instance %d", wheel_id);
    }

    if (task_handles->wheel_run_task_handle != nullptr)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Stopping Wheel run task for instance %d", wheel_id);
        vTaskDelete(task_handles->wheel_run_task_handle);
        task_handles->wheel_run_task_handle = nullptr;
    }
    if (task_handles->odo_broadcast != nullptr)
    {
        vTaskDelete(task_handles->odo_broadcast);
        task_handles->odo_broadcast = nullptr;
    }
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Wheel instance %d destroyed", wheel_id);
}

uint8_t Wheel::GetWheelID() const
{
    return wheel_id;
}

void Wheel::Run()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Wheel instance %d Run task started", wheel_id);
    uint32_t receivedFlags;

    while (true)
    {
        xTaskNotifyWait(0, ULONG_MAX, &receivedFlags, portMAX_DELAY);
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Received Notification");

        if (receivedFlags & CONTROL_MODE_UPDATE)
        {
            updateControlMode();
        }

        if (receivedFlags & ODO_BROADCAST_STATUS_UPDATE)
        {
            updateOdoBroadcastStatus();
        }
    }
}

void Wheel::PWMDirectControl()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Wheel %d PWM Direct Control task started", wheel_id);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (control_task_run)
    {
        float speed = wheel_data->pwmValue;
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Wheel %d speed set: %.2f", wheel_id, speed);
        motorDriver->setSpeed(speed);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(loop_delays.PWM));
    }
    motorDriver->setSpeed(0);
    xTaskNotify(task_handles->wheel_run_task_handle, CONTROL_TASK_DELETED, eSetBits);
    vTaskDelete(NULL);
}

void Wheel::anglePIDControl()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Wheel %d angle PID Control task started", wheel_id);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    encoder->start_pulse_counter();
    auto pid = std::make_unique<PID<angle_t>>(&wheel_data->odometryData.angle, &wheel_data->pwmValue,
                                              &wheel_data->setpoint.angle,
                                              wheel_data->anglePIDConstants.p,
                                              wheel_data->anglePIDConstants.i,
                                              wheel_data->anglePIDConstants.d,
                                              PID<angle_t>::DIRECT,
                                              1000 / wheel_data->updateFrequenciesWheel.angle_pid);
    pid->SetMode(pid->AUTOMATIC);

    while (control_task_run)
    {
        wheel_data->odometryData.angle = encoder->get_raw_angle();
        pid->Compute();
        motorDriver->setSpeed(wheel_data->pwmValue);
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, " %d pwm: %.2f RPM SP: %.2f INP %.2f", wheel_id, wheel_data->pwmValue, wheel_data->setpoint.angle, wheel_data->odometryData.angle);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(loop_delays.anglePID));
    }

    pid.reset();
    encoder->stop_pulse_counter();
    motorDriver->setSpeed(0);
    xTaskNotify(task_handles->wheel_run_task_handle, CONTROL_TASK_DELETED, eSetBits);
    vTaskDelete(NULL);
}

void Wheel::speedPIDControl()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Wheel %d speed PID Control task started", wheel_id);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    encoder->start_pulse_counter();
    auto pid = std::make_unique<PID<angularvelocity_t>>(&wheel_data->odometryData.rpm, &wheel_data->pwmValue,
                                                        &wheel_data->setpoint.rpm,
                                                        wheel_data->speedPIDConstants.p,
                                                        wheel_data->speedPIDConstants.i,
                                                        wheel_data->speedPIDConstants.d,
                                                        PID<angularvelocity_t>::DIRECT,
                                                        1000 / wheel_data->updateFrequenciesWheel.speed_pid);

    pid->SetMode(PID<angularvelocity_t>::AUTOMATIC);

    while (control_task_run)
    {
        wheel_data->odometryData.rpm = encoder->get_raw_velocity();
        pid->Compute();
        motorDriver->setSpeed(wheel_data->pwmValue);
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "%d pwm: %.2f RPM SP: %.2f odo: %.2f", wheel_id, wheel_data->pwmValue, wheel_data->setpoint.rpm, wheel_data->odometryData.rpm);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(loop_delays.speedPID));
    }

    pid.reset();
    encoder->stop_pulse_counter();
    motorDriver->setSpeed(0);
    xTaskNotify(task_handles->wheel_run_task_handle, CONTROL_TASK_DELETED, eSetBits);
    vTaskDelete(NULL);
}

void Wheel::odoBroadcast()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Wheel %d encoder update task started", wheel_id);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true)
    {
        if (wheel_data->odoBroadcastStatus.angle)
        {
            wheel_data->odometryData.angle = encoder->get_raw_angle();
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Wheel %d angle: %.3f", wheel_id, wheel_data->odometryData.angle);
        }
        if (wheel_data->odoBroadcastStatus.speed)
        {
            wheel_data->odometryData.rpm = encoder->get_raw_velocity();
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Wheel %d rpm: %.3f", wheel_id, wheel_data->odometryData.rpm);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(loop_delays.encoder));
    }
}

bool Wheel::updateControlMode()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Wheel %d updating control mode", wheel_id);

    if (!deleteControlTask())
        return false;

    switch (wheel_data->control_mode)
    {
    case ControlMode::PWM_DIRECT_CONTROL:
        control_task_run = true;
        createTask([](void *param)
                   { static_cast<Wheel *>(param)->PWMDirectControl(); },
                   "PWMDirectControl", &task_handles->control_task_handle, PWM_DIRECT_CONTROL_TASK_STACK_SIZE, PWM_DIRECT_CONTROL_TASK_PRIORITY);
        break;
    case ControlMode::POSITION_CONTROL:
        control_task_run = true;
        createTask([](void *param)
                   { static_cast<Wheel *>(param)->anglePIDControl(); },
                   "anglePIDControl", &task_handles->control_task_handle, CONTROL_TASK_STACK_SIZE, CONTROL_TASK_PRIORITY);
        break;
    case ControlMode::SPEED_CONTROL:
        control_task_run = true;
        createTask([](void *param)
                   { static_cast<Wheel *>(param)->speedPIDControl(); },
                   "speedPIDControl", &task_handles->control_task_handle, CONTROL_TASK_STACK_SIZE, CONTROL_TASK_PRIORITY);
        break;
    case ControlMode::OFF:
        task_handles->control_task_handle = nullptr;
        break;

    default:
        break;
    }
    return true;
}

void Wheel::updateOdoBroadcastStatus()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Wheel %d changing ODO Broadcast Status", wheel_id);
    if ((wheel_data->odoBroadcastStatus.angle ||
         wheel_data->odoBroadcastStatus.speed) &&
        wheel_data->control_mode == ControlMode::PWM_DIRECT_CONTROL)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Wheel %d starting ODO Broadcast", wheel_id);
        encoder->start_pulse_counter();
        createTask([](void *param)
                   { static_cast<Wheel *>(param)->odoBroadcast(); },
                   "OdoBroadcastTask", &task_handles->odo_broadcast, ODO_BROADCAST_TASK_STACK_SIZE, ODO_BROADCAST_TASK_PRIORITY);
    }
    else if (task_handles->odo_broadcast != nullptr)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Wheel %d stopping ODO Broadcast", wheel_id);
        encoder->stop_pulse_counter();
        vTaskDelete(task_handles->odo_broadcast);
        task_handles->odo_broadcast = nullptr;
    }
}

void Wheel::createTask(TaskFunction_t task, const char *name, TaskHandle_t *handle, uint32_t stack_size, UBaseType_t priority)
{
    if (*handle != nullptr)
    {
        vTaskDelete(*handle);
        *handle = nullptr;
    }
    xTaskCreate(task, name, stack_size, this, priority, handle);
}

bool Wheel::deleteControlTask()
{
    if (task_handles->control_task_handle == nullptr)
        return true;

    control_task_run = false;
    uint32_t flags;
    auto status = xTaskNotifyWait(0, CONTROL_TASK_DELETED, &flags, pdMS_TO_TICKS(1000));
    if (status != pdPASS || !(flags & CONTROL_TASK_DELETED))
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Failed to delete control task");
        return false;
    }
    else
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Control task deleted successfully");
        return true;
    }
}
