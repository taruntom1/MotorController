#include "Wheel.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO // Set local log level for this file

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
// run task
#define DESTROY_WHEEL_OBJECT (1 << 5)
#define CONTROL_TASK_DELETED (1 << 10)
// manage wheels task
#define WHEEL_DESTROYED (1 << 12)

Wheel::Wheel(controller_properties_t *controller_properties, wheel_data_t *wheel_data,
             WheelTaskHandles *task_handles, TaskHandle_t *manage_wheels_taskHandle)
    : wheel_id(wheel_data->motor_id),
      wheel_data(wheel_data),
      task_handles(task_handles),
      manage_wheels_taskHandle(manage_wheels_taskHandle)
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Initializing Wheel instance %d", wheel_id);

    InitLoopDelays(controller_properties);
    InitMotorDriver();
    InitEncoder();
    InitTaskHandles();
    StartWheelTask();
}

void Wheel::InitLoopDelays(controller_properties_t *controller_properties)
{
    loop_delays.anglePID = 1000 / wheel_data->updateFrequenciesWheel.angle_pid;
    loop_delays.speedPID = 1000 / wheel_data->updateFrequenciesWheel.speed_pid;
    loop_delays.PWM = 1000 / wheel_data->updateFrequenciesWheel.pwm;
    loop_delays.encoder = 1000 / controller_properties->odoBroadcastFrequency;
}

void Wheel::InitMotorDriver()
{
    motor_config = {
        .directionPin = (gpio_num_t)wheel_data->motorConnections.dir,
        .pwmPin = (gpio_num_t)wheel_data->motorConnections.pwm,
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
            .edge_gpio_num = (gpio_num_t)wheel_data->motorConnections.enc_a,
            .level_gpio_num = (gpio_num_t)wheel_data->motorConnections.enc_b}};

    encoder = std::make_unique<EncoderPulseReader>(&encoder_config);

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Encoder initialized for Wheel %d", wheel_id);
}

void Wheel::InitTaskHandles()
{
    task_handles->wheel_run_task_handle = nullptr;
    task_handles->odo_broadcast = nullptr;
    task_handles->control_task_handle = nullptr;
}

void Wheel::StartWheelTask()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Starting Wheel run task for instance %d", wheel_id);

    createTask([](void *param)
               { static_cast<Wheel *>(param)->Run(); },
               "Wheel run task",
               &task_handles->wheel_run_task_handle,
               WHEEL_RUN_TASK_STACK_SIZE,
               WHEEL_RUN_TASK_PRIORITY);
}

Wheel::Wheel(Wheel &&other) noexcept
    : TAG(other.TAG),
      wheel_id(other.wheel_id),
      wheel_data(other.wheel_data),
      task_handles(other.task_handles),
      manage_wheels_taskHandle(other.manage_wheels_taskHandle),
      loop_delays(other.loop_delays),
      motor_config(other.motor_config),
      motorDriver(std::move(other.motorDriver)),
      encoder_config(other.encoder_config),
      encoder(std::move(other.encoder))

{
    // Optional: reset other's raw pointers or flags if necessary
    control_task_run.store(false, std::memory_order_release);
    other.wheel_data = nullptr;
    other.task_handles = nullptr;
    other.control_task_run.store(false, std::memory_order_release);
}

Wheel &Wheel::operator=(Wheel &&other) noexcept
{
    if (this != &other)
    {
        TAG = other.TAG;
        wheel_id = other.wheel_id;
        wheel_data = other.wheel_data;
        task_handles = other.task_handles;
        manage_wheels_taskHandle = other.manage_wheels_taskHandle;
        loop_delays = other.loop_delays;
        motor_config = other.motor_config;
        motorDriver = std::move(other.motorDriver);
        encoder_config = other.encoder_config;
        encoder = std::move(other.encoder);
        control_task_run.store(false, std::memory_order_release);

        // Optional: reset other's state
        other.wheel_data = nullptr;
        other.task_handles = nullptr;
        other.control_task_run.store(false, std::memory_order_release);
    }
    return *this;
}

Wheel::~Wheel()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Destroying Wheel instance %d", wheel_id);
    xTaskNotify(task_handles->wheel_run_task_handle, DESTROY_WHEEL_OBJECT, eSetBits);

    // Wait for destruction to complete
    uint32_t receivedFlags;
    while (true)
    {
        xTaskNotifyWait(0, WHEEL_DESTROYED, &receivedFlags, portMAX_DELAY);
        if ((receivedFlags & WHEEL_DESTROYED) != 0)
            break;
    }
    if (task_handles->wheel_run_task_handle != nullptr)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Stopping Wheel run task for instance %d", wheel_id);
        vTaskDelete(task_handles->wheel_run_task_handle);
        task_handles->wheel_run_task_handle = nullptr;
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
        if (receivedFlags & PID_CONSTANTS_UPDATE)
        {
            pid_const_update.store(true, std::memory_order_release);
        }

        if (receivedFlags & ODO_BROADCAST_STATUS_UPDATE)
        {
            updateOdoBroadcastStatus();
        }

        if (receivedFlags & DESTROY_WHEEL_OBJECT)
        {
            objectDestructor();
        }
    }
}

void Wheel::updateOdometry()
{
    encoder_ticks_t ticks;
    encoder_tickrate_t tickrate;
    timestamp_t timestamp;
    encoder->get_tick_tickrate(ticks, tickrate, timestamp);

    angle = static_cast<angle_t>(ticks);
    angular_velocity = static_cast<angularvelocity_t>(tickrate);

    wheel_data->odometryData.angle.store(angle);
    wheel_data->odometryData.angular_velocity.store(angular_velocity);
    wheel_data->odometryData.timestamp.store(timestamp);
}

void Wheel::PWMDirectControl()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Wheel %d PWM Direct Control task started", wheel_id);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    encoder->start_pulse_counter();
    while (likely(control_task_run.load(std::memory_order_acquire)))
    {
        updateOdometry();
        pwmvalue_t pwm_value = wheel_data->pwmValue.load();
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Wheel %d speed set: %.2f", wheel_id, pwm_value);
        motorDriver->setSpeed(pwm_value);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(loop_delays.PWM));
    }
    motorDriver->setSpeed(0);
    encoder->stop_pulse_counter();
    xTaskNotify(task_handles->wheel_run_task_handle, CONTROL_TASK_DELETED, eSetBits);
    vTaskDelete(NULL);
}

void Wheel::anglePIDControl()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Wheel %d angle PID Control task started", wheel_id);

    angle_t setpoint = 0;
    pwmvalue_t pwm_value = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    encoder->start_pulse_counter();
    auto pid = std::make_unique<PID<angle_t>>(&angle, &pwm_value,
                                              &setpoint,
                                              wheel_data->anglePIDConstants.p,
                                              wheel_data->anglePIDConstants.i,
                                              wheel_data->anglePIDConstants.d,
                                              PID<angle_t>::DIRECT,
                                              1000 / wheel_data->updateFrequenciesWheel.angle_pid);
    pid->SetMode(pid->AUTOMATIC);

    while (likely(control_task_run.load(std::memory_order_acquire)))
    {
        setpoint = wheel_data->setpoint.angle.load();

        updateOdometry();

        pid->Compute();

        wheel_data->pwmValue.store(pwm_value);
        motorDriver->setSpeed(pwm_value);

        ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, " %d pwm: %.2f RPM SP: %.2f INP %.2f",
                            wheel_id, pwm_value, setpoint,
                            angle);

        if (unlikely(pid_const_update.load(std::memory_order_acquire)))
        {
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Wheel %d updating PID constants", wheel_id);
            pid->SetTunings(wheel_data->anglePIDConstants.p,
                            wheel_data->anglePIDConstants.i,
                            wheel_data->anglePIDConstants.d);
            pid_const_update.store(false, std::memory_order_release);
        }

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

    angularvelocity_t setpoint = 0;
    pwmvalue_t pwm_value = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    encoder->start_pulse_counter();
    auto pid = std::make_unique<PID<angularvelocity_t>>(&angular_velocity, &pwm_value,
                                                        &setpoint,
                                                        wheel_data->speedPIDConstants.p,
                                                        wheel_data->speedPIDConstants.i,
                                                        wheel_data->speedPIDConstants.d,
                                                        PID<angularvelocity_t>::DIRECT,
                                                        1000 / wheel_data->updateFrequenciesWheel.speed_pid);

    pid->SetMode(PID<angularvelocity_t>::AUTOMATIC);

    while (likely(control_task_run.load(std::memory_order_acquire)))
    {
        setpoint = wheel_data->setpoint.rpm.load();

        updateOdometry();

        pid->Compute();

        wheel_data->pwmValue.store(pwm_value);
        motorDriver->setSpeed(pwm_value);

        ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "%d pwm: %.2f RPM SP: %.2f odo: %.2f",
                            wheel_id, pwm_value, setpoint,
                            angular_velocity);

        if (unlikely(pid_const_update.load(std::memory_order_acquire)))
        {
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Wheel %d updating PID constants", wheel_id);
            pid->SetTunings(wheel_data->speedPIDConstants.p,
                            wheel_data->speedPIDConstants.i,
                            wheel_data->speedPIDConstants.d);
            pid_const_update.store(false, std::memory_order_release);
        }

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
        updateOdometry();
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
        control_task_run.store(true, std::memory_order_release);
        createTask([](void *param)
                   { static_cast<Wheel *>(param)->PWMDirectControl(); },
                   "PWMDirectControl", &task_handles->control_task_handle, PWM_DIRECT_CONTROL_TASK_STACK_SIZE, PWM_DIRECT_CONTROL_TASK_PRIORITY);
        break;
    case ControlMode::POSITION_CONTROL:
        control_task_run.store(true, std::memory_order_release);
        createTask([](void *param)
                   { static_cast<Wheel *>(param)->anglePIDControl(); },
                   "anglePIDControl", &task_handles->control_task_handle, CONTROL_TASK_STACK_SIZE, CONTROL_TASK_PRIORITY);
        break;
    case ControlMode::SPEED_CONTROL:
        control_task_run.store(true, std::memory_order_release);
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
        wheel_data->control_mode == ControlMode::OFF)
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

    control_task_run.store(false, std::memory_order_release);
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

void Wheel::objectDestructor()
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Stopping Control task for instance %d", wheel_id);
    if (deleteControlTask())
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Control task stopped for instance %d", wheel_id);
    }

    if (task_handles->odo_broadcast != nullptr)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Stopping Odo Broadcast task for instance %d", wheel_id);
        vTaskDelete(task_handles->odo_broadcast);
        task_handles->odo_broadcast = nullptr;
    }

    xTaskNotify(*manage_wheels_taskHandle, WHEEL_DESTROYED, eSetBits);
}
