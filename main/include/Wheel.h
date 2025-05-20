#ifndef WHEEL_H
#define WHEEL_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "MyStructs.h"
#include "MotorDriver.h"
#include "EncoderPulseReader.h"
#include "PID.h"
#include <memory>

struct LoopDelays

{
    uint32_t anglePID = 0;
    uint32_t speedPID = 0;
    uint32_t PWM = 0;
    uint32_t encoder = 0;
};

class Wheel
{
private:
    const char *TAG = "Wheel";

    uint8_t wheel_id;
    wheel_data_t *wheel_data;
    WheelTaskHandles *task_handles;

    LoopDelays loop_delays;

    MotorDriverConfig motor_config;
    std::unique_ptr<MotorDriver> motorDriver;

    pcnt_config_t encoder_config;
    std::unique_ptr<EncoderPulseReader> encoder;

    volatile bool control_task_run = false;

    bool updateControlMode();
    void updateOdoBroadcastStatus();
    void PWMDirectControl();
    void anglePIDControl();
    void speedPIDControl();
    void odoBroadcast();
    void createTask(TaskFunction_t task, const char *name, TaskHandle_t *handle, uint32_t stack_size, UBaseType_t priority);
    bool deleteControlTask();

    void Run();

public:
    Wheel(controller_properties_t *controller_properties, wheel_data_t *wheel_data, WheelTaskHandles *task_handles);
    ~Wheel();
    Wheel(Wheel &&other) noexcept;
    // Move assignment
    Wheel &operator=(Wheel &&other) noexcept;
    // Deleted copy operations
    Wheel(const Wheel &) = delete;
    Wheel &operator=(const Wheel &) = delete;

    uint8_t GetWheelID() const;
};

#endif // WHEEL_H