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
private:
    // Logger Tag
    const char *TAG = "Wheel";

    // Identification
    uint8_t wheel_id;

    // References to shared data
    wheel_data_t *wheel_data;
    WheelTaskHandles *task_handles;
    TaskHandle_t *manage_wheels_taskHandle;

    // Timing and control
    LoopDelays loop_delays;

    // Motor and Encoder
    MotorDriverConfig motor_config;
    std::unique_ptr<MotorDriver> motorDriver;

    pcnt_config_t encoder_config;
    std::unique_ptr<EncoderPulseReader> encoder;

    // State flags
    std::atomic<bool> control_task_run{false};
    std::atomic<bool> pid_const_update{false};

    // Initialization Methods
    void InitLoopDelays(controller_properties_t *controller_properties);
    void InitMotorDriver();
    void InitEncoder();
    void InitTaskHandles();
    void StartWheelTask();

    // Private Methods
    void objectDestructor();
    bool updateControlMode();
    void updateOdoBroadcastStatus();

    void PWMDirectControl();
    void anglePIDControl();
    void speedPIDControl();
    void odoBroadcast();

    void createTask(TaskFunction_t task, const char *name, TaskHandle_t *handle,
                    uint32_t stack_size, UBaseType_t priority);
    bool deleteControlTask();

    void Run();

public:
    // Constructor & Destructor
    Wheel(controller_properties_t *controller_properties, wheel_data_t *wheel_data,
          WheelTaskHandles *task_handles, TaskHandle_t *manage_wheels_taskHandle);

    ~Wheel();

    // Move Semantics
    Wheel(Wheel &&other) noexcept;
    Wheel &operator=(Wheel &&other) noexcept;

    // Deleted Copy Semantics
    Wheel(const Wheel &) = delete;
    Wheel &operator=(const Wheel &) = delete;

    // Getters
    uint8_t GetWheelID() const;
};
