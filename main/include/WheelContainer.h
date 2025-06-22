#pragma once

#include <vector>
#include <optional>
#include <functional>
#include <atomic>

#include "Wheel.h"
#include "ControlInterface.h"
#include "MyStructs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * @brief Container for managing Wheel objects and their operations
 */
class WheelContainer
{
public:
    WheelContainer();
    ~WheelContainer();

    // Wheel management operations
    void updateWheelCount(uint8_t count);
    void updateWheel(const wheel_data_t &wheel);
    void updateControlMode(uint8_t id, ControlMode mode);
    void updatePIDConstants(uint8_t id, PIDType type, pid_constants_t constants);
    void updateSetpoints(const std::vector<float> &setpoints);
    void updateControlLoopFrequency(frequency_t frequency);

    // Control loop operations
    void executeControlLoop();
    
    // Odometry operations
    std::vector<odometry_t> collectOdometry();
    
    // Queue processing
    void processWheelDataQueue();
    void processControlModeQueue();
    void processPendingWheelCount();

    // Getters
    uint8_t getWheelCount() const { return wheel_count_; }
    const std::vector<std::optional<Wheel>>& getWheels() const { return wheels_; }
    
    // Queue operations
    void queueWheelData(const wheel_data_t &wheel);
    void queueControlMode(uint8_t id, ControlMode mode);
    void setPendingWheelCount(uint8_t count);

private:
    static constexpr const char *TAG = "WheelContainer";
    static constexpr size_t DEFAULT_WHEEL_DATA_QUEUE_SIZE = 3;
    static constexpr size_t DEFAULT_CONTROL_MODE_QUEUE_SIZE = 4;
    static constexpr TickType_t DEFAULT_CONTROL_DELAY_TICKS = pdMS_TO_TICKS(1000);
    static constexpr TickType_t DEFAULT_QUEUE_SEND_TIMEOUT_TICKS = 10;
    static constexpr TickType_t DEFAULT_QUEUE_RECEIVE_TIMEOUT_TICKS = 2;

    std::vector<std::optional<Wheel>> wheels_;
    uint8_t wheel_count_{0};

    // Queue handles for wheel updates
    QueueHandle_t wheel_data_queue = nullptr;
    QueueHandle_t control_mode_queue = nullptr;

    // Pending wheel count for safe updates
    std::atomic<uint8_t> pending_wheel_count{0};

    // Current control loop delay for new wheels
    TickType_t current_control_delay_ticks{DEFAULT_CONTROL_DELAY_TICKS}; // Default 100ms
};
