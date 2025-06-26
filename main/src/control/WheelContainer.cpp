#include "control/WheelContainer.h"
#include "esp_log.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO

WheelContainer::WheelContainer()
    : wheel_data_queue(xQueueCreate(DEFAULT_WHEEL_DATA_QUEUE_SIZE, wheel_data_t::size)),
      control_mode_queue(xQueueCreate(DEFAULT_CONTROL_MODE_QUEUE_SIZE, sizeof(std::pair<uint8_t, ControlMode>))),
      current_control_delay_ticks(DEFAULT_CONTROL_DELAY_TICKS)
{
}

WheelContainer::~WheelContainer()
{
    // Delete queues
    if (wheel_data_queue != nullptr)
    {
        vQueueDelete(wheel_data_queue);
        wheel_data_queue = nullptr;
    }
    if (control_mode_queue != nullptr)
    {
        vQueueDelete(control_mode_queue);
        control_mode_queue = nullptr;
    }
    // Clear wheels
    wheels_.clear();
}

void WheelContainer::updateWheelCount(uint8_t count)
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Allocating memory for %d motors", count);
    wheel_count_ = count;
    wheels_.resize(wheel_count_);
}

void WheelContainer::updateWheel(const wheel_data_t &wheel)
{
    uint8_t i = wheel.motor_id;
    if (i >= wheel_count_)
    {
        return;
    }

    auto &wheel_slot = wheels_[i];
    if (wheel_slot.has_value())
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Replacing existing wheel at index: %d", i);
    }
    else
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Creating new wheel at index: %d", i);
    }

    wheel_slot.emplace(&wheel, current_control_delay_ticks);
}

void WheelContainer::updateControlMode(uint8_t id, ControlMode mode)
{
    if (id >= wheel_count_)
    {
        return;
    }

    auto &wheel = wheels_[id];
    if (wheel)
    {
        wheel->updateControlMode(mode);
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Control mode changed for wheel id : %d", id);
    }
}

void WheelContainer::updatePIDConstants(uint8_t id, PIDType type, pid_constants_t constants)
{
    if (id >= wheels_.size() || !(wheels_.at(id).has_value()))
    {
        return;
    }

    wheels_.at(id)->updatePIDConstants(type, constants);
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "PID constants changed for wheel : %d, Type %d", id, static_cast<uint8_t>(type));
}

void WheelContainer::updateSetpoints(const std::vector<float> &setpoints)
{
    int i = 0;
    for (auto &wheel : wheels_)
    {
        if (wheel)
        {
            if (i < setpoints.size())
            {
                wheel->updateSetpoint(setpoints[i]);
            }
            else
            {
                wheel->updateSetpoint(0.0f);
            }
            i++;
        }
    }
}

void WheelContainer::updateControlLoopFrequency(frequency_t frequency)
{
    // Calculate delay in ticks - use 1000Hz as default tick rate if not available
    current_control_delay_ticks = (xPortGetTickRateHz() / frequency); // Assuming 1000Hz tick rate

    for (auto &wheel_opt : wheels_)
    {
        if (wheel_opt)
        {
            wheel_opt->updateLoopDelay(frequency);
        }
    }
}

void WheelContainer::executeControlLoop()
{
    for (auto &wheel_opt : wheels_)
    {
        if (wheel_opt)
        {
            wheel_opt->updateControlLoop();
        }
    }
}

std::vector<odometry_t> WheelContainer::collectOdometry()
{
    std::vector<odometry_t> odometry_data;
    odometry_data.reserve(wheel_count_);

    for (auto &wheel_opt : wheels_)
    {
        if (wheel_opt)
        {
            odometry_data.push_back(wheel_opt->getOdometry());
        }
    }

    return odometry_data;
}

void WheelContainer::queueWheelData(const wheel_data_t &wheel)
{
    xQueueSendToBack(wheel_data_queue, &wheel, DEFAULT_QUEUE_SEND_TIMEOUT_TICKS);
}

void WheelContainer::queueControlMode(uint8_t id, ControlMode mode)
{
    std::pair<uint8_t, ControlMode> control_mode_pair(id, mode);
    xQueueSendToBack(control_mode_queue, &control_mode_pair, DEFAULT_QUEUE_SEND_TIMEOUT_TICKS);
}

void WheelContainer::setPendingWheelCount(uint8_t count)
{
    pending_wheel_count.store(count, std::memory_order_release);
}

void WheelContainer::processPendingWheelCount()
{
    uint8_t new_wheel_count = pending_wheel_count.load(std::memory_order_acquire);
    if (new_wheel_count != wheel_count_)
    {
        updateWheelCount(new_wheel_count);
    }
}

void WheelContainer::processWheelDataQueue()
{
    wheel_data_t wheel_data;
    while (xQueueReceive(wheel_data_queue, &wheel_data, DEFAULT_QUEUE_RECEIVE_TIMEOUT_TICKS) == pdTRUE)
    {
        updateWheel(wheel_data);
    }
}

void WheelContainer::processControlModeQueue()
{
    std::pair<uint8_t, ControlMode> control_mode_pair;
    if (xQueueReceive(control_mode_queue, &control_mode_pair, DEFAULT_QUEUE_RECEIVE_TIMEOUT_TICKS) == pdTRUE)
    {
        updateControlMode(control_mode_pair.first, control_mode_pair.second);
    }
}
