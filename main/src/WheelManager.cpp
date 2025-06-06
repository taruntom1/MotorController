#include "WheelManager.h"
#include "esp_log.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO

WheelManager::WheelManager()
    : wheel_data_queue(xQueueCreate(3, sizeof(wheel_data_t))),
      control_mode_queue(xQueueCreate(4, sizeof(std::pair<uint8_t, ControlMode>))) {}

void WheelManager::Run()
{
    xTaskCreate(wheelManageTaskEntry, "WheelManager", 4096, this, 5, &wheel_manage_task_handle);
}

void WheelManager::wheelManageTaskEntry(void *pvParameters)
{
    static_cast<WheelManager *>(pvParameters)->WheelManageTask();
}

void WheelManager::WheelManageTask()
{
    static const char *TAG = "WheelManager";

    while (true)
    {
        uint32_t ulNotificationValue;
        xTaskNotifyWait(0, ULONG_MAX, &ulNotificationValue, portMAX_DELAY);
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Motor management task triggered");
        if (ulNotificationValue & static_cast<uint32_t>(wheel_manager_notifications::NUM_WHEEL_UPDATE))
        {
            changeWheelCount();
        }
        if (ulNotificationValue & static_cast<uint32_t>(wheel_manager_notifications::WHEEL_UPDATE))
        {
            changeRequestedWheels();
        }
    }
}

void WheelManager::controlTaskEntry(void *pvParameters)
{
    static_cast<WheelManager *>(pvParameters)->controlTask();
}

void WheelManager::controlTask()
{
    while (true)
    {
        for (auto &wheel : wheels_)
        {
            wheel.updateControlLoop();
        }
        if (control_loop_run.load(std::memory_order_relaxed) == false)
        {
            xTaskNotify(wheel_manage_task_handle,
                        static_cast<uint32_t>(wheel_manager_notifications::CONTROL_LOOP_SUSPENDED),
                        eSetBits);
            vTaskSuspend(NULL);
        }
    }
}

void WheelManager::odoBroadcastTaskEntry(void *pvParameters)
{
    static_cast<WheelManager *>(pvParameters)->odoBroadcastTask();
}

void WheelManager::odoBroadcastTask()
{
    std::vector<std::pair<timestamp_t, odometry_t>> odoBroadcastData;
    odoBroadcastData.reserve(wheel_count_);

    while (true)
    {
        for (auto &wheel : wheels_)
        {
            odoBroadcastData.push_back({esp_timer_get_time(), wheel.getOdometry()});
        }
        odoBroadcastCallback(odoBroadcastData);
        odoBroadcastData.clear();

        if (odo_broadcast_run.load(std::memory_order_relaxed) == false)
        {
            xTaskNotify(wheel_manage_task_handle,
                        static_cast<uint32_t>(wheel_manager_notifications::ODO_BROADCAST_SUSPENDED),
                        eSetBits);
            vTaskSuspend(NULL);
        }
    }
}

void WheelManager::updateWheelCount(uint8_t count)
{
    wheel_count_ = count;
    xTaskNotify(wheel_manage_task_handle,
                static_cast<uint32_t>(wheel_manager_notifications::NUM_WHEEL_UPDATE), eSetBits);
}

void WheelManager::updateWheel(Wheel &wheel)
{
    xQueueSendToBack(wheel_data_queue, &wheel, 10);
    xTaskNotify(wheel_manage_task_handle,
                static_cast<uint32_t>(wheel_manager_notifications::WHEEL_UPDATE), eSetBits);
}

void WheelManager::updateControlMode(uint8_t id, ControlMode mode)
{
    std::pair<uint8_t, ControlMode> control_mode_pair(id, mode);
    xQueueSendToBack(control_mode_queue, &control_mode_pair, 10);
    xTaskNotify(wheel_manage_task_handle,
                static_cast<uint32_t>(wheel_manager_notifications::CONTROL_MODE_UPDATE), eSetBits);
}

void WheelManager::updateOdoBroadcast(bool enabled)
{
    uint32_t notification = static_cast<uint32_t>((enabled) ? wheel_manager_notifications::START_ODO_BROADCAST
                                                            : wheel_manager_notifications::STOP_ODO_BROADCAST);
    xTaskNotify(wheel_manage_task_handle, notification, eSetBits);
}

void WheelManager::updatePIDConstants(uint8_t id, PIDType type, pid_constants_t constants)
{
    wheels_.at(id).updatePIDConstants(type, constants);
}

void WheelManager::updateSetpoints(std::vector<float> &setpoints)
{
    if (setpoints.size() > wheels_.size())
        return;
    int i = 0;
    for (auto &setpoint : setpoints)
    {
        wheels_.at(i).updateSetpoint(setpoint);
        i++;
    }
}

void WheelManager::changeWheelCount()
{
    if (!suspendAndWaitForControlLoopSuspend())
        return;

    uint8_t count = wheel_count_;

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Cleaning up previous motor instances");
    wheels_.clear();

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Allocating memory for %d motors", count);
    wheels_.reserve(count);
}

void WheelManager::changeRequestedWheels()
{
    if (!suspendAndWaitForControlLoopSuspend())
        return;

    wheel_data_t wheel_data;
    while (xQueueReceive(wheel_data_queue, &wheel_data, 0) == pdTRUE)
    {
        uint8_t i = wheel_data.motor_id;
        if (i >= wheels_.size())
            continue;

        auto it = std::find_if(wheels_.begin(), wheels_.end(), [i](const Wheel &obj)
                               { return obj.GetWheelID() == i; });

        if (it != wheels_.end())
        {
            it->~Wheel();
            new (&(*it)) Wheel(&wheel_data);
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "New object created for wheel id : %d ", i);
        }
        else
        {
            wheels_.emplace_back(&wheel_data);
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Object replaced for wheel id : %d ", i);
        }
    }
}

void WheelManager::changeControlMode()
{
    const bool was_running = control_loop_run.load();

    if (!suspendAndWaitForControlLoopSuspend())
        return;

    std::pair<uint8_t, ControlMode> control_mode_pair;
    if (xQueueReceive(control_mode_queue, &control_mode_pair, 2) == pdTRUE &&
        control_mode_pair.first < wheel_count_)
    {
        wheels_[control_mode_pair.first].updateControlMode(control_mode_pair.second);
    }

    if (was_running)
    {
        control_loop_run.store(true);
        vTaskResume(control_task_handle);
    }
}

void WheelManager::startOdoBroadcast()
{
    xTaskCreate(odoBroadcastTaskEntry, "Odo Broadcast Task", 4096, this, 5, &odo_broadcast_task_handle);
}

void WheelManager::startControlLoop()
{
    xTaskCreate(controlTaskEntry, "Control Task", 4096, this, 5, &control_task_handle);
}

bool WheelManager::stopControlLoop()
{
    if (suspendAndWaitForControlLoopSuspend())
    {
        vTaskDelete(control_task_handle);
        return true;
    }
    return false;
}

bool WheelManager::stopOdoBroadcast()
{
    if (suspendAndWaitForOdoBroadcastSuspend())
    {
        vTaskDelete(odo_broadcast_task_handle);
        return true;
    }

    return false;
}

bool WheelManager::suspendAndWaitForControlLoopSuspend()
{
    const uint32_t expected_notification = static_cast<uint32_t>(wheel_manager_notifications::CONTROL_LOOP_SUSPENDED);
    control_loop_run.store(false); // Stop the loop

    uint32_t notification = 0;
    while (!(notification & expected_notification))
    {
        if (xTaskNotifyWait(0, expected_notification, &notification, 10) == pdFAIL)
            return false; // Timeout or failure
    }

    return true;
}

bool WheelManager::suspendAndWaitForOdoBroadcastSuspend()
{
    const uint32_t expected_notification = static_cast<uint32_t>(wheel_manager_notifications::ODO_BROADCAST_SUSPENDED);
    control_loop_run.store(false); // Stop the loop

    uint32_t notification = 0;
    while (!(notification & expected_notification))
    {
        if (xTaskNotifyWait(0, expected_notification, &notification, 10) == pdFAIL)
            return false; // Timeout or failure
    }

    return true;
}
