#include "WheelManager.h"
#include "esp_log.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO

WheelManager::WheelManager(WheelManagerConfig config)
    : control_task_delay_ticks(pdMS_TO_TICKS(1000 / config.control_loop_frequency)),
      odo_broadcast_task_delay_ticks(pdMS_TO_TICKS(1000 / config.odo_broadcast_frequency)),
      wheel_data_queue(xQueueCreate(3, sizeof(wheel_data_t))),
      control_mode_queue(xQueueCreate(4, sizeof(std::pair<uint8_t, ControlMode>)))
{
    xTaskCreate(wheelManageTaskEntry, "WheelManager", 4096, this, 6, &wheel_manage_task_handle);
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
        if (ulNotificationValue & static_cast<uint32_t>(wheel_manager_notifications::NUM_WHEEL_UPDATE))
        {
            changeWheelCount();
        }
        if (ulNotificationValue & static_cast<uint32_t>(wheel_manager_notifications::WHEEL_UPDATE))
        {
            changeRequestedWheels();
        }
        if (ulNotificationValue & static_cast<uint32_t>(wheel_manager_notifications::CONTROL_MODE_UPDATE))
        {
            changeControlMode();
        }

        handleTaskActionNotification(ulNotificationValue);
    }
}

void WheelManager::controlTaskEntry(void *pvParameters)
{
    static_cast<WheelManager *>(pvParameters)->controlTask();
}

void WheelManager::controlTask()
{
    TickType_t last_wake_time;
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
        vTaskDelayUntil(&last_wake_time, control_task_delay_ticks.load(std::memory_order_acquire));
    }
}

void WheelManager::odoBroadcastTaskEntry(void *pvParameters)
{
    static_cast<WheelManager *>(pvParameters)->odoBroadcastTask();
}

void WheelManager::odoBroadcastTask()
{
    odoBroadcastData.second.reserve(wheel_count_);
    TickType_t last_wake_time;

    while (true)
    {
        odoBroadcastData.second.clear();
        odoBroadcastData.first = esp_timer_get_time();

        for (auto &wheel : wheels_)
        {
            odoBroadcastData.second.push_back(wheel.getOdometry());
        }
        odoBroadcastCallback(odoBroadcastData);

        if (odo_broadcast_run.load(std::memory_order_relaxed) == false)
        {
            xTaskNotify(wheel_manage_task_handle,
                        static_cast<uint32_t>(wheel_manager_notifications::ODO_BROADCAST_SUSPENDED),
                        eSetBits);
            vTaskSuspend(NULL);
        }
        vTaskDelayUntil(&last_wake_time, odo_broadcast_task_delay_ticks.load(std::memory_order_acquire));
    }
}

void WheelManager::updateWheelCount(uint8_t count)
{
    wheel_count_ = count;
    xTaskNotify(wheel_manage_task_handle,
                static_cast<uint32_t>(wheel_manager_notifications::NUM_WHEEL_UPDATE), eSetBits);
}

void WheelManager::updateWheel(const wheel_data_t &wheel)
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

void WheelManager::updateOdoBroadcastFrequency(frequency_t frequency)
{
    odo_broadcast_task_delay_ticks = pdMS_TO_TICKS(1000 / frequency);
}

void WheelManager::updateControlLoopFrequency(frequency_t frequency)
{
    control_task_delay_ticks = pdMS_TO_TICKS(1000 / frequency);
    for (auto &wheel : wheels_)
        wheel.updateLoopDelay(frequency);
}

void WheelManager::updatePIDConstants(uint8_t id, PIDType type, pid_constants_t constants)
{
    wheels_.at(id).updatePIDConstants(type, constants);
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "PID constants changed for wheel : %d, Type %d", id, static_cast<uint8_t>(type));
}

void WheelManager::updateSetpoints(const std::vector<float> &setpoints)
{
    if (setpoints.size() < wheels_.size())
        return;
    int i = 0;
    for (auto &wheel : wheels_)
    {
        wheel.updateSetpoint(setpoints[i]);
        i++;
    }
}

void WheelManager::changeWheelCount()
{
    controlLoopTaskAction(TaskAction::Suspend);
    odoBroadcastTaskAction(TaskAction::Suspend);

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Cleaning up previous motor instances");
    wheels_.clear();

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Allocating memory for %d motors", wheel_count_);
    wheels_.reserve(wheel_count_);

    controlLoopTaskAction(TaskAction::Resume);
    odoBroadcastTaskAction(TaskAction::Resume);
    odoBroadcastTaskAction(TaskAction::Start);
    controlLoopTaskAction(TaskAction::Start);
}

void WheelManager::changeRequestedWheels()
{
    controlLoopTaskAction(TaskAction::Suspend);
    odoBroadcastTaskAction(TaskAction::Suspend);

    wheel_data_t wheel_data;
    while (xQueueReceive(wheel_data_queue, &wheel_data, 0) == pdTRUE)
    {
        uint8_t i = wheel_data.motor_id;
        if (i >= wheel_count_)
            continue;

        auto it = std::find_if(wheels_.begin(), wheels_.end(), [i](const Wheel &obj)
                               { return obj.GetWheelID() == i; });

        if (it != wheels_.end())
        {
            it->~Wheel();
            new (&(*it)) Wheel(&wheel_data, control_task_delay_ticks.load());
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Object replaced for wheel id : %d ", i);
        }
        else
        {
            wheels_.emplace_back(&wheel_data, control_task_delay_ticks.load());
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "New object created for wheel id : %d ", i);
        }
    }

    controlLoopTaskAction(TaskAction::Resume);
    odoBroadcastTaskAction(TaskAction::Resume);
}

void WheelManager::changeControlMode()
{
    controlLoopTaskAction(TaskAction::Suspend);
    odoBroadcastTaskAction(TaskAction::Suspend);

    std::pair<uint8_t, ControlMode> control_mode_pair;
    if (xQueueReceive(control_mode_queue, &control_mode_pair, 2) == pdTRUE &&
        control_mode_pair.first < wheel_count_)
    {
        wheels_[control_mode_pair.first].updateControlMode(control_mode_pair.second);
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Control mode changed for wheel id : %d", control_mode_pair.first);
    }

    controlLoopTaskAction(TaskAction::Resume);
    odoBroadcastTaskAction(TaskAction::Resume);
}

void WheelManager::handleTaskActionNotification(uint32_t notification)
{
    if (notification & static_cast<uint32_t>(wheel_manager_notifications::START_CONTROL_LOOP))
    {
        controlLoopTaskAction(TaskAction::Start);
    }
    else if (notification & static_cast<uint32_t>(wheel_manager_notifications::STOP_CONTROL_LOOP))
    {
        controlLoopTaskAction(TaskAction::Stop);
    }
    else if (notification & static_cast<uint32_t>(wheel_manager_notifications::SUSPEND_CONTROL_LOOP))
    {
        controlLoopTaskAction(TaskAction::Suspend);
    }
    else if (notification & static_cast<uint32_t>(wheel_manager_notifications::RESUME_CONTROL_LOOP))
    {
        controlLoopTaskAction(TaskAction::Resume);
    }
    if (notification & static_cast<uint32_t>(wheel_manager_notifications::START_ODO_BROADCAST))
    {
        odoBroadcastTaskAction(TaskAction::Start);
    }
    else if (notification & static_cast<uint32_t>(wheel_manager_notifications::STOP_ODO_BROADCAST))
    {
        odoBroadcastTaskAction(TaskAction::Stop);
    }
    else if (notification & static_cast<uint32_t>(wheel_manager_notifications::SUSPEND_ODO_BROADCAST))
    {
        odoBroadcastTaskAction(TaskAction::Suspend);
    }
    else if (notification & static_cast<uint32_t>(wheel_manager_notifications::RESUME_ODO_BROADCAST))
    {
        odoBroadcastTaskAction(TaskAction::Resume);
    }
}

void WheelManager::controlLoopTaskActionNonBlocking(TaskAction action)
{
    switch (action)
    {
    case TaskAction::Start:
        xTaskNotify(wheel_manage_task_handle, static_cast<uint32_t>(wheel_manager_notifications::START_CONTROL_LOOP), eSetBits);
        break;
    case TaskAction::Stop:
        xTaskNotify(wheel_manage_task_handle, static_cast<uint32_t>(wheel_manager_notifications::STOP_CONTROL_LOOP), eSetBits);
        break;
    case TaskAction::Suspend:
        xTaskNotify(wheel_manage_task_handle, static_cast<uint32_t>(wheel_manager_notifications::SUSPEND_CONTROL_LOOP), eSetBits);
        break;
    case TaskAction::Resume:
        xTaskNotify(wheel_manage_task_handle, static_cast<uint32_t>(wheel_manager_notifications::RESUME_CONTROL_LOOP), eSetBits);
        break;
    default:
        break;
    }
}

void WheelManager::odoBroadcastTaskActionNonBlocking(TaskAction action)
{
    switch (action)
    {
    case TaskAction::Start:
        xTaskNotify(wheel_manage_task_handle, static_cast<uint32_t>(wheel_manager_notifications::START_ODO_BROADCAST), eSetBits);
        break;
    case TaskAction::Stop:
        xTaskNotify(wheel_manage_task_handle, static_cast<uint32_t>(wheel_manager_notifications::STOP_ODO_BROADCAST), eSetBits);
        break;
    case TaskAction::Suspend:
        xTaskNotify(wheel_manage_task_handle, static_cast<uint32_t>(wheel_manager_notifications::SUSPEND_ODO_BROADCAST), eSetBits);
        break;
    case TaskAction::Resume:
        xTaskNotify(wheel_manage_task_handle, static_cast<uint32_t>(wheel_manager_notifications::RESUME_ODO_BROADCAST), eSetBits);
        break;
    default:
        break;
    }
}

bool WheelManager::createControlTask()
{
    if (control_task_handle == nullptr)
    {
        control_loop_run.store(true);
        xTaskCreate(controlTaskEntry, "Control Task", 2000, this, 5, &control_task_handle);
        return true;
    }
    return false;
}

bool WheelManager::createOdoBroadcastTask()
{
    if (odo_broadcast_task_handle == nullptr)
    {
        odo_broadcast_run.store(true);
        xTaskCreate(odoBroadcastTaskEntry, "Odo Broadcast Task", 2500, this, 5, &odo_broadcast_task_handle);
        return true;
    }
    return false;
}

bool WheelManager::controlLoopTaskAction(TaskAction action)
{
    return handleTaskAction(action, control_task_handle, control_task_state_, [this]()
                            { return createControlTask(); }, control_loop_run, [this]()
                            { return suspendAndWaitForControlLoopSuspend(); });
}

bool WheelManager::odoBroadcastTaskAction(TaskAction action)
{
    return handleTaskAction(action, odo_broadcast_task_handle, control_task_state_, [this]()
                            { return createOdoBroadcastTask(); }, odo_broadcast_run, [this]()
                            { return suspendAndWaitForOdoBroadcastSuspend(); });
}

bool WheelManager::handleTaskAction(TaskAction action,
                                    TaskHandle_t &task_handle,
                                    TaskState &task_state,
                                    std::function<bool()> task_creator,
                                    std::atomic<bool> &run_flag,
                                    std::function<bool()> suspend_handler)
{
    if (task_handle == nullptr && action != TaskAction::Start)
        return false;

    switch (action)
    {
    case TaskAction::Start:
        if (task_handle == nullptr || task_state == TaskState::Deleted)
        {
            return task_creator();
        }
        return false;

    case TaskAction::Stop:
        if (task_state != TaskState::Deleted && suspend_handler())
        {
            vTaskDelete(task_handle);
            task_handle = nullptr;
            return true;
        }
        return false;

    case TaskAction::Suspend:
        if (task_state == TaskState::Running)
        {
            return suspend_handler();
        }
        return false;

    case TaskAction::Resume:
        if (task_state == TaskState::Suspended)
        {
            run_flag.store(true);
            vTaskResume(task_handle);
            return true;
        }
        return false;
    }

    return false;
}

bool WheelManager::suspendAndWaitForControlLoopSuspend()
{
    if (control_task_handle == nullptr)
        return true;

    eTaskState state = eTaskGetState(control_task_handle);
    if (state == eDeleted || state == eSuspended)
        return true;

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
    if (odo_broadcast_task_handle == nullptr)
        return true;

    eTaskState state = eTaskGetState(odo_broadcast_task_handle);
    if (state == eDeleted || state == eSuspended)
        return true;

    const uint32_t expected_notification = static_cast<uint32_t>(wheel_manager_notifications::ODO_BROADCAST_SUSPENDED);
    odo_broadcast_run.store(false); // Stop the loop

    uint32_t notification = 0;
    while (!(notification & expected_notification))
    {
        if (xTaskNotifyWait(0, expected_notification, &notification, 10) == pdFAIL)
            return false; // Timeout or failure
    }

    return true;
}
