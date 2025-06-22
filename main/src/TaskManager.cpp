#include "TaskManager.h"
#include "WheelContainer.h"
#include "esp_log.h"
#include "esp_timer.h"

TaskManager::TaskManager(TaskManagerConfig config, WheelContainer &wheelContainer)
    : wheel_container_(wheelContainer),
      control_task_delay_ticks(pdMS_TO_TICKS(1000 / config.control_loop_frequency)),
      odo_broadcast_task_delay_ticks(pdMS_TO_TICKS(1000 / config.odo_broadcast_frequency))
{
    xTaskCreate(wheelManageTaskEntry, "WheelManager", 4096, this, 6, &wheel_manage_task_handle);
}

TaskManager::~TaskManager()
{
    // Stop and delete control task if running
    if (control_task_handle != nullptr)
    {
        control_loop_run.store(false);
        vTaskDelete(control_task_handle);
        control_task_handle = nullptr;
    }
    // Stop and delete odometry broadcast task if running
    if (odo_broadcast_task_handle != nullptr)
    {
        odo_broadcast_run.store(false);
        vTaskDelete(odo_broadcast_task_handle);
        odo_broadcast_task_handle = nullptr;
    }
    // Delete wheel manager task if running
    if (wheel_manage_task_handle != nullptr)
    {
        vTaskDelete(wheel_manage_task_handle);
        wheel_manage_task_handle = nullptr;
    }
}

void TaskManager::wheelManageTaskEntry(void *pvParameters)
{
    static_cast<TaskManager *>(pvParameters)->wheelManageTask();
}

void TaskManager::wheelManageTask()
{
    while (true)
    {
        uint32_t ulNotificationValue;
        xTaskNotifyWait(0, ULONG_MAX, &ulNotificationValue, portMAX_DELAY);
        if (ulNotificationValue & static_cast<uint32_t>(task_manager_notifications::NUM_WHEEL_UPDATE))
        {
            controlLoopTaskAction(TaskAction::Suspend);
            odoBroadcastTaskAction(TaskAction::Suspend);

            // Now safely update the wheel count
            wheel_container_.processPendingWheelCount();

            controlLoopTaskAction(TaskAction::Resume);
            odoBroadcastTaskAction(TaskAction::Resume);
            odoBroadcastTaskAction(TaskAction::Start);
            controlLoopTaskAction(TaskAction::Start);
        }

        if (ulNotificationValue & static_cast<uint32_t>(task_manager_notifications::WHEEL_UPDATE))
        {
            controlLoopTaskAction(TaskAction::Suspend);
            odoBroadcastTaskAction(TaskAction::Suspend);

            wheel_container_.processWheelDataQueue();

            controlLoopTaskAction(TaskAction::Resume);
            odoBroadcastTaskAction(TaskAction::Resume);
        }

        if (ulNotificationValue & static_cast<uint32_t>(task_manager_notifications::CONTROL_MODE_UPDATE))
        {
            controlLoopTaskAction(TaskAction::Suspend);
            odoBroadcastTaskAction(TaskAction::Suspend);

            wheel_container_.processControlModeQueue();

            controlLoopTaskAction(TaskAction::Resume);
            odoBroadcastTaskAction(TaskAction::Resume);
        }

        handleTaskActionNotification(ulNotificationValue);
    }
}

void TaskManager::controlTaskEntry(void *pvParameters)
{
    static_cast<TaskManager *>(pvParameters)->controlTask();
}

void TaskManager::controlTask()
{
    TickType_t last_wake_time = xTaskGetTickCount();
    while (true)
    {
        wheel_container_.executeControlLoop();

        if (control_loop_run.load(std::memory_order_acquire) == false)
        {
            notifyTaskManager(task_manager_notifications::CONTROL_LOOP_SUSPENDED);
            vTaskSuspend(NULL);
        }
        vTaskDelayUntil(&last_wake_time, control_task_delay_ticks.load(std::memory_order_acquire));
    }
}

void TaskManager::odoBroadcastTaskEntry(void *pvParameters)
{
    static_cast<TaskManager *>(pvParameters)->odoBroadcastTask();
}

void TaskManager::odoBroadcastTask()
{
    odoBroadcastData.second.reserve(wheel_container_.getWheelCount());
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true)
    {
        odoBroadcastData.first = esp_timer_get_time();
        odoBroadcastData.second = wheel_container_.collectOdometry();

        assert(odoBroadcastCallback && "odoBroadcastCallback must be set before calling");
        odoBroadcastCallback(odoBroadcastData);

        if (odo_broadcast_run.load(std::memory_order_acquire) == false)
        {
            notifyTaskManager(task_manager_notifications::ODO_BROADCAST_SUSPENDED);
            vTaskSuspend(NULL);
        }
        vTaskDelayUntil(&last_wake_time, odo_broadcast_task_delay_ticks.load(std::memory_order_acquire));
    }
}

void TaskManager::updateOdoBroadcastFrequency(frequency_t frequency)
{
    odo_broadcast_task_delay_ticks = pdMS_TO_TICKS(1000 / frequency);
}

void TaskManager::updateControlLoopFrequency(frequency_t frequency)
{
    control_task_delay_ticks = pdMS_TO_TICKS(1000 / frequency);
    wheel_container_.updateControlLoopFrequency(frequency);
}

void TaskManager::handleTaskActionNotification(uint32_t notification)
{
    if (notification & static_cast<uint32_t>(task_manager_notifications::START_CONTROL_LOOP))
    {
        controlLoopTaskAction(TaskAction::Start);
    }
    else if (notification & static_cast<uint32_t>(task_manager_notifications::STOP_CONTROL_LOOP))
    {
        controlLoopTaskAction(TaskAction::Stop);
    }
    else if (notification & static_cast<uint32_t>(task_manager_notifications::SUSPEND_CONTROL_LOOP))
    {
        controlLoopTaskAction(TaskAction::Suspend);
    }
    else if (notification & static_cast<uint32_t>(task_manager_notifications::RESUME_CONTROL_LOOP))
    {
        controlLoopTaskAction(TaskAction::Resume);
    }
    if (notification & static_cast<uint32_t>(task_manager_notifications::START_ODO_BROADCAST))
    {
        odoBroadcastTaskAction(TaskAction::Start);
    }
    else if (notification & static_cast<uint32_t>(task_manager_notifications::STOP_ODO_BROADCAST))
    {
        odoBroadcastTaskAction(TaskAction::Stop);
    }
    else if (notification & static_cast<uint32_t>(task_manager_notifications::SUSPEND_ODO_BROADCAST))
    {
        odoBroadcastTaskAction(TaskAction::Suspend);
    }
    else if (notification & static_cast<uint32_t>(task_manager_notifications::RESUME_ODO_BROADCAST))
    {
        odoBroadcastTaskAction(TaskAction::Resume);
    }
}

void TaskManager::controlLoopTaskActionNonBlocking(TaskAction action)
{
    switch (action)
    {
    case TaskAction::Start:
        notifyTaskManager(task_manager_notifications::START_CONTROL_LOOP);
        break;
    case TaskAction::Stop:
        notifyTaskManager(task_manager_notifications::STOP_CONTROL_LOOP);
        break;
    case TaskAction::Suspend:
        notifyTaskManager(task_manager_notifications::SUSPEND_CONTROL_LOOP);
        break;
    case TaskAction::Resume:
        notifyTaskManager(task_manager_notifications::RESUME_CONTROL_LOOP);
        break;
    default:
        break;
    }
}

void TaskManager::odoBroadcastTaskActionNonBlocking(TaskAction action)
{
    switch (action)
    {
    case TaskAction::Start:
        notifyTaskManager(task_manager_notifications::START_ODO_BROADCAST);
        break;
    case TaskAction::Stop:
        notifyTaskManager(task_manager_notifications::STOP_ODO_BROADCAST);
        break;
    case TaskAction::Suspend:
        notifyTaskManager(task_manager_notifications::SUSPEND_ODO_BROADCAST);
        break;
    case TaskAction::Resume:
        notifyTaskManager(task_manager_notifications::RESUME_ODO_BROADCAST);
        break;
    default:
        break;
    }
}

bool TaskManager::createControlTask()
{
    if (control_task_handle == nullptr)
    {
        control_loop_run.store(true);
        BaseType_t result = xTaskCreate(controlTaskEntry, "Control Task", 2000, this, 5, &control_task_handle);
        if (result == pdPASS)
        {
            control_task_state_ = TaskState::Running;
            return true;
        }
        else
        {
            control_loop_run.store(false);
            control_task_handle = nullptr;
            return false;
        }
    }
    return false;
}

bool TaskManager::createOdoBroadcastTask()
{
    if (odo_broadcast_task_handle == nullptr)
    {
        odo_broadcast_run.store(true);
        BaseType_t result = xTaskCreate(odoBroadcastTaskEntry, "Odo Broadcast Task", 2500, this, 5, &odo_broadcast_task_handle);
        if (result == pdPASS)
        {
            odo_broadcast_task_state_ = TaskState::Running;
            return true;
        }
        else
        {
            odo_broadcast_run.store(false);
            odo_broadcast_task_handle = nullptr;
            return false;
        }
    }
    return false;
}

bool TaskManager::controlLoopTaskAction(TaskAction action)
{
    return handleTaskAction(action, control_task_handle, control_task_state_, [this]()
                            { return createControlTask(); }, control_loop_run, [this]()
                            { return suspendAndWaitForControlLoopSuspend(); });
}

bool TaskManager::odoBroadcastTaskAction(TaskAction action)
{
    return handleTaskAction(action, odo_broadcast_task_handle, odo_broadcast_task_state_, [this]()
                            { return createOdoBroadcastTask(); }, odo_broadcast_run, [this]()
                            { return suspendAndWaitForOdoBroadcastSuspend(); });
}

bool TaskManager::handleTaskAction(TaskAction action,
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
            bool created = task_creator();
            if (created)
                task_state = TaskState::Running;
            return created;
        }
        return false;

    case TaskAction::Stop:
        if (task_state != TaskState::Deleted && suspend_handler())
        {
            vTaskDelete(task_handle);
            task_handle = nullptr;
            task_state = TaskState::Deleted;
            return true;
        }
        return false;

    case TaskAction::Suspend:
        if (task_state == TaskState::Running)
        {
            bool suspended = suspend_handler();
            if (suspended)
                task_state = TaskState::Suspended;
            return suspended;
        }
        return false;

    case TaskAction::Resume:
        if (task_state == TaskState::Suspended)
        {
            run_flag.store(true);
            vTaskResume(task_handle);
            task_state = TaskState::Running;
            return true;
        }
        return false;
    }

    return false;
}

bool TaskManager::suspendAndWaitForControlLoopSuspend()
{
    if (control_task_handle == nullptr)
        return true;

    eTaskState state = eTaskGetState(control_task_handle);
    if (state == eDeleted || state == eSuspended)
    {
        if (state == eDeleted)
            control_task_handle = nullptr;
        return true;
    }

    const uint32_t expected_notification = static_cast<uint32_t>(task_manager_notifications::CONTROL_LOOP_SUSPENDED);
    control_loop_run.store(false); // Stop the loop

    uint32_t notification = 0;
    while (!(notification & expected_notification))
    {
        if (xTaskNotifyWait(0, expected_notification, &notification, 100) == pdFAIL)
            return false; // Timeout or failure
    }

    return true;
}

bool TaskManager::suspendAndWaitForOdoBroadcastSuspend()
{
    if (odo_broadcast_task_handle == nullptr)
        return true;

    eTaskState state = eTaskGetState(odo_broadcast_task_handle);
    if (state == eDeleted || state == eSuspended)
    {
        if (state == eDeleted)
            odo_broadcast_task_handle = nullptr;
        return true;
    }

    const uint32_t expected_notification = static_cast<uint32_t>(task_manager_notifications::ODO_BROADCAST_SUSPENDED);
    odo_broadcast_run.store(false); // Stop the loop

    uint32_t notification = 0;
    while (!(notification & expected_notification))
    {
        if (xTaskNotifyWait(0, expected_notification, &notification, 100) == pdFAIL)
            return false; // Timeout or failure
    }

    return true;
}
