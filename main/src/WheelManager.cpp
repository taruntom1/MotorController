#include "WheelManager.h"
#include "esp_log.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO

WheelManager::WheelManager(controller_data_t *controllerData, TaskHandles *handles)
    : data_(controllerData), taskHandles_(handles) {}

void WheelManager::TaskEntry(void *pvParameters)
{
    static_cast<WheelManager *>(pvParameters)->Task();
}

void WheelManager::Task()
{
    static const char *TAG = "WheelManager";

    while (true)
    {
        uint32_t ulNotificationValue;
        xTaskNotifyWait(0, ULONG_MAX, &ulNotificationValue, portMAX_DELAY);
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Motor management task triggered");

        if (ulNotificationValue & (1 << 10))
        {
            ReallocateWheels();
        }

        UpdateRequestedWheels(ulNotificationValue & 0x03FF); // lower 10 bits
    }
}

void WheelManager::ReallocateWheels()
{
    static const char *TAG = "WheelManager";

    uint8_t motor_count = data_->controllerProperties.numMotors;
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Cleaning up previous motor instances");
    wheels_.clear();
    taskHandles_->wheel_task_handles.clear();

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Allocating memory for %d motors", motor_count);
    taskHandles_->wheel_task_handles.resize(motor_count);
    wheels_.reserve(motor_count);
}

void WheelManager::UpdateRequestedWheels(uint16_t wheelMask)
{
    uint8_t motor_count = data_->controllerProperties.numMotors;

    for (uint8_t i = 0; i < 10; ++i)
    {
        if (wheelMask & (1 << i) && i < motor_count)
        {
            auto it = std::find_if(wheels_.begin(), wheels_.end(), [i](const Wheel &obj)
                                   { return obj.GetWheelID() == i; });

            if (it != wheels_.end())
            {
                it->~Wheel();
                new (&(*it)) Wheel(&data_->controllerProperties, &data_->wheelData[i],
                                   &taskHandles_->wheel_task_handles[i], &taskHandles_->wheel_manager);
            }
            else
            {
                wheels_.emplace_back(&data_->controllerProperties, &data_->wheelData[i],
                                     &taskHandles_->wheel_task_handles[i], &taskHandles_->wheel_manager);
            }
        }
    }
}
