#include "ControlInterface.h"
#include "Wheel.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <vector>
#include "esp_timer.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO // Set local log level for this file

controller_data_t data;
TaskHandles taskHandles;
protocol_config config;

void ManageWheels(void *pvParameters)
{
    static const char *TAG = "ManageMotors";
    static uint8_t motor_count = 0;
    std::vector<Wheel> wheels;

    while (true)
    {
        uint32_t ulNotificationValue;
        xTaskNotifyWait(0, ULONG_MAX, &ulNotificationValue, portMAX_DELAY);
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Motor management task triggered");
        motor_count = data.controllerProperties.numMotors;

        // Check if the 11th bit (from LSB) is set
        if (ulNotificationValue & (1 << 10)) // 11th bit
        {
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Cleaning up previous motor instances");
            wheels.clear();
            taskHandles.wheel_task_handles.clear();

            // Allocate new motor structures
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "Allocating memory for %d motors", motor_count);
            taskHandles.wheel_task_handles.resize(motor_count);
            wheels.reserve(motor_count);
        }

        // Check if the first 10 bits are not zero
        uint16_t wheelMask = (ulNotificationValue & 0x03FF); // Extract first 10 bits
        for (uint8_t i = 0; i < 10; ++i)
        {
            if (wheelMask & (1 << i)) // If the bit is set, construct the corresponding wheel
            {
                if (i < motor_count) // Ensure within bounds
                {
                    auto it = std::find_if(wheels.begin(), wheels.end(), [i](const Wheel &obj)
                                           { return obj.GetWheelID() == i; });

                    if (it != wheels.end())
                    {
                        it->~Wheel();
                        new (&(*it)) Wheel(&data.controllerProperties, &data.wheelData[i],
                                           &taskHandles.wheel_task_handles[i], &taskHandles.wheel_manager);
                    }
                    else
                    {
                        wheels.emplace_back(&data.controllerProperties, &data.wheelData[i],
                                            &taskHandles.wheel_task_handles[i], &taskHandles.wheel_manager);
                    }
                }
            }
        }
    }
}

void cpu_usage_logger_task(void *param)
{
    char *buffer = new char[1024]; // Increase size if you have many tasks
    if (!buffer)
    {
        ESP_LOGE("CPU_STATS", "Failed to allocate stats buffer");
        vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        vTaskGetRunTimeStats(buffer);
        ESP_LOGI("CPU_STATS", "\nTask\t\tTime\t\t%% CPU\n%s", buffer);
        vTaskDelay(pdMS_TO_TICKS(4000)); // Log every 4 second
    }

    // Shouldn't reach here, but clean up just in case
    free(buffer);
    vTaskDelete(NULL);
}

// Return time in 0.1 microsecond units (FreeRTOS expects increasing counter)
uint32_t get_run_time_counter_value(void)
{
    return (uint32_t)(esp_timer_get_time() / 10);
}

// Required stub — nothing needed if using esp_timer
void configure_timer_for_run_time_stats(void)
{
    // Nothing to configure
}

extern "C" void app_main()
{

    config.port = UART_NUM_0;
    config.baudRate = 115200;
    config.pinRX = 44;
    config.pinTX = 43;

    xTaskCreate(ManageWheels, "Manage Wheels", 4096, NULL, 6, &taskHandles.wheel_manager);
    ControlInterface *controlInterface = new ControlInterface(config, data, taskHandles);
    xTaskCreate(cpu_usage_logger_task, "cpu_logger", 4096, NULL, 1, NULL);
}
