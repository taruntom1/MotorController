#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "ControlInterface.h"
#include "WheelManager.h"
#include "MPU6050Reader.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO // Set local log level for this file

controller_data_t data;
TaskHandles taskHandles;
protocol_config config;

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
    config.baudRate = 576000;
    config.pinRX = 44;
    config.pinTX = 43;

    static WheelManager wheelManager(&data, &taskHandles);
    xTaskCreate(WheelManager::wheelManageTaskEntry, "Manage Wheels", 4096, &wheelManager, 6, &taskHandles.wheel_manager);

    ControlInterface *controlInterface = new ControlInterface(config, data, taskHandles);

    MPU6050Reader::Config cfg{};
    cfg.i2c_port = I2C_NUM_0;
    cfg.sda_io_num = GPIO_NUM_41;
    cfg.scl_io_num = GPIO_NUM_42;
    cfg.i2c_clk_speed_hz = 400000;
    cfg.dev_addr = MPU6050_I2C_ADDRESS;
    cfg.acce_fs = ACCE_FS_4G;
    cfg.gyro_fs = GYRO_FS_500DPS;
    cfg.sample_rate_hz = 50;
    cfg.task_priority = tskIDLE_PRIORITY + 1;
    cfg.task_stack_size = 4096;

    imu_data_t imu_data{};
    

    static MPU6050Reader imuReader(cfg, &imu_data);

    xTaskCreate(cpu_usage_logger_task, "cpu_logger", 4096, NULL, 1, NULL);
}
