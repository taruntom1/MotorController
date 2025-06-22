#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "ControllerManager.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO // Set local log level for this file

// Static constants for configuration
static constexpr size_t CPU_STATS_BUFFER_SIZE = 1024;
static constexpr TickType_t CPU_STATS_LOG_INTERVAL_MS = 4000;
static constexpr UBaseType_t CPU_LOGGER_TASK_STACK_SIZE = 3000;
static constexpr UBaseType_t CPU_LOGGER_TASK_PRIORITY = 1;
static constexpr uint32_t UART_BAUD_RATE = 576000;
static constexpr int UART_RX_PIN = 44;
static constexpr int UART_TX_PIN = 43;
static constexpr uint32_t TIME_COUNTER_DIVISOR = 10; // For 0.1 microsecond units

protocol_config config;

void print_all_tasks_stack_high_watermark()
{
    // Get the number of tasks
    UBaseType_t num_tasks = uxTaskGetNumberOfTasks();

    // Allocate memory to store task status
    TaskStatus_t *task_status_array = (TaskStatus_t *)pvPortMalloc(num_tasks * sizeof(TaskStatus_t));
    if (task_status_array == NULL)
    {
        ESP_LOGE("StackMonitor", "Failed to allocate memory for task status");
        return;
    }

    // Get system state (snapshot of all tasks)
    UBaseType_t array_size = uxTaskGetSystemState(task_status_array, num_tasks, NULL);

    // Print high watermark for each task
    ESP_LOGI("StackMonitor", "Task Name        | High Watermark (words)");
    ESP_LOGI("StackMonitor", "-----------------|-------------------------");

    for (UBaseType_t i = 0; i < array_size; i++)
    {
        ESP_LOGI("StackMonitor", "%-16s| %lu",
                 task_status_array[i].pcTaskName,
                 task_status_array[i].usStackHighWaterMark);
    }

    // Free allocated memory
    vPortFree(task_status_array);
}

void cpu_usage_logger_task(void *param)
{
    char *buffer = new char[CPU_STATS_BUFFER_SIZE];
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
        //print_all_tasks_stack_high_watermark();
        vTaskDelay(pdMS_TO_TICKS(CPU_STATS_LOG_INTERVAL_MS));
    }

    // Shouldn't reach here, but clean up just in case
    free(buffer);
    vTaskDelete(NULL);
}

// Return time in 0.1 microsecond units (FreeRTOS expects increasing counter)
uint32_t get_run_time_counter_value(void)
{
    return (uint32_t)(esp_timer_get_time() / TIME_COUNTER_DIVISOR);
}

// Required stub — nothing needed if using esp_timer
void configure_timer_for_run_time_stats(void)
{
    // Nothing to configure
}

extern "C" void app_main()
{
    config.port = UART_NUM_0;
    config.baudRate = UART_BAUD_RATE;
    config.pinRX = UART_RX_PIN;
    config.pinTX = UART_TX_PIN;

    // Initialize the singleton ControllerManager
    ControllerManager::getInstance();

    // MPU6050Reader::Config cfg{};
    // cfg.i2c_port = I2C_NUM_0;
    // cfg.sda_io_num = GPIO_NUM_41;
    // cfg.scl_io_num = GPIO_NUM_42;
    // cfg.i2c_clk_speed_hz = 400000;
    // cfg.dev_addr = MPU6050_I2C_ADDRESS;
    // cfg.acce_fs = ACCE_FS_4G;
    // cfg.gyro_fs = GYRO_FS_500DPS;
    // cfg.sample_rate_hz = 50;
    // cfg.task_priority = tskIDLE_PRIORITY + 1;
    // cfg.task_stack_size = 4096;

    // imu_data_t imu_data{};

    // static MPU6050Reader imuReader(cfg, &imu_data);

    xTaskCreate(cpu_usage_logger_task, "cpu_logger", CPU_LOGGER_TASK_STACK_SIZE, NULL, CPU_LOGGER_TASK_PRIORITY, NULL);
}
