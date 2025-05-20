#include "MPU6050Reader.h"
#include <cstring> // For memset
#include <cstdio>  // For printf (optional debugging)

MPU6050Reader::MPU6050Reader(const Config &config, imu_data_t *out_data_ptr)
    : cfg_(config),
      data_ptr_(out_data_ptr),
      sensor_handle_(nullptr),
      data_mutex_(nullptr),
      task_handle_(nullptr),
      is_running_(false)
{
    // Initialize the shared data to zeros
    if (data_ptr_)
    {
        data_ptr_->acce_x = 0.0f;
        data_ptr_->acce_y = 0.0f;
        data_ptr_->acce_z = 0.0f;
        data_ptr_->gyro_x = 0.0f;
        data_ptr_->gyro_y = 0.0f;
        data_ptr_->gyro_z = 0.0f;
    }

    // Create a mutex for protecting data_ptr_
    data_mutex_ = xSemaphoreCreateMutex();
    if (!data_mutex_)
    {
        // Failed to create mutex
        printf("MPU6050Reader: Failed to create data mutex\n");
        return;
    }

    // Initialize sensor (I2C driver + MPU6050 handle + configuration)
    esp_err_t err = initializeSensor();
    if (err != ESP_OK)
    {
        printf("MPU6050Reader: Sensor initialization failed: %d\n", err);
        cleanup();
    }
}

MPU6050Reader::~MPU6050Reader()
{
    stop();    // Ensure the reading task is stopped
    cleanup(); // Clean up resources
}

esp_err_t MPU6050Reader::initializeSensor()
{
    esp_err_t ret;

    // 1. Configure I2C peripheral
    i2c_config_t i2c_conf{};
    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.sda_io_num = cfg_.sda_io_num;
    i2c_conf.scl_io_num = cfg_.scl_io_num;
    i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.master.clk_speed = cfg_.i2c_clk_speed_hz;
    ret = i2c_param_config(cfg_.i2c_port, &i2c_conf);
    if (ret != ESP_OK)
    {
        printf("MPU6050Reader: i2c_param_config failed: %d\n", ret);
        return ret;
    }

    ret = i2c_driver_install(cfg_.i2c_port, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK)
    {
        printf("MPU6050Reader: i2c_driver_install failed: %d\n", ret);
        return ret;
    }

    // 2. Create and initialize MPU6050 handle
    sensor_handle_ = mpu6050_create(cfg_.i2c_port, cfg_.dev_addr);
    if (sensor_handle_ == nullptr)
    {
        printf("MPU6050Reader: mpu6050_create returned NULL\n");
        return ESP_FAIL;
    }

    // 3. Wake up the sensor (it defaults to sleep on power-up)
    ret = mpu6050_wake_up(sensor_handle_);
    if (ret != ESP_OK)
    {
        printf("MPU6050Reader: mpu6050_wake_up failed: %d\n", ret);
        return ret;
    }

    // 4. Configure accelerometer and gyroscope full-scale ranges
    ret = mpu6050_config(sensor_handle_, cfg_.acce_fs, cfg_.gyro_fs);
    if (ret != ESP_OK)
    {
        printf("MPU6050Reader: mpu6050_config failed: %d\n", ret);
        return ret;
    }

    return ESP_OK;
}

void MPU6050Reader::run()
{
    if (is_running_ || sensor_handle_ == nullptr)
    {
        // Already running or invalid handle
        return;
    }

    // Create the FreeRTOS task
    BaseType_t xReturned = xTaskCreate(
        &MPU6050Reader::taskFunc,
        "MPU6050_Read_Task",
        cfg_.task_stack_size / sizeof(StackType_t),
        this,
        cfg_.task_priority,
        &task_handle_);

    if (xReturned != pdPASS)
    {
        printf("MPU6050Reader: Failed to create reading task\n");
        task_handle_ = nullptr;
        is_running_ = false;
    }
    else
    {
        is_running_ = true;
    }
}

void MPU6050Reader::stop()
{
    if (!is_running_)
    {
        return;
    }
    // Delete the FreeRTOS task
    if (task_handle_)
    {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }
    is_running_ = false;
}

void MPU6050Reader::cleanup()
{
    // Delete MPU6050 handle
    if (sensor_handle_)
    {
        mpu6050_delete(sensor_handle_);
        sensor_handle_ = nullptr;
    }

    // Uninstall I2C driver
    // Note: If you plan to use I2C for other peripherals, skip this.
    i2c_driver_delete(cfg_.i2c_port);

    // Delete mutex
    if (data_mutex_)
    {
        vSemaphoreDelete(data_mutex_);
        data_mutex_ = nullptr;
    }
}

imu_data_t MPU6050Reader::readData()
{
    imu_data_t copy;
    if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        copy = *data_ptr_;
        xSemaphoreGive(data_mutex_);
    }
    else
    {
        copy = {}; // Zero-initialize if mutex not acquired
    }
    return copy;
}

void MPU6050Reader::taskFunc(void *arg)
{
    auto *self = static_cast<MPU6050Reader *>(arg);
    const TickType_t delay_ticks = pdMS_TO_TICKS(1000 / self->cfg_.sample_rate_hz);

    mpu6050_acce_value_t acce_val{};
    mpu6050_gyro_value_t gyro_val{};

    while (true)
    {
        // 1. Read accelerometer values
        if (mpu6050_get_acce(self->sensor_handle_, &acce_val) != ESP_OK)
        {
            // Optionally log or handle error
        }

        // 2. Read gyroscope values
        if (mpu6050_get_gyro(self->sensor_handle_, &gyro_val) != ESP_OK)
        {
            // Optionally log or handle error
        }

        // 3. Write into shared IMUData struct under mutex
        if (self->data_mutex_)
        {
            if (xSemaphoreTake(self->data_mutex_, 0) == pdTRUE)
            {
                self->data_ptr_->acce_x = acce_val.acce_x;
                self->data_ptr_->acce_y = acce_val.acce_y;
                self->data_ptr_->acce_z = acce_val.acce_z;
                self->data_ptr_->gyro_x = gyro_val.gyro_x;
                self->data_ptr_->gyro_y = gyro_val.gyro_y;
                self->data_ptr_->gyro_z = gyro_val.gyro_z;
                xSemaphoreGive(self->data_mutex_);
            }
        }

        // 4. Delay until next cycle
        vTaskDelay(delay_ticks);
    }

    // Should never reach here; if loop exits, delete the task
    vTaskDelete(nullptr);
}
