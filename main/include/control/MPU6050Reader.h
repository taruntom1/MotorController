// MPU6050Reader.h

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_err.h>
#include <driver/i2c.h>
#include <driver/gpio.h>
#include "mpu6050.h" // Include the provided C library

#ifdef __cplusplus
extern "C"
{
#endif
// (The C declarations from the provided MPU6050 library go here,
// but since they are already in "mpu6050.h", no need to duplicate.)
#ifdef __cplusplus
}
#endif

/**
 * @brief Struct that holds the 6-axis IMU data (3-axis accelerometer, 3-axis gyroscope)
 */
struct imu_data_t
{
    float acce_x;
    float acce_y;
    float acce_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
};

/**
 * @brief C++ class to read MPU6050 data on ESP-IDF using FreeRTOS tasks and a mutex.
 *
 * Usage:
 *   1. Fill a Config struct.
 *   MPU6050Reader::Config cfg{};
 *   cfg.i2c_port = I2C_NUM_0;
 *   cfg.sda_io_num = GPIO_NUM_21;
 *   cfg.scl_io_num = GPIO_NUM_22;
 *   cfg.i2c_clk_speed_hz = 400000;  400kHz
 *   cfg.dev_addr = MPU6050_I2C_ADDRESS;
 *   cfg.acce_fs = ACCE_FS_4G;       ±4g range
 *   cfg.gyro_fs = GYRO_FS_500DPS;   ±500°/s range
 *   cfg.sample_rate_hz = 50;        50 Hz sampling
 *
 *   IMUData imu_data;   Will hold the latest readings
 *   MPU6050Reader reader(cfg, &imu_data);
 *   reader.run();       Starts the FreeRTOS task reading at 50 Hz
 *
 *    To read safely from imu_data:
 *   reader.takeDataMutex();
 *   float ax = imu_data.acce_x;
 *   float ay = imu_data.acce_y;
 *   float az = imu_data.acce_z;
 *   float gx = imu_data.gyro_x;
 *   float gy = imu_data.gyro_y;
 *   float gz = imu_data.gyro_z;
 *   reader.giveDataMutex();
 *
 *    When done:
 *   reader.stop();
 *    (Destructor also stops the task and cleans up)
 */
class MPU6050Reader
{
public:
    /**
     * @brief Configuration struct for MPU6050Reader
     */
    struct Config
    {
        i2c_port_t i2c_port;       /*!< I2C port number (e.g., I2C_NUM_0) */
        gpio_num_t sda_io_num;     /*!< GPIO number for SDA */
        gpio_num_t scl_io_num;     /*!< GPIO number for SCL */
        uint32_t i2c_clk_speed_hz; /*!< I2C clock speed in Hz */
        uint16_t dev_addr;         /*!< MPU6050 I2C address (0x68 or 0x69) */
        mpu6050_acce_fs_t acce_fs; /*!< Accelerometer full-scale range */
        mpu6050_gyro_fs_t gyro_fs; /*!< Gyroscope full-scale range */
        uint32_t sample_rate_hz;   /*!< Desired sampling rate in Hz */
        UBaseType_t task_priority; /*!< FreeRTOS task priority */
        uint32_t task_stack_size;  /*!< FreeRTOS task stack size (bytes) */
    };

    /**
     * @brief Construct the MPU6050Reader with the given configuration and data pointer.
     *
     * @param config Configuration parameters for I2C, MPU6050 settings, and task.
     * @param out_data_ptr Pointer to an IMUData struct where readings will be stored.
     *                     Must remain valid for the lifetime of this object.
     */
    MPU6050Reader(const Config &config, imu_data_t *out_data_ptr);

    /**
     * @brief Destructor: stops the reading task, deletes MPU6050 handle, and cleans up.
     */
    ~MPU6050Reader();

    /**
     * @brief Starts the FreeRTOS task that periodically reads MPU6050 data.
     *
     * This method spawns a background task. If called multiple times, subsequent calls do nothing.
     */
    void run();

    /**
     * @brief Stops the FreeRTOS reading task (if running).
     *
     * After calling stop(), run() may be called again to restart.
     */
    void stop();

    /**
     * @brief Thread-safe method to get the latest IMU data.
     *
     * This method copies the most recent accelerometer and gyroscope readings
     * from the shared data struct. Access is protected using a mutex to ensure
     * thread safety. If the mutex cannot be acquired within 10 ms, the returned
     * struct will be zero-initialized.
     *
     * @return imu_data_t A copy of the latest IMU readings (accel and gyro).
     */
    imu_data_t readData();

private:
    // Prevent copying
    MPU6050Reader(const MPU6050Reader &) = delete;
    MPU6050Reader &operator=(const MPU6050Reader &) = delete;

    /**
     * @brief FreeRTOS task function: infinite loop reading MPU6050 and writing to IMUData.
     * @param arg Pointer to MPU6050Reader instance.
     */
    static void taskFunc(void *arg);

    /**
     * @brief Internal helper: initialize I2C driver and MPU6050 sensor.
     */
    esp_err_t initializeSensor();

    /**
     * @brief Clean up I2C driver, delete MPU6050 handle, delete mutex.
     */
    void cleanup();

private:
    Config cfg_;
    imu_data_t *data_ptr_;           /*!< Shared data struct pointer */
    mpu6050_handle_t sensor_handle_; /*!< Underlying C handle for MPU6050 */
    SemaphoreHandle_t data_mutex_;   /*!< Mutex protecting data_ptr_ */
    TaskHandle_t task_handle_;       /*!< Handle for the FreeRTOS reading task */
    bool is_running_;                /*!< True if task is running */
};
