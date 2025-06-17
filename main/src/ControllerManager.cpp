#include "ControllerManager.h"
#include "MPU6050Reader.h"

ControllerManager::ControllerManager()
    : wheel_manager_({100, 50}),
      control_interface_({576000, UART_NUM_0, 2048, 44, 43, 0xAA, 100}),
      mpu6050_reader_(nullptr)
{
    connectCallbacks();

    wheel_manager_.controlLoopTaskActionNonBlocking(TaskAction::Start);
}

ControllerManager::~ControllerManager()
{
    deleteIMU();
}

void ControllerManager::createIMU(const imu_config_t &config)
{
    deleteIMU();
    MPU6050Reader::config cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = static_cast<gpio_num_t>(config.sda_pin),
        .scl_io_num = static_cast<gpio_num_t>(config.scl_pin),
        .i2c_clk_speed_hz = 400000,
        .dev_addr = MPU6050_I2C_ADDRESS,
        .acce_fs = ACCE_FS_2G,
        .gyro_fs = GYRO_FS_250DPS,
        .sample_rate_hz = config.sample_rate_hz,
        .task_priority = 5,
        .task_stack_size = 4096};
    mpu6050_reader_ = new MPU6050Reader(cfg);
    mpu6050_reader_->run();
}

void ControllerManager::deleteIMU()
{
    if (mpu6050_reader_)
    {
        delete mpu6050_reader_;
        mpu6050_reader_ = nullptr;
    }
}

void ControllerManager::setControllerProperties(const controller_properties_t &controller_properties)
{
    wheel_manager_.updateWheelCount(controller_properties.numMotors);
    wheel_manager_.updateControlLoopFrequency(controller_properties.updateFrequencies.control_run_frequency);
    wheel_manager_.updateOdoBroadcastFrequency(controller_properties.updateFrequencies.odoBroadcastFrequency);
}

void ControllerManager::connectCallbacks()
{
    // From contorl_interface_ to ControllerManager
    control_interface_.setControllerPropertiesCallback([this](controller_properties_t &properties)
                                                       { this->setControllerProperties(properties); });

    // From wheel_manager_ to control_interface_
    wheel_manager_.setOdoBroadcastCallback([this](const std::pair<timestamp_t, std::vector<odometry_t>> &data)
                                           { this->control_interface_.SendOdoData(data); });

    // From control_interface_ to wheel_manager_
    control_interface_.setWheelDataCallback(
        [this](const wheel_data_t &data)
        {
            wheel_manager_.updateWheel(data);
        });

    control_interface_.setWheelControlModeCallback(
        [this](uint8_t id, ControlMode mode)
        {
            wheel_manager_.updateControlMode(id, mode);
        });

    control_interface_.setOdoBroadcastCallbackNonBlocking(
        [this](TaskAction action)
        {
            wheel_manager_.odoBroadcastTaskActionNonBlocking(action);
        });
    control_interface_.setOdoBroadcastCallbackBlocking(
        [this](TaskAction action)
        {
            wheel_manager_.odoBroadcastTaskAction(action);
        });
    control_interface_.setPIDConstantsCallback(
        [this](uint8_t id, PIDType type, pid_constants_t constants)
        {
            wheel_manager_.updatePIDConstants(id, type, constants);
        });

    control_interface_.setWheelSetpointCallback(
        [this](const std::vector<float> &setpoints)
        {
            wheel_manager_.updateSetpoints(setpoints);
        });
}
