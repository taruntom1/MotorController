#include "Print.h"

void printOdoBroadcastStatus(const odo_broadcast_flags_t &status)
{
    cout << "OdoBroadcastStatus: angle=" << status.angle
         << ", speed=" << status.speed
         << ", pwmBroadcast=" << status.pwm_value << endl;
}

void printPIDConstants(const pid_constants_t &pid)
{
    cout << "PIDConstants: P=" << pid.p << ", I=" << pid.i << ", D=" << pid.d << endl;
}

void printPWMLimits(const limits_pwm_t &limits)
{
    cout << "PWMLimits: min=" << (int)limits.min << ", max=" << (int)limits.max << endl;
}

void printMotorConnections(const connections_wheel_t &conn)
{
    cout << "MotorConnections: dir=" << (int)conn.dir
         << ", pwm=" << (int)conn.pwm
         << ", enc_a=" << (int)conn.enc_a
         << ", enc_b=" << (int)conn.enc_b << endl;
}

void printOdometryData(const odometry_t &data)
{
    cout << "OdometryData: angle=" << data.angle << ", rpm=" << data.angular_velocity << endl;
}

void printSetpoint(const setpoint_t &setpoint)
{
    cout << "Setpoint: angle=" << setpoint.angle << ", rpm=" << setpoint.rpm << endl;
}

void printUpdateFrequenciesWheel(const wheel_update_frequencies_t &freqs)
{
    cout << "UpdateFrequenciesWheel: pwm=" << freqs.pwm
         << ", anglePID=" << freqs.angle_pid
         << ", speed_pid=" << freqs.speed_pid << endl;
}

void printControlMode(const ControlMode &mode)
{
    cout << "ControlMode: ";
    switch (mode)
    {
    case ControlMode::POSITION_CONTROL:
        cout << "POSITION_CONTROL";
        break;
    case ControlMode::SPEED_CONTROL:
        cout << "SPEED_CONTROL";
        break;
    case ControlMode::PWM_DIRECT_CONTROL:
        cout << "PWM_DIRECT_CONTROL";
        break;
    case ControlMode::OFF:
        cout << "OFF";
        break;
    }
    cout << endl;
}

void printMotorData(const wheel_data_t &motor)
{
    cout << "MotorData: motor_id=" << (int)motor.motor_id << endl;
    printControlMode(motor.control_mode);
    printPIDConstants(motor.anglePIDConstants);
    printPIDConstants(motor.speedPIDConstants);
    printMotorConnections(motor.motorConnections);
    printOdometryData(motor.odometryData);
    printSetpoint(motor.setpoint);
    printOdoBroadcastStatus(motor.odoBroadcastStatus);
    printUpdateFrequenciesWheel(motor.updateFrequenciesWheel);
    cout << "PWM Value: " << motor.pwmValue << endl;
}

void printUpdateFrequencies(const update_frequencies_t &freqs)
{
    cout << "UpdateFrequencies: interfaceRun=" << freqs.interfaceRun << endl;
}

void printControllerProperties(const controller_properties_t &props)
{
    cout << "ControllerProperties: run=" << props.run
         << ", numMotors=" << (int)props.numMotors
         << ", odoBroadcastFrequency=" << props.odoBroadcastFrequency << endl;
    printOdoBroadcastStatus(props.odoBroadcastStatus);
    printUpdateFrequencies(props.updateFrequencies);
}

void printControllerData(const controller_data_t &controller)
{
    cout << "ControllerData:" << endl;
    for (uint8_t i = 0; i < controller.controllerProperties.numMotors; ++i)
    {
        cout << "\nMotor " << (int)i << ":" << endl;
        printMotorData(controller.wheelData[i]);
    }
    printControllerProperties(controller.controllerProperties);
}
