// Header File: print_structs.h
#ifndef PRINT_H
#define PRINT_H

#include <iostream>
#include "stdint.h"
#include "MyStructs.h"

using namespace std;


void printOdoBroadcastStatus(const odo_broadcast_flags_t &status);
void printPIDConstants(const pid_constants_t &pid);
void printPWMLimits(const limits_pwm_t &limits);
void printMotorConnections(const connections_wheel_t &conn);
void printOdometryData(const odometry_t &data);
//void printSetpoint(const setpoint_t &setpoint);
void printUpdateFrequenciesWheel(const wheel_update_frequencies_t &freqs);
void printControlMode(const ControlMode &mode);
void printMotorData(const wheel_data_t &motor);
void printUpdateFrequencies(const update_frequencies_t &freqs);
void printControllerProperties(const controller_properties_t &props);
void printControllerData(const controller_data_t &controller);

#endif // PRINT_STRUCTS_H