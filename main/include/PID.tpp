#ifndef PID_TPP
#define PID_TPP

#include <algorithm>

template <typename T>
PID<T>::PID(T *Input, T *Output, T *Setpoint,
            T Kp, T Ki, T Kd, int ControllerDirection,
            unsigned int sampleTime, T (*predictor)(T *, float *),
            T *batteryVoltage)
    : predictor(predictor), batteryVoltage(batteryVoltage)
{
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  inAuto = false;

  SetOutputLimits(T(-1), T(1));
  SetControllerDirection(ControllerDirection);
  SampleTime = sampleTime;
  SetTunings(Kp, Ki, Kd);
}

template <typename T>
bool PID<T>::Compute()
{
  if (!inAuto)
    return false;

  T input = *myInput;
  T error = *mySetpoint - input;
  T dInput = (input - lastInput);
  outputSum += (ki * error);

  outputSum = std::clamp(outputSum, outMin, outMax);

  T output = kp * error + outputSum - kd * dInput + predictor(mySetpoint, batteryVoltage);
  *myOutput = std::clamp(output, outMin, outMax);

  lastInput = input;
  return true;
}

template <typename T>
void PID<T>::SetTunings(T Kp, T Ki, T Kd)
{
  if (Kp < 0 || Ki < 0 || Kd < 0)
    return;

  dispKp = Kp;
  dispKi = Ki;
  dispKd = Kd;

  T SampleTimeInSec = static_cast<T>(SampleTime) / 1000;
  kp = Kp;
  ki = Ki * SampleTimeInSec;
  kd = Kd / SampleTimeInSec;

  if (controllerDirection == REVERSE)
  {
    kp = -kp;
    ki = -ki;
    kd = -kd;
  }
}

template <typename T>
void PID<T>::SetSampleTime(int NewSampleTime)
{
  if (NewSampleTime > 0)
  {
    T ratio = static_cast<T>(NewSampleTime) / static_cast<T>(SampleTime);
    ki *= ratio;
    kd /= ratio;
    SampleTime = static_cast<unsigned int>(NewSampleTime);
  }
}

template <typename T>
void PID<T>::SetOutputLimits(T Min, T Max)
{
  if (Min >= Max)
    return;
  outMin = Min;
  outMax = Max;

  if (inAuto)
  {
    *myOutput = std::clamp(*myOutput, outMin, outMax);
    outputSum = std::clamp(outputSum, outMin, outMax);
  }
}

template <typename T>
void PID<T>::SetMode(int Mode)
{
  bool newAuto = (Mode == AUTOMATIC);
  if (newAuto && !inAuto)
  {
    Initialize();
  }
  inAuto = newAuto;
}

template <typename T>
void PID<T>::Initialize()
{
  outputSum = *myOutput;
  lastInput = *myInput;
  outputSum = std::clamp(outputSum, outMin, outMax);
}

template <typename T>
void PID<T>::SetControllerDirection(int Direction)
{
  if (inAuto && Direction != controllerDirection)
  {
    kp = -kp;
    ki = -ki;
    kd = -kd;
  }
  controllerDirection = Direction;
}

template <typename T>
T PID<T>::GetKp() { return dispKp; }

template <typename T>
T PID<T>::GetKi() { return dispKi; }

template <typename T>
T PID<T>::GetKd() { return dispKd; }

template <typename T>
int PID<T>::GetMode() { return inAuto ? AUTOMATIC : MANUAL; }

template <typename T>
int PID<T>::GetDirection() { return controllerDirection; }

#endif
