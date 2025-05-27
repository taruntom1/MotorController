#ifndef PID_h
#define PID_h

#include <algorithm>

template <typename T>
T defaultPredictor(T *setpoint, float *battery_voltage)

{
  return T(0);
}

template <typename T>
class PID
{
public:
  static constexpr int AUTOMATIC = 1;
  static constexpr int MANUAL = 0;
  static constexpr int DIRECT = 0;
  static constexpr int REVERSE = 1;

  PID(T *Input, T *Output, T *Setpoint,
      T Kp, T Ki, T Kd, int ControllerDirection,
      unsigned int sampleTime, T (*predictor)(T *, float *) = defaultPredictor<T>,
      T *batteryVoltage = nullptr);

  void SetMode(int Mode);
  bool Compute();
  void SetOutputLimits(T, T);
  void SetTunings(T, T, T);
  void SetControllerDirection(int);
  void SetSampleTime(int);

  T GetKp();
  T GetKi();
  T GetKd();
  int GetMode();
  int GetDirection();

private:
  void Initialize();

  T dispKp, dispKi, dispKd;
  T kp, ki, kd;

  T (*predictor)(T * setpoint, float * battery_voltage);
  T *batteryVoltage;

  int controllerDirection;

  T *myInput;
  T *myOutput;
  T *mySetpoint;

  unsigned long lastTime;
  T outputSum, lastInput;

  unsigned int SampleTime;
  T outMin, outMax;
  bool inAuto;
};

#include "PID.tpp" // Implementation goes here for template classes

#endif
