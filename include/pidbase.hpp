/**
 * @file pid.hpp
 * @author Karan Sutradhar: Driver
 * @author Ajinkya Parwekar: Navigator
 * @brief Definition of a PID Controller base class for Ackerman Steering Mechanism
 * It uses controller gain values and returns output value based on setpoint and feedback values.
 * @Copyright "Copyright 2020" <Ajinkya Parwekar>
 * @Copyright "Copyright 2020" <Karan Sutradhar>
 */

#pragma once

#include <math.h>
#include <iostream>
#include <chrono>
#include <string>
#include <vector>
#define OUTMIN -1e6     // default minimum saturation value
#define OUTMAX 1e6    // default maximum saturation value


// Declaring class definition
class pidbase {
 protected:
  double kp, kd, ki, kb, errorSum, previousError, integralError, dt;
  double setpoint;
  bool dtMode, firstRunFlag;
  std::chrono::system_clock::time_point prevTime;
  double baseline, carLen, arcRadius, rWheelVel, lWheelVel,
    steeringAngle, setpointSpeed, setpointHeading;
  double dtSim, posX, posY, updatedHeading;
  double leftWheelSpeed, rightWheelSpeed;
  double n;
  uint32_t t;
  uint8_t antiWindUp;
  std::vector<double> vectorOutput;
  double outMin, outMax;
  double backCalcOld = 0.0;
  double kf, CnP, CnI, CnD;
  double tSec;
  double x, a, b, min, max;

 public:
  /**
   * @brief Base Constructor for the PID Controller class.
   * @param None.
   * @return None.
   */

  pidbase();

  /**
   * @brief Function to set the proportional gain variable of the PID controller
   * @param kpIn (Proportional gain)
   * @return None.
   */

  virtual double setKp(double kdIn);

  /**
   * @brief Function to set the differential gain variable of the PID controller
   * @param kd (Differential gain)
   * @return None.
   */

  virtual double setKd(double kdIn);

  /**
   * @brief Function to set the integral gain variable of the PID controller
   * @param ki (Integral gain)
   * @return None.
   */

  virtual double setKi(double kiIn);

  /**
   * @brief Function to set the time variable of the PID controller
   * @param dt (time variable)
   * @return None.
   */

  virtual double setDt(double dtIn);

  /**
   * @brief Destructor for the pidbase class.
   * @param None.
   * @return None.
   */

  virtual ~pidbase() {}
};
