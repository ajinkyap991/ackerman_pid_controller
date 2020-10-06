/**
 * @file pid.hpp
 * @author Ajinkya Parwekar
 * @author Karan Sutradhar
 * @brief Definition of a PID Controller for Ackerman Steering Mechanism
 * It uses controller gain values and returns output value based on setpoint and feedback values.
 *
 * @Copyright "Copyright 2020" <Ajinkya Parwekar>
 * @Copyright "Copyright 2020" <Karan Sutradhar>
 */

#pragma once

#include <iostream>
#include <string>

class pidController {
 private:
  double kp, ki, kd, error, previousError, integralError, dt;

 public:
  /**
   * @brief Base Constructor for the PID Controller class.
   * @param None.
   * @return None.
   */
  pidController();

  /**
   * @brief Constructor for PID controller class with three gain parameters.
   * @param kp Proportional Gain of PID controller.
   * @param kd Differential Gain of PID controller.
   * @param ki Integral Gain of PID controller.
   * @return None.
   */
  pidController(double kp, double kd, double ki);

  /**
   * @brief Function to compute the output of the PID controller.
   * The output of the function is based on the three error values corresponding to three controller gains.
   * @param feedback value i.e. the actual value of the parameter being corrected.
   * @return controlAction Output calculated by the PID controller using the gain values.
   */

  double compute(double feedback);

  /**
   * @brief Function to set the proportional gain variable of the PID controller
   * @param kp (Proportional gain)
   * @return None.
   */

  void setKp(double);

  /**
   * @brief Function to set the differential gain variable of the PID controller
   * @param kd (Differential gain)
   * @return None.
   */

  void setKd(double);

  /**
   * @brief Function to set the integral gain variable of the PID controller
   * @param ki (Integral gain)
   * @return None.
   */

  void setKi(double);

  /**
   * @brief Function to set the time variable of the PID controller
   * @param dt (time variable)
   * @return None.
   */

  void setDt(double);

  /**
   * @brief Function to get the proportional gain variable of the PID controller
   * @param None
   * @return kp (Proportional gain)
   */

  double getkp();

  /**
   * @brief Function to get the differential gain variable of the PID controller
   * @param None
   * @return kd (Differential gain)
   */

  double getKd();

  /**
   * @brief Function to get the integral gain variable of the PID controller
   * @param None
   * @return ki (Integral gain)
   */

  double getKi();

  /**
   * @brief Function to get the time variable value of the PID controller
   * @param None
   * @return dt (time variable)
   */

  double getDt();

  /**
   * @brief Function to return the previous error value (for test suite)
   * @param None.
   * @return Previous error value(previousError).
   */

  double getPreviousError();

  /**
   * @brief Function to return the integral error value (for test suite)
   * @param None.
   * @return Integral error value(integralError).
   */

  double getIntegralError();

  /**
   * Destructor for PID controller
   * @param None.
   * @return None.
   */

  // ~pidController();
};
