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
  double kp, kd, ki, kb, error, previousError, integralError, dt;

 public:
  /**
   * @brief Base Constructor for the PID Controller class.
   * @param None.
   * @return None.
   */
  pidController();

  /**
   * @brief Constructor for PID controller class with three gain parameters and time variable.
   * @param kpValue Proportional Gain of PID controller.
   * @param kdValue Differential Gain of PID controller.
   * @param kiValue Integral Gain of PID controller.
   * @param dtValue time variable for controller.
   * @return None.
   */

  pidController(double kpValue, double kdValue, double kiValue, double dtValue);

  /**
   * @brief Constructor for PID controller class with all private attributes.
   * @param kpValue Proportional Gain of PID controller.
   * @param kdValue Differential Gain of PID controller.
   * @param kiValue Integral Gain of PID controller.
   * @param kbValue Variable for back calculation to eliminate windup.
   * @param dtValue time variable for controller.
   * @param errorvalue error value for proportional gain.
   * @param previousErrorValue error value for differential gain.
   * @param integralErrorValue error value for integral gain.
   * @return None.
   */

  pidController(double kpValue, double kdValue, double kiValue, double kbValue,
    double dtValue, double errorValue, double previousErrorValue, double integralErrorValue);

  /**
   * @brief Function to compute the output of the PID controller.
   * The output of the function is based on the three error values corresponding to three controller gains.
   * @param feedback value i.e. the actual value of the parameter being corrected.
   * @return controlAction Output calculated by the PID controller using the gain values.
   */

  double computeControlAction(double feedback, double setpoint);

  /**
   * @brief Function to change the time value.
   * @param newDt (new time value).
   * @return None.
   */

  void changeInTime(double newDt);


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
   * @brief Function to set the back calculation variable of the PID controller
   * @param kb (for back calculation)
   * @return None.
   */

  void setKb(double);

  /**
   * @brief Function to set the time variable of the PID controller
   * @param dt (time variable)
   * @return None.
   */

  void setDt(double);

  /**
   * @brief Function to set the error value of the PID controller
   * @param error (for proportional error))
   * @return None.
   */

  void setError(double);

  /**
   * @brief Function to set the previous error of the PID controller
   * @param previousError (for differential error)
   * @return None.
   */

  void setPreviousError(double);

  /**
   * @brief Function to set the integral error of the PID controller
   * @param integralError (for integral error)
   * @return None.
   */

  void setIntegralError(double);

  /**
   * @brief Function to get the proportional gain variable of the PID controller
   * @param None
   * @return kp (Proportional gain)
   */

  double getKp();

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
   * @brief Function to get the back calculation variable of the PID controller
   * @param None
   * @return kb (for back calculation)
   */

  double getKb();

  /**
   * @brief Function to get the time variable value of the PID controller
   * @param None
   * @return dt (time variable)
   */

  double getDt();

  /**
   * @brief Function to get the proportional error value of the PID controller
   * @param None
   * @return error (for proportional error)
   */

  double getError();

  /**
   * @brief Function to get the previous error value of the PID controller
   * @param None
   * @return previousError (for differential error)
   */

  double getPreviousError();

  /**
   * @brief Function to get the integral error value of the PID controller
   * @param None
   * @return integralError (for integral error)
   */

  double getIntegralError();

  /**
   * @brief Function to compute the arc radius of the wheel from rotation point.
   * @param None.
   * @return arc radius.
   */

  double computeArcRadius();

  /**
   * @brief Function to compute the wheel velocities of both wheels.
   * @param None.
   * @return wheel velocity.
   */

  double computeWheelSpeed();

  /**
   * @brief Function to compute the steering angle for the wheel from rotation point.
   * @param steering angle, velocity of right wheel, velocity of left wheel and heading output.
   * @return steering angle.
   */

  double computeSteeringAngle(double steeringAngle, double rightWheelSpeed,
    double leftWheelSpeed, double compassHeadingOutput);

  /**
   * @brief Function to generate the throttle output value.
   * @param None.
   * @return throttle output value.
   */

  double throttleOutput();

  /**
   * @brief Function to set the setpoint values.
   * @param setpoint velocity and setpoint heading.
   * @return None.
   */

  void setSetPoints(double setpointSpeed, double setpointHeading);

  /**
   * Destructor for PID controller
   * @param None.
   * @return None.
   */

  // ~pidController();
};
