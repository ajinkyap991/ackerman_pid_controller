/**
 * @file pidbase.cpp
 * @author Karan Sutradhar: Driver
 * @author Ajinkya Parwekar: Navigator
 * @brief The pidbase.cpp file for Ackerman PID controller program.
 * It contains Ackerman PID controller base class methods definitions.
 * @Copyright "Copyright 2020" <Ajinkya Parwekar>
 * @Copyright "Copyright 2020" <Karan Sutradhar>
 */


#include "pidbase.hpp"

  /**
   * @brief Base Constructor for the PID Controller class.
   * @param None.
   * @return None.
   */

  pidbase::pidbase() {
    kp = 1;
    kd = 0;
    ki = 0;
    dt = 0.1;
    kb = 0;
    errorSum = 0;
    previousError = 0;
    integralError = 0;
    prevTime = std::chrono::system_clock::now();
    dtMode = false;
    setpoint = 0;
    firstRunFlag = true;
    baseline = 1;
    carLen = 1;
    arcRadius = 0;
    rWheelVel = 0;
    lWheelVel = 0;
    steeringAngle = 0;
    setpointSpeed = 2;
    setpointHeading = 0;
    dtSim = 0.05;
    posX = 0;
    posY = 0;
    updatedHeading = 0;
    leftWheelSpeed = 0;
    rightWheelSpeed = 0;
    antiWindUp = 1;
    CnI = 0.0;
    CnD = 0.0;
    tSec = t / 1e3;
    x = 0.0;
    a = 0.0;
    b = 0.0;
    min = 0.0;
    max = 0.0;
    n = 0.0, t = 0.0, kf = 0.0, CnP = 0.0, outMin = 0.0, outMax = 0.0;
  }

  /**
   * @brief Function to set the proportional gain variable of the PID controller
   * @param kp (Proportional gain)
   * @return None.
   */

  double pidbase::setKp(double kpIn) {
    kp = kpIn;
    return kp;
  }

  /**
   * @brief Function to set the differential gain variable of the PID controller
   * @param kd (Differential gain)
   * @return None.
   */

  double pidbase::setKd(double kdIn) {
    kd = kdIn;
    return kd;
  }

  /**
   * @brief Function to set the integral gain variable of the PID controller
   * @param ki (Integral gain)
   * @return None.
   */

  double pidbase::setKi(double kiIn) {
    ki = kiIn;
    return ki;
  }

  /**
   * @brief Function to set the time variable of the PID controller
   * @param dt (time variable)
   * @return None.
   */

  double pidbase::setDt(double dtIn) {
    dt = dtIn;
    return dt;
  }

  /**
   * Destructor for PID controller
   * @param None.
   * @return None.
   */

  // ~pidController() {
  // }
