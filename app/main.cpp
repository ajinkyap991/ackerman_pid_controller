/**
 * @file main.cpp
 * @author Karan Sutradhar: Driver
 * @author Ajinkya Parwekar: Navigator
 * @brief The main.cpp file for Ackerman PID controller program.
 * It contains object creation and computation of control value.
 * @Copyright "Copyright 2020" <Ajinkya Parwekar>
 * @Copyright "Copyright 2020" <Karan Sutradhar>
 */

#include <iostream>
#include <memory>
#include "pidbase.hpp"
#include "pidcontroller.hpp"

/**
 * @brief main function of the program.
 * @param None.
 * @return 0.
 */

int main() {
  double carLen = 1;

  // Heading and velocity to be achieved.
  // Heading should be between -3.14 and +3.14
  double headingSp = 2, velSp = 1;
  double number_of_iterations = 25;

  // Initializing ackerman controller by creating its instance
  // using constructor with 0 parameters
  pidController obj;

  std::unique_ptr<pidbase> npid = std::make_unique<pidController>();

  npid->setKp(-12);
  npid->setKd(0);
  npid->setKi(0);

  // initialising new parameters
  double heading = 0, posX = 0, posY = 0;
  double *headingptr = &heading;
  double *posXptr = &posX;
  double *posYptr = &posY;

  // setting setpoints
  obj.setSetPoints(headingSp, velSp);
  std::cout << "The heading setpoint is : " << headingSp << std::endl;
  double lSpeed = velSp / 2, rSpeed = velSp / 2, steer = 0;
  double *lSpeedptr = &lSpeed;
  double *rSpeedptr = &rSpeed;
  double *steerptr = &steer;

  // Getting final output values
  for (int z = 1; z < number_of_iterations; z++) {
    std::cout << "Simulation Iteration Number: " << z << std::endl;
    // computing vehicle position
    obj.compute(steerptr, lSpeedptr, rSpeedptr, posXptr, posYptr, headingptr,
                   carLen);
    std::cout << "Current Heading: " << *headingptr << std::endl
              << "Current X Position: " << *posXptr << std::endl
              << "Current Y Position: " << *posYptr << std::endl;
    // computing steering angle of the vehicle
    obj.computePIDParameters(steerptr, headingptr, rSpeedptr, lSpeedptr);
    std::cout << "Steering angle PID output in degrees: " << *steerptr << std::endl
              << std::endl;
  }
  return 0;
}
