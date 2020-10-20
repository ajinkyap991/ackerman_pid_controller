/**
 * @file main.cpp
 * @author Ajinkya Parwekar
 * @author Karan Sutradhar
 * @brief The main.cpp file for Ackerman PID controller program.
 * It contains object creation and computation of control value.
 * @Copyright "Copyright 2020" <Ajinkya Parwekar>
 * @Copyright "Copyright 2020" <Karan Sutradhar>
 */

#include "pidcontroller.hpp"

/**
 * @brief main function of the program.
 * @param None.
 * @return 0.
 */

int main() {
    // Initialising Constructor (creating an object of the class)
    // stub implementation
    pidController controller(1, 0, 0, 0, 0, 0, 0, 0);

    // Calling all unused functions: stub implementation
    std::cout << controller.computeControlAction(2, 0) << std::endl;
    controller.changeInTime(1.0);
    controller.setSetPoints(1.0, 1.0);
    return 0;
}
