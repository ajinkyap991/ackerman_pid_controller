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

int main() {
    // stub implementation
    pidController controller(1, 0, 0, 0);
    std::cout << controller.computeControlAction(2, 0) << std::endl;
    return 0;
}
