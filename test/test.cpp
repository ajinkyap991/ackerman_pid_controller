/**
 * @file test.cpp
 * @author Ajinkya Parwekar: Driver
 * @author Karan Sutradhar: Navigator
 * @brief Test code functions for PID controller using gtest
 * @Copyright "Copyright 2020" <Ajinkya Parwekar>
 * @Copyright "Copyright 2020" <Karan Sutradhar>
 */

#include <gtest/gtest.h>
#include "../include/pidcontroller.hpp"

/**
 * @brief This test checks if the control law works as expected
 * @param controllerTest is the name of the group of tests
 * @param controlFunctionTest1 is the specific name to check the control function
 */

TEST(controllerTest, controlFunctionTest1) {
    pidController controller(1, 0, 0);
    EXPECT_EQ(2, pidController.compute(2));
}


/**
 * @brief This test checks if the controller has properly stored its parameters
 * @param controllerTest is the name of the group of tests
 * @param paramGetTest is the specific name to check the getControlParam function
 */

TEST(controllerTest, paramGetTest) {
    pidController velController(2.7, 4.5, 6.3);
    EXPECT_EQ(2.7, pidController.getKp());
    EXPECT_EQ(4.5, pidController.getKd());
    EXPECT_EQ(6.3, pidController.getKi());
}

/**
* @brief This test checks if the controller can change the inherent parameters
* @param controllerTest is the name of the group of tests
* @param paramSetTest is the specific name to check the getControlParam function
*/
TEST(controllerTest, paramSetTest) {
    Controller velController(2.7, 4.5, 6.3);

    pidController.setKp(5.6);
    pidController.setKd(2.3);
    pidController.setKi(3.6);

    EXPECT_EQ(5.6, velController.getKp());
    EXPECT_EQ(2.3, velController.getKd());
    EXPECT_EQ(3.6, velController.getKi());
}
