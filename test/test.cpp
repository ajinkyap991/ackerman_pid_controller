/**
 * @file test.cpp
 * @author Ajinkya Parwekar: Driver
 * @author Karan Sutradhar: Navigator
 * @brief Test code functions for PID controller using gtest
 * @Copyright "Copyright 2020" <Ajinkya Parwekar>
 * @Copyright "Copyright 2020" <Karan Sutradhar>
 */

#include <gtest/gtest.h>
#include "pidcontroller.hpp"

/**
 * @brief This test checks if the control law works as expected
 * @param controllerTest is the name of the group of tests
 * @param controlFunctionTest1 is the specific name to check the control function
 */

TEST(controllerTest, controlFunctionTest1) {
    pidController controller(1, 0, 0, 0);
    EXPECT_EQ(2, controller.computeControlAction(2, 0));
}


/**
 * @brief This test checks if the controller has properly stored its parameters
 * @param controllerTest is the name of the group of tests
 * @param paramGetTest is the specific name to check the getControlParam function
 */

TEST(controllerTest, paramGetTest) {
    pidController controller(2.7, 4.5, 6.3, 1.0);
    EXPECT_EQ(2.7, controller.getKp());
    EXPECT_EQ(4.5, controller.getKd());
    EXPECT_EQ(6.3, controller.getKi());
}

/**
* @brief This test checks if the controller can change the inherent parameters
* @param controllerTest is the name of the group of tests
* @param paramSetTest is the specific name to check the getControlParam function
*/
TEST(controllerTest, paramSetTest) {
    pidController controller(2.7, 4.5, 6.3, 1.0);

    controller.setKp(5.6);
    controller.setKd(2.3);
    controller.setKi(3.6);

    EXPECT_EQ(5.6, controller.getKp());
    EXPECT_EQ(2.3, controller.getKd());
    EXPECT_EQ(3.6, controller.getKi());
}

/**
 * @brief This test checks if the control law works as expected
 * @param controllerTest is the name of the group of tests
 * @param getVeriableTest3 is the specific name to check the get veriable function
 */

TEST(controllerTest, getVeriableTest3) {
    pidController controller(1, 0, 0, 0);
    EXPECT_EQ(0, controller.computeArcRadius());
}

/**
 * @brief This test checks if the control law works as expected
 * @param controllerTest is the name of the group of tests
 * @param getVeriableTest4 is the specific name to check the get veriable function
 */

TEST(controllerTest, getVeriableTest4) {
    pidController controller(1, 0, 0, 0);
    EXPECT_EQ(0, controller.computeWheelSpeed());
}

/**
 * @brief This test checks if the control law works as expected
 * @param controllerTest is the name of the group of tests
 * @param getVeriableTest5 is the specific name to check the get veriable function
 */

TEST(controllerTest, getVeriableTest5) {
    pidController controller(1, 0, 0, 0);
    EXPECT_EQ(0, controller.computeSteeringAngle(0, 0, 0, 0));
}
