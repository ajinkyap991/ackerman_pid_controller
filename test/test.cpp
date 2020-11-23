/**
 * @file test.cpp
 * @author Karan Sutradhar: Driver
 * @author Ajinkya Parwekar: Navigator
 * @brief Test code functions for PID obj using gtest and gmock
 * @Copyright "Copyright 2020" <Ajinkya Parwekar>
 * @Copyright "Copyright 2020" <Karan Sutradhar>
 */

#include <iostream>
#include "gmock/gmock.h"
#include "mocktest.hpp"
#include "pidbase.hpp"
#include "pidcontroller.hpp"
#include "gmockstatic.hpp"
#include "gtest/gtest.h"

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;

/*
 * @brief Test case for the setKp function using mock.
 * @param None.
 * @return None.
 */
TEST(gmockStatic, testSetKpFunc) {
  pidController obj;
  std::unique_ptr<mockTest> mpid(new mockTest);
  std::unique_ptr<gmockStatic> gmock;
  EXPECT_CALL(*mpid, setKp(3.0)).Times(1).WillOnce(Return(3.0));
  EXPECT_EQ(3, obj.getKp());
  gmock->set_Kp(std::move(mpid));
  EXPECT_EQ(2, obj.setKp(2));
}
/*
 * @brief Test case for the setKd function using mock.
 * @param None.
 * @return None.
 */

TEST(gmockStatic, testSetKdFunc) {
  pidController obj;
  std::unique_ptr<mockTest> mpid(new mockTest);
  std::unique_ptr<gmockStatic> gmock;
  EXPECT_CALL(*mpid, setKd(2.0)).Times(1).WillOnce(Return(2.0));
  EXPECT_EQ(2, obj.getKd());
  gmock->set_Kd(std::move(mpid));
  EXPECT_EQ(3.4, obj.setKd(3.4));
}
/*
 * @brief Test case for the setKi function using mock.
 * @param None.
 * @return None.
 */
TEST(gmockStatic, testSetKiFunc) {
  pidController obj;
  std::unique_ptr<mockTest> mpid(new mockTest);
  std::unique_ptr<gmockStatic> gmock;
  EXPECT_CALL(*mpid, setKi(1.0)).Times(1).WillOnce(Return(1.0));
  EXPECT_EQ(1, obj.getKi());
  gmock->set_Ki(std::move(mpid));
  EXPECT_EQ(1.2, obj.setKi(1.2));
}

/*
 * @brief Unit test of the PID class to test the get functions.
 * @param None.
 * @return None.
 */

TEST(PIDTest, testGainsFunc) {
  mockTest gmock;
  pidController obj(3, 2, 4, 0.1, true);
  EXPECT_EQ(3, obj.getKp());
  EXPECT_EQ(2, obj.getKd());
  EXPECT_EQ(4, obj.getKi());
}
