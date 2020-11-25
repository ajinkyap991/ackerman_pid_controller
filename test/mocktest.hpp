/**
 * @file mocktest.hpp
 * @author Karan Sutradhar: Driver
 * @author Ajinkya Parwekar: Navigator
 * @brief Definition of a gmock testing class for Ackerman Steering Mechanism
 * @Copyright "Copyright 2020" <Ajinkya Parwekar>
 * @Copyright "Copyright 2020" <Karan Sutradhar>
 */

#pragma once

#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "pidbase.hpp"
/*
 * @brief  Class to initialize the mock method to
 * test using the mock method.
 */
class mockTest: public pidbase {
 public:
  /*
   * @brief Initialize the mock test for the setKp function
   * @param The value of Proportional Gain(Kp)
   * @return None.
   */
  MOCK_METHOD1(setKp, double(double));
  /*
   * @brief Initialize the mock test for the setKi function
   * @param The value of integral Gain(Ki)
   * @return None.
   */
  MOCK_METHOD1(setKd, double(double));
  /*
   * @brief Initialize the mock test for the setKd function
   * @param The value of Differential Gain(Kd)
   * @return None.
   */
  MOCK_METHOD1(setKi, double(double));
  /*
   * @brief Initialize the mock test for the setKd function
   * @param The value of Differential Gain(Kd)
   * @return None.
   */
  // MOCK_METHOD1(setDt, double(double));
  /*
   * @brief Initialize the mock test for the compute function
   * @param The value of the expected state.
   * @param The value of the current state
   * @return None.
   */
  // MOCK_METHOD2(compute, double(double, double));
};
