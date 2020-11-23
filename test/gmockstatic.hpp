#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>
#include "pidbase.hpp"
/*
 * @brief gmockStatic class to call the functions from the pidbase class.
 */
class gmockStatic : public ::testing::Test {
 public:
  /*
   * @brief Function to test the setKp function.
   * @param unique pointer to the newPID class
   * @return None.
   *
   */
  void set_Kp(std::unique_ptr<pidbase> npid) {
    npid->setKp(3.0);
  }
  /*
   * @brief Function to test the setKd function.
   * @param unique pointer to the newPID class
   * @return None.
   *
   */
  void set_Kd(std::unique_ptr<pidbase> npid) {
    npid->setKd(2.0);
  }
  /*
   * @brief Function to test the setKi function.
   * @param unique pointer to the newPID class
   * @return None.
   *
   */
  void set_Ki(std::unique_ptr<pidbase> npid) {
    npid->setKi(1.0);
  }
};
