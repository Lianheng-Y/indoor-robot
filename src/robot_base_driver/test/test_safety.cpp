#include <gtest/gtest.h>
#include <chrono>
#include <limits>
#include "robot_base_driver/safety_math.hpp"
using namespace std::chrono_literals;

TEST(Safety, LimitsLinearAndAngularSpeed) {
  geometry_msgs::msg::Twist input; input.linear.x = 2.0; input.angular.z = -4.0;
  const auto output = robot_base_driver::limit_twist(input, 0.6, 1.5);
  EXPECT_DOUBLE_EQ(output.linear.x, 0.6); EXPECT_DOUBLE_EQ(output.angular.z, -1.5);
}
TEST(Safety, RejectsNonFiniteVelocity) {
  geometry_msgs::msg::Twist input; input.linear.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(robot_base_driver::finite_twist(input));
  EXPECT_DOUBLE_EQ(robot_base_driver::limit_twist(input, 0.6, 1.5).linear.x, 0.0);
}
TEST(Safety, StopsAfterHalfSecondTimeout) {
  const auto last = std::chrono::steady_clock::now();
  EXPECT_TRUE(robot_base_driver::command_fresh(last + 499ms, last, 0.5));
  EXPECT_FALSE(robot_base_driver::command_fresh(last + 501ms, last, 0.5));
}
TEST(Safety, IntegratesStraightAndRotationMotion) {
  double x = 0.0, y = 0.0, yaw = 0.0;
  robot_base_driver::integrate_unicycle(x, y, yaw, 1.0, 0.0, 1.0);
  EXPECT_NEAR(x, 1.0, 1e-9); EXPECT_NEAR(y, 0.0, 1e-9);
  robot_base_driver::integrate_unicycle(x, y, yaw, 0.0, 1.0, 1.0);
  EXPECT_NEAR(yaw, 1.0, 1e-9);
}
