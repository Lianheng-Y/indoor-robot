#include <gtest/gtest.h>
#include "robot_navigation/controller_math.hpp"

TEST(ControllerMath, NormalizesAngles) {
  const double positive = robot_navigation::normalize_angle(3.0 * M_PI);
  const double negative = robot_navigation::normalize_angle(-3.0 * M_PI);
  EXPECT_NEAR(std::abs(positive), M_PI, 1e-9);
  EXPECT_NEAR(std::abs(negative), M_PI, 1e-9);
  EXPECT_LE(positive, M_PI); EXPECT_GE(positive, -M_PI);
  EXPECT_LE(negative, M_PI); EXPECT_GE(negative, -M_PI);
}
TEST(ControllerMath, AppliesAccelerationAndDecelerationLimits) {
  EXPECT_DOUBLE_EQ(robot_navigation::limit_rate(1.0, 0.0, 2.0, 3.0, 0.25), 0.5);
  EXPECT_DOUBLE_EQ(robot_navigation::limit_rate(0.0, 1.0, 2.0, 3.0, 0.25), 0.25);
}
TEST(ControllerMath, RequiresPositionAndHeadingForArrival) {
  EXPECT_FALSE(robot_navigation::within_goal(0.05, 0.2, 0.08, 0.05));
  EXPECT_TRUE(robot_navigation::within_goal(0.05, 2.0 * M_PI + 0.02, 0.08, 0.05));
}
TEST(ControllerMath, StopsAtPositionAndHeadingGoal) {
  EXPECT_TRUE(robot_navigation::should_stop_at_goal(0.01, 0.01, 0.08, 0.05));
  EXPECT_FALSE(robot_navigation::should_stop_at_goal(0.01, 0.2, 0.08, 0.05));
}
