#pragma once
#include <algorithm>
#include <cmath>

namespace robot_navigation {
inline double normalize_angle(double angle) { return std::remainder(angle, 2.0 * M_PI); }
inline double limit_rate(double target, double current, double accel, double decel, double dt) {
  const double limit = (std::abs(target) > std::abs(current) ? accel : decel) * dt;
  return current + std::clamp(target - current, -limit, limit);
}
inline bool within_goal(double distance, double yaw_error, double position_tolerance, double yaw_tolerance) {
  return distance <= position_tolerance && std::abs(normalize_angle(yaw_error)) <= yaw_tolerance;
}
inline bool should_stop_at_goal(double distance, double yaw_error, double position_tolerance, double yaw_tolerance) {
  return within_goal(distance, yaw_error, position_tolerance, yaw_tolerance);
}
}
