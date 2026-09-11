#pragma once
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>

namespace robot_base_driver {
inline bool finite_twist(const geometry_msgs::msg::Twist & t) {
  return std::isfinite(t.linear.x) && std::isfinite(t.linear.y) && std::isfinite(t.linear.z) &&
         std::isfinite(t.angular.x) && std::isfinite(t.angular.y) && std::isfinite(t.angular.z);
}
inline geometry_msgs::msg::Twist limit_twist(const geometry_msgs::msg::Twist & t, double max_linear, double max_angular) {
  geometry_msgs::msg::Twist out;
  if (!finite_twist(t)) return out;
  out.linear.x = std::clamp(t.linear.x, -max_linear, max_linear);
  out.angular.z = std::clamp(t.angular.z, -max_angular, max_angular);
  return out;
}
inline bool command_fresh(const std::chrono::steady_clock::time_point & now,
  const std::chrono::steady_clock::time_point & last, double timeout) {
  return std::chrono::duration<double>(now - last).count() <= timeout;
}
inline void integrate_unicycle(double & x, double & y, double & yaw, double linear, double angular, double dt) {
  const double midpoint = yaw + angular * dt * 0.5;
  x += linear * std::cos(midpoint) * dt;
  y += linear * std::sin(midpoint) * dt;
  yaw = std::remainder(yaw + angular * dt, 2.0 * M_PI);
}
}
