#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

class VelocitySafety final : public rclcpp::Node
{
public:
  VelocitySafety() : Node("velocity_safety")
  {
    max_linear_speed_ = declare_parameter("max_linear_speed", 0.6);
    max_angular_speed_ = declare_parameter("max_angular_speed", 1.5);
    command_timeout_ = declare_parameter("command_timeout", 0.5);
    publish_rate_ = declare_parameter("publish_rate", 50.0);
    if (max_linear_speed_ <= 0.0 || max_angular_speed_ <= 0.0 ||
      command_timeout_ <= 0.0 || publish_rate_ <= 0.0)
    {
      throw std::invalid_argument("velocity safety parameters must be positive");
    }
    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10), [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        last_command_ = std::chrono::steady_clock::now();
        command_received_ = true;
        if (!finite(*message)) {
          command_ = geometry_msgs::msg::Twist();
          RCLCPP_ERROR(get_logger(), "Rejected non-finite velocity command; stopping");
          return;
        }
        command_ = *message;
      });
    safe_command_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_safe", 10);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_));
    timer_ = create_wall_timer(period, [this]() { publish_safe_command(); });
  }

private:
  static bool finite(const geometry_msgs::msg::Twist & twist)
  {
    return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
           std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
           std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
  }

  void publish_safe_command()
  {
    geometry_msgs::msg::Twist safe;
    const double command_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_command_).count();
    if (command_received_ && command_age <= command_timeout_) {
      safe.linear.x = std::clamp(command_.linear.x, -max_linear_speed_, max_linear_speed_);
      safe.angular.z = std::clamp(command_.angular.z, -max_angular_speed_, max_angular_speed_);
    }
    safe_command_pub_->publish(safe);
  }

  double max_linear_speed_;
  double max_angular_speed_;
  double command_timeout_;
  double publish_rate_;
  bool command_received_{false};
  std::chrono::steady_clock::time_point last_command_;
  geometry_msgs::msg::Twist command_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocitySafety>());
  rclcpp::shutdown();
  return 0;
}
