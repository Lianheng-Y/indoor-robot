#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace
{
double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

class Planner final : public rclcpp::Node
{
public:
  Planner() : Node("planner")
  {
    linear_gain_ = declare_parameter("linear_gain", 0.8);
    angular_gain_ = declare_parameter("angular_gain", 2.0);
    max_linear_speed_ = declare_parameter("max_linear_speed", 0.45);
    max_angular_speed_ = declare_parameter("max_angular_speed", 1.2);
    position_tolerance_ = declare_parameter("position_tolerance", 0.08);
    control_rate_ = declare_parameter("control_rate", 20.0);
    goal_frame_ = declare_parameter("goal_frame", "odom");
    if (linear_gain_ <= 0.0 || angular_gain_ <= 0.0 || max_linear_speed_ <= 0.0 ||
      max_angular_speed_ <= 0.0 || position_tolerance_ <= 0.0 || control_rate_ <= 0.0)
    {
      throw std::invalid_argument("controller parameters must be positive");
    }

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        if (!message->header.frame_id.empty() && message->header.frame_id != goal_frame_) {
          RCLCPP_WARN(
            get_logger(), "Ignoring goal in '%s'; expected '%s'",
            message->header.frame_id.c_str(), goal_frame_.c_str());
          return;
        }
        goal_ = *message;
        goal_active_ = true;
        publish_goal_state(false);
        RCLCPP_INFO(
          get_logger(), "New goal: (%.2f, %.2f)",
          goal_.pose.position.x, goal_.pose.position.y);
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        odometry_ = *message;
        odometry_received_ = true;
      });
    cancel_sub_ = create_subscription<std_msgs::msg::Bool>(
      "navigation_cancel", rclcpp::QoS(10),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        if (message->data) {
          goal_active_ = false;
          publish_goal_state(false);
        }
      });
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
    goal_reached_pub_ = create_publisher<std_msgs::msg::Bool>(
      "goal_reached", rclcpp::QoS(1).transient_local());

    const auto period = std::chrono::duration<double>(1.0 / control_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this]() {control();});
    publish_goal_state(false);
    RCLCPP_INFO(get_logger(), "Point-to-point controller ready");
  }

private:
  void publish_goal_state(bool reached)
  {
    std_msgs::msg::Bool message;
    message.data = reached;
    goal_reached_pub_->publish(message);
  }

  void control()
  {
    geometry_msgs::msg::Twist command;
    if (!goal_active_ || !odometry_received_) {
      command_pub_->publish(command);
      return;
    }

    const double dx = goal_.pose.position.x - odometry_.pose.pose.position.x;
    const double dy = goal_.pose.position.y - odometry_.pose.pose.position.y;
    const double distance = std::hypot(dx, dy);
    if (distance <= position_tolerance_) {
      goal_active_ = false;
      command_pub_->publish(command);
      publish_goal_state(true);
      RCLCPP_INFO(get_logger(), "Goal reached");
      return;
    }

    const double yaw = yaw_from_quaternion(odometry_.pose.pose.orientation);
    const double heading_error = std::remainder(std::atan2(dy, dx) - yaw, 2.0 * M_PI);
    command.angular.z = std::clamp(
      angular_gain_ * heading_error, -max_angular_speed_, max_angular_speed_);

    const double alignment = std::max(0.0, std::cos(heading_error));
    command.linear.x = std::min(linear_gain_ * distance, max_linear_speed_) * alignment;
    command_pub_->publish(command);
  }

  double linear_gain_;
  double angular_gain_;
  double max_linear_speed_;
  double max_angular_speed_;
  double position_tolerance_;
  double control_rate_;
  std::string goal_frame_;
  bool goal_active_{false};
  bool odometry_received_{false};
  geometry_msgs::msg::PoseStamped goal_;
  nav_msgs::msg::Odometry odometry_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Planner>());
  rclcpp::shutdown();
  return 0;
}
