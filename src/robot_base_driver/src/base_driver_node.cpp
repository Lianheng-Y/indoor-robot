#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include "robot_base_driver/safety_math.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#if __has_include(<tf2/LinearMath/Quaternion.hpp>)
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#else
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#endif

using namespace std::chrono_literals;

namespace
{
}  // namespace

class BaseDriver final : public rclcpp::Node
{
public:
  BaseDriver() : Node("base_driver")
  {
    max_linear_speed_ = declare_parameter("max_linear_speed", 0.6);
    max_angular_speed_ = declare_parameter("max_angular_speed", 1.5);
    command_timeout_ = declare_parameter("command_timeout", 0.5);
    publish_rate_ = declare_parameter("publish_rate", 50.0);
    odom_frame_ = declare_parameter("odom_frame", "odom");
    base_frame_ = declare_parameter("base_frame", "base_footprint");

    if (max_linear_speed_ <= 0.0 || max_angular_speed_ <= 0.0 ||
      command_timeout_ <= 0.0 || publish_rate_ <= 0.0)
    {
      throw std::invalid_argument("speed, timeout and publish rate parameters must be positive");
    }

    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        last_command_time_ = now();
        command_received_ = true;
        if (!robot_base_driver::finite_twist(*message)) {
          command_ = geometry_msgs::msg::Twist();
          RCLCPP_ERROR(get_logger(), "Rejected non-finite velocity command; stopping");
          return;
        }
        command_ = *message;
      });
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
    safe_command_pub_ =
      create_publisher<geometry_msgs::msg::Twist>("cmd_vel_safe", rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    last_tick_time_ = now();
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this]() {tick();});
    RCLCPP_INFO(
      get_logger(), "Base simulation ready (%.2f m/s, %.2f rad/s, %.2f s timeout)",
      max_linear_speed_, max_angular_speed_, command_timeout_);
  }

private:
  void tick()
  {
    const auto stamp = now();
    const double dt = std::clamp((stamp - last_tick_time_).seconds(), 0.0, 0.1);
    last_tick_time_ = stamp;

    geometry_msgs::msg::Twist safe_command;
    if (command_received_ && (stamp - last_command_time_).seconds() <= command_timeout_) {
      safe_command = robot_base_driver::limit_twist(command_, max_linear_speed_, max_angular_speed_);
    }
    safe_command_pub_->publish(safe_command);

    robot_base_driver::integrate_unicycle(
      x_, y_, yaw_, safe_command.linear.x, safe_command.angular.z, dt);

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw_);

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = x_;
    odometry.pose.pose.position.y = y_;
    odometry.pose.pose.orientation.x = orientation.x();
    odometry.pose.pose.orientation.y = orientation.y();
    odometry.pose.pose.orientation.z = orientation.z();
    odometry.pose.pose.orientation.w = orientation.w();
    odometry.pose.covariance[0] = 0.01;
    odometry.pose.covariance[7] = 0.01;
    odometry.pose.covariance[35] = 0.02;
    odometry.twist.twist = safe_command;
    odometry.twist.covariance = odometry.pose.covariance;
    odom_pub_->publish(odometry);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odometry.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.rotation = odometry.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  double max_linear_speed_;
  double max_angular_speed_;
  double command_timeout_;
  double publish_rate_;
  std::string odom_frame_;
  std::string base_frame_;
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  bool command_received_{false};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Twist command_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_command_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseDriver>());
  rclcpp::shutdown();
  return 0;
}
