#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_interfaces/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include "robot_navigation/controller_math.hpp"

namespace
{
bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q.x) && std::isfinite(q.y) &&
         std::isfinite(q.z) && std::isfinite(q.w);
}

bool finite_twist(const geometry_msgs::msg::Twist & twist)
{
  const auto & linear = twist.linear;
  const auto & angular = twist.angular;
  return std::isfinite(linear.x) && std::isfinite(linear.y) && std::isfinite(linear.z) &&
         std::isfinite(angular.x) && std::isfinite(angular.y) && std::isfinite(angular.z);
}

bool valid_odometry_pose(const geometry_msgs::msg::Pose & pose)
{
  if (!finite_pose(pose)) {
    return false;
  }
  const auto & q = pose.orientation;
  const double norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return norm_squared > 1e-12;
}

bool valid_orientation(const geometry_msgs::msg::Quaternion & q)
{
  return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 1e-12;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(norm) || norm <= 1e-12) {
    return 0.0;
  }
  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;
  return std::atan2(
    2.0 * (w * z + x * y),
    1.0 - 2.0 * (y * y + z * z));
}
}  // namespace

class PointController final : public rclcpp::Node
{
public:
  using NavigateToPose = robot_interfaces::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  PointController() : Node("point_controller")
  {
    linear_gain_ = declare_parameter("linear_gain", 0.8);
    angular_gain_ = declare_parameter("angular_gain", 2.0);
    max_linear_speed_ = declare_parameter("max_linear_speed", 0.45);
    max_angular_speed_ = declare_parameter("max_angular_speed", 1.2);
    position_tolerance_ = declare_parameter("position_tolerance", 0.08);
    control_rate_ = declare_parameter("control_rate", 20.0);
    odom_timeout_ = declare_parameter("odom_timeout", 0.5);
    yaw_tolerance_ = declare_parameter("yaw_tolerance", 0.05);
    max_linear_accel_ = declare_parameter("max_linear_accel", 0.8);
    max_linear_decel_ = declare_parameter("max_linear_decel", 1.2);
    max_angular_accel_ = declare_parameter("max_angular_accel", 2.0);
    max_angular_decel_ = declare_parameter("max_angular_decel", 3.0);
    command_smoothing_alpha_ = declare_parameter("command_smoothing_alpha", 0.35);
    max_odom_jump_ = declare_parameter("max_odom_jump", 1.0);
    odom_filter_alpha_ = declare_parameter("odom_filter_alpha", 0.35);
    allow_reverse_ = declare_parameter("allow_reverse", false);
    goal_frame_ = declare_parameter("goal_frame", "odom");
    if (linear_gain_ <= 0.0 || angular_gain_ <= 0.0 || max_linear_speed_ <= 0.0 ||
      max_angular_speed_ <= 0.0 || position_tolerance_ <= 0.0 ||
      control_rate_ <= 0.0 || odom_timeout_ <= 0.0 || yaw_tolerance_ <= 0.0 ||
      max_linear_accel_ <= 0.0 || max_linear_decel_ <= 0.0 || max_angular_accel_ <= 0.0 ||
      max_angular_decel_ <= 0.0 || command_smoothing_alpha_ <= 0.0 ||
      command_smoothing_alpha_ > 1.0 || max_odom_jump_ <= 0.0 ||
      odom_filter_alpha_ <= 0.0 || odom_filter_alpha_ > 1.0)
    {
      throw std::invalid_argument("controller parameters must be positive");
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        if (!valid_odometry_pose(message->pose.pose) || !finite_twist(message->twist.twist)) {
          RCLCPP_ERROR(
            get_logger(), "Rejected odometry containing non-finite or invalid pose/velocity");
          return;
        }
        const double x = message->pose.pose.position.x;
        const double y = message->pose.pose.position.y;
        if (filtered_odom_received_ && std::hypot(x - filtered_x_, y - filtered_y_) > max_odom_jump_) {
          RCLCPP_WARN(get_logger(), "Rejected odometry position jump; retaining last estimate");
          return;
        }
        const double raw_yaw = yaw_from_quaternion(message->pose.pose.orientation);
        if (!filtered_odom_received_) {
          filtered_x_ = x; filtered_y_ = y; filtered_yaw_ = raw_yaw;
        } else {
          filtered_x_ = odom_filter_alpha_ * x + (1.0 - odom_filter_alpha_) * filtered_x_;
          filtered_y_ = odom_filter_alpha_ * y + (1.0 - odom_filter_alpha_) * filtered_y_;
          const double yaw_delta = std::remainder(raw_yaw - filtered_yaw_, 2.0 * M_PI);
          filtered_yaw_ = std::remainder(filtered_yaw_ + odom_filter_alpha_ * yaw_delta, 2.0 * M_PI);
        }
        odometry_ = *message;
        odometry_.pose.pose.position.x = filtered_x_;
        odometry_.pose.pose.position.y = filtered_y_;
        tf2::Quaternion filtered_orientation;
        filtered_orientation.setRPY(0.0, 0.0, filtered_yaw_);
        odometry_.pose.pose.orientation.x = filtered_orientation.x();
        odometry_.pose.pose.orientation.y = filtered_orientation.y();
        odometry_.pose.pose.orientation.z = filtered_orientation.z();
        odometry_.pose.pose.orientation.w = filtered_orientation.w();
        odometry_received_ = true;
        filtered_odom_received_ = true;
        last_odom_time_ = std::chrono::steady_clock::now();
      });
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
    action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      "navigate_to_pose",
      [this](
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const NavigateToPose::Goal> goal)
      {
        const auto & target = goal->target_pose;
        if (!finite_pose(target.pose) || !valid_orientation(target.pose.orientation)) {
          RCLCPP_WARN(get_logger(), "Rejected goal containing NaN or infinity");
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (!target.header.frame_id.empty() && target.header.frame_id != goal_frame_) {
          RCLCPP_WARN(
            get_logger(), "Rejected goal in '%s'; expected '%s'",
            target.header.frame_id.c_str(), goal_frame_.c_str());
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle)
      {
        if (goal_handle != active_goal_) {
          return rclcpp_action::CancelResponse::REJECT;
        }
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle)
      {
        if (active_goal_ && active_goal_->is_active()) {
          auto result = std::make_shared<NavigateToPose::Result>();
          result->success = false;
          result->message = "superseded by a newer goal";
          active_goal_->abort(result);
        }
        active_goal_ = goal_handle;
        target_ = goal_handle->get_goal()->target_pose;
        phase_ = Phase::DRIVING;
        last_command_ = geometry_msgs::msg::Twist();
        goal_start_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(
          get_logger(), "Accepted goal: (%.2f, %.2f)",
          target_.pose.position.x, target_.pose.position.y);
      });

    const auto period = std::chrono::duration<double>(1.0 / control_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this]() {control();});
    RCLCPP_INFO(get_logger(), "NavigateToPose action server ready");
  }

private:
  void stop()
  {
    last_command_ = geometry_msgs::msg::Twist();
    command_pub_->publish(last_command_);
  }

  void publish_command(const geometry_msgs::msg::Twist & target, double dt)
  {
    geometry_msgs::msg::Twist smoothed;
    smoothed.linear.x = command_smoothing_alpha_ * target.linear.x +
      (1.0 - command_smoothing_alpha_) * last_command_.linear.x;
    smoothed.angular.z = command_smoothing_alpha_ * target.angular.z +
      (1.0 - command_smoothing_alpha_) * last_command_.angular.z;
    geometry_msgs::msg::Twist limited;
    limited.linear.x = robot_navigation::limit_rate(
      smoothed.linear.x, last_command_.linear.x, max_linear_accel_, max_linear_decel_, dt);
    limited.angular.z = robot_navigation::limit_rate(
      smoothed.angular.z, last_command_.angular.z, max_angular_accel_, max_angular_decel_, dt);
    if (!finite_twist(limited)) {
      finish_with_failure("non-finite smoothed velocity command");
      return;
    }
    last_command_ = limited;
    command_pub_->publish(last_command_);
  }

  void finish_with_failure(const std::string & message)
  {
    stop();
    if (active_goal_ && active_goal_->is_active()) {
      auto result = std::make_shared<NavigateToPose::Result>();
      result->success = false;
      result->message = message;
      active_goal_->abort(result);
    }
    active_goal_.reset();
    RCLCPP_ERROR(get_logger(), "Navigation failed: %s", message.c_str());
  }

  void control()
  {
    if (!active_goal_) {
      stop();
      return;
    }

    if (active_goal_->is_canceling()) {
      stop();
      auto result = std::make_shared<NavigateToPose::Result>();
      result->success = false;
      result->message = "goal canceled";
      active_goal_->canceled(result);
      active_goal_.reset();
      RCLCPP_INFO(get_logger(), "Navigation goal canceled");
      return;
    }

    const auto steady_now = std::chrono::steady_clock::now();
    if (!odometry_received_) {
      const double waiting_seconds =
        std::chrono::duration<double>(steady_now - goal_start_time_).count();
      if (waiting_seconds > odom_timeout_) {
        finish_with_failure("odometry was not received before timeout");
      } else {
        stop();
      }
      return;
    }

    const double odom_age =
      std::chrono::duration<double>(steady_now - last_odom_time_).count();
    if (odom_age > odom_timeout_) {
      finish_with_failure("odometry timed out");
      return;
    }

    const auto control_now = std::chrono::steady_clock::now();
    const double dt = std::clamp(
      std::chrono::duration<double>(control_now - last_control_time_).count(), 1e-3, 0.2);
    last_control_time_ = control_now;
    const double dx = target_.pose.position.x - odometry_.pose.pose.position.x;
    const double dy = target_.pose.position.y - odometry_.pose.pose.position.y;
    const double distance = std::hypot(dx, dy);
    if (!std::isfinite(distance)) {
      finish_with_failure("navigation calculation produced a non-finite value");
      return;
    }

    const double yaw = yaw_from_quaternion(odometry_.pose.pose.orientation);
    geometry_msgs::msg::Twist command;
    if (distance <= position_tolerance_) {
      phase_ = Phase::ALIGNING;
      const double target_yaw = yaw_from_quaternion(target_.pose.orientation);
      const double yaw_error = std::remainder(target_yaw - yaw, 2.0 * M_PI);
      if (robot_navigation::should_stop_at_goal(
          distance, yaw_error, position_tolerance_, yaw_tolerance_)) {
        stop();
        auto result = std::make_shared<NavigateToPose::Result>();
        result->success = true; result->message = "goal position and orientation reached";
        active_goal_->succeed(result); active_goal_.reset();
        RCLCPP_INFO(get_logger(), "Goal position and orientation reached");
        return;
      }
      command.angular.z = std::clamp(angular_gain_ * yaw_error, -max_angular_speed_, max_angular_speed_);
    } else {
      double heading_error = std::remainder(std::atan2(dy, dx) - yaw, 2.0 * M_PI);
      double direction = 1.0;
      if (allow_reverse_ && std::abs(heading_error) > M_PI_2) {
        heading_error = std::remainder(heading_error + M_PI, 2.0 * M_PI);
        direction = -1.0;
      }
      command.angular.z = std::clamp(angular_gain_ * heading_error, -max_angular_speed_, max_angular_speed_);
      const double alignment = std::max(0.0, std::cos(heading_error));
      command.linear.x = direction * std::min(linear_gain_ * distance, max_linear_speed_) * alignment;
    }
    publish_command(command, dt);

    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    feedback->remaining_distance = distance;
    feedback->current_pose.header = odometry_.header;
    feedback->current_pose.pose = odometry_.pose.pose;
    active_goal_->publish_feedback(feedback);
  }

  double linear_gain_;
  double angular_gain_;
  double max_linear_speed_;
  double max_angular_speed_;
  double position_tolerance_;
  double control_rate_;
  double odom_timeout_;
  double yaw_tolerance_;
  double max_linear_accel_;
  double max_linear_decel_;
  double max_angular_accel_;
  double max_angular_decel_;
  double command_smoothing_alpha_;
  double max_odom_jump_;
  bool allow_reverse_{false};
  std::string goal_frame_;
  bool odometry_received_{false};
  bool filtered_odom_received_{false};
  double filtered_x_{0.0};
  double filtered_y_{0.0};
  double filtered_yaw_{0.0};
  double odom_filter_alpha_{0.35};
  enum class Phase { DRIVING, ALIGNING };
  Phase phase_{Phase::DRIVING};
  geometry_msgs::msg::Twist last_command_;
  std::chrono::steady_clock::time_point last_control_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_odom_time_;
  std::chrono::steady_clock::time_point goal_start_time_;
  geometry_msgs::msg::PoseStamped target_;
  nav_msgs::msg::Odometry odometry_;
  std::shared_ptr<GoalHandle> active_goal_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointController>());
  rclcpp::shutdown();
  return 0;
}
