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
  using NavigateToPose = robot_interfaces::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  Planner() : Node("planner")
  {
    linear_gain_ = declare_parameter("linear_gain", 0.8);
    angular_gain_ = declare_parameter("angular_gain", 2.0);
    max_linear_speed_ = declare_parameter("max_linear_speed", 0.45);
    max_angular_speed_ = declare_parameter("max_angular_speed", 1.2);
    position_tolerance_ = declare_parameter("position_tolerance", 0.08);
    control_rate_ = declare_parameter("control_rate", 20.0);
    odom_timeout_ = declare_parameter("odom_timeout", 0.5);
    goal_frame_ = declare_parameter("goal_frame", "odom");
    if (linear_gain_ <= 0.0 || angular_gain_ <= 0.0 || max_linear_speed_ <= 0.0 ||
      max_angular_speed_ <= 0.0 || position_tolerance_ <= 0.0 ||
      control_rate_ <= 0.0 || odom_timeout_ <= 0.0)
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
        odometry_ = *message;
        odometry_received_ = true;
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
        if (!finite_pose(target.pose)) {
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
    command_pub_->publish(geometry_msgs::msg::Twist());
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

    const double dx = target_.pose.position.x - odometry_.pose.pose.position.x;
    const double dy = target_.pose.position.y - odometry_.pose.pose.position.y;
    const double distance = std::hypot(dx, dy);
    if (!std::isfinite(distance)) {
      finish_with_failure("navigation calculation produced a non-finite value");
      return;
    }

    if (distance <= position_tolerance_) {
      stop();
      auto result = std::make_shared<NavigateToPose::Result>();
      result->success = true;
      result->message = "goal reached";
      active_goal_->succeed(result);
      active_goal_.reset();
      RCLCPP_INFO(get_logger(), "Goal reached");
      return;
    }

    const double yaw = yaw_from_quaternion(odometry_.pose.pose.orientation);
    const double heading_error = std::remainder(std::atan2(dy, dx) - yaw, 2.0 * M_PI);
    geometry_msgs::msg::Twist command;
    command.angular.z = std::clamp(
      angular_gain_ * heading_error, -max_angular_speed_, max_angular_speed_);
    const double alignment = std::max(0.0, std::cos(heading_error));
    command.linear.x = std::min(linear_gain_ * distance, max_linear_speed_) * alignment;
    if (!finite_twist(command)) {
      finish_with_failure("navigation produced a non-finite velocity command");
      return;
    }
    command_pub_->publish(command);

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
  std::string goal_frame_;
  bool odometry_received_{false};
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
  rclcpp::spin(std::make_shared<Planner>());
  rclcpp::shutdown();
  return 0;
}
