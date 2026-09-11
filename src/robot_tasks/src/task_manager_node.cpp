#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_interfaces/action/navigate_to_pose.hpp>
#include <std_msgs/msg/string.hpp>

class TaskManager final : public rclcpp::Node
{
public:
  using NavigateToPose = robot_interfaces::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  TaskManager() : Node("task_manager")
  {
    waypoint_x_ = declare_parameter<std::vector<double>>("waypoint_x", {1.0, 1.0, 0.0});
    waypoint_y_ = declare_parameter<std::vector<double>>("waypoint_y", {0.0, 1.0, 0.0});
    frame_id_ = declare_parameter("frame_id", "odom");
    auto_start_ = declare_parameter("auto_start", false);
    if (waypoint_x_.empty() || waypoint_x_.size() != waypoint_y_.size()) {
      throw std::invalid_argument("waypoint_x and waypoint_y must have the same non-zero size");
    }
    for (std::size_t index = 0; index < waypoint_x_.size(); ++index) {
      if (!std::isfinite(waypoint_x_[index]) || !std::isfinite(waypoint_y_[index])) {
        throw std::invalid_argument("waypoints must contain only finite coordinates");
      }
    }

    status_pub_ = create_publisher<std_msgs::msg::String>(
      "task_status", rclcpp::QoS(1).transient_local());
    command_sub_ = create_subscription<std_msgs::msg::String>(
      "task_command", rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr message) {handle_command(message->data);});
    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    publish_status("idle");
    if (auto_start_) {
      startup_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
        startup_timer_->cancel();
        start_patrol();
      });
    }
    RCLCPP_INFO(get_logger(), "Task manager ready; commands: patrol, home, stop");
  }

private:
  enum class State {IDLE, PATROL, RETURNING_HOME};

  void handle_command(const std::string & command)
  {
    if (command == "patrol") {
      start_patrol();
    } else if (command == "home") {
      ++request_id_;
      navigation_client_->async_cancel_all_goals();
      state_ = State::RETURNING_HOME;
      publish_status("returning_home");
      send_navigation_goal(0.0, 0.0);
    } else if (command == "stop") {
      ++request_id_;
      state_ = State::IDLE;
      navigation_client_->async_cancel_all_goals();
      publish_status("stopped");
    } else {
      RCLCPP_WARN(get_logger(), "Unknown task command: '%s'", command.c_str());
      publish_status("invalid_command");
    }
  }

  void start_patrol()
  {
    ++request_id_;
    navigation_client_->async_cancel_all_goals();
    state_ = State::PATROL;
    waypoint_index_ = 0;
    publish_status("patrolling");
    send_navigation_goal(waypoint_x_[waypoint_index_], waypoint_y_[waypoint_index_]);
  }

  void send_navigation_goal(double x, double y)
  {
    if (!navigation_client_->wait_for_action_server(std::chrono::seconds(1))) {
      state_ = State::IDLE;
      publish_status("navigation_unavailable");
      RCLCPP_ERROR(get_logger(), "NavigateToPose action server is unavailable");
      return;
    }

    NavigateToPose::Goal goal;
    goal.target_pose.header.stamp = now();
    goal.target_pose.header.frame_id = frame_id_;
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = 1.0;

    const std::uint64_t sent_request_id = request_id_;
    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.result_callback =
      [this, sent_request_id](const GoalHandle::WrappedResult & wrapped_result) {
        if (sent_request_id != request_id_) {
          return;
        }
        if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
          !wrapped_result.result || !wrapped_result.result->success)
        {
          state_ = State::IDLE;
          const std::string reason = wrapped_result.result ?
            wrapped_result.result->message : "goal rejected or result unavailable";
          publish_status("navigation_failed: " + reason);
          return;
        }

        if (state_ == State::PATROL) {
          ++waypoint_index_;
          if (waypoint_index_ < waypoint_x_.size()) {
            send_navigation_goal(waypoint_x_[waypoint_index_], waypoint_y_[waypoint_index_]);
          } else {
            state_ = State::IDLE;
            publish_status("completed");
          }
        } else if (state_ == State::RETURNING_HOME) {
          state_ = State::IDLE;
          publish_status("home");
        }
      };
    options.feedback_callback =
      [this, sent_request_id](
        GoalHandle::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
      {
        if (sent_request_id == request_id_) {
          RCLCPP_DEBUG(
            get_logger(), "Remaining distance: %.3f m", feedback->remaining_distance);
        }
      };

    navigation_client_->async_send_goal(goal, options);
  }

  void publish_status(const std::string & value)
  {
    std_msgs::msg::String status;
    status.data = value;
    status_pub_->publish(status);
    RCLCPP_INFO(get_logger(), "Task status: %s", value.c_str());
  }

  State state_{State::IDLE};
  std::size_t waypoint_index_{0};
  std::uint64_t request_id_{0};
  std::vector<double> waypoint_x_;
  std::vector<double> waypoint_y_;
  std::string frame_id_;
  bool auto_start_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskManager>());
  rclcpp::shutdown();
  return 0;
}
