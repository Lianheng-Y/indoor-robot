#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class TaskManager final : public rclcpp::Node
{
public:
  TaskManager() : Node("task_manager")
  {
    waypoint_x_ = declare_parameter<std::vector<double>>("waypoint_x", {1.0, 1.0, 0.0});
    waypoint_y_ = declare_parameter<std::vector<double>>("waypoint_y", {0.0, 1.0, 0.0});
    frame_id_ = declare_parameter("frame_id", "odom");
    auto_start_ = declare_parameter("auto_start", false);
    if (waypoint_x_.empty() || waypoint_x_.size() != waypoint_y_.size()) {
      throw std::invalid_argument("waypoint_x and waypoint_y must have the same non-zero size");
    }

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "goal_pose", rclcpp::QoS(1).transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "task_status", rclcpp::QoS(1).transient_local());
    cancel_pub_ = create_publisher<std_msgs::msg::Bool>("navigation_cancel", rclcpp::QoS(10));
    command_sub_ = create_subscription<std_msgs::msg::String>(
      "task_command", rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr message) {handle_command(message->data);});
    reached_sub_ = create_subscription<std_msgs::msg::Bool>(
      "goal_reached", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        if (message->data && state_ == State::PATROL) {
          waypoint_index_++;
          if (waypoint_index_ < waypoint_x_.size()) {
            publish_goal(waypoint_index_);
          } else {
            state_ = State::IDLE;
            publish_status("completed");
          }
        } else if (message->data && state_ == State::RETURNING_HOME) {
          state_ = State::IDLE;
          publish_status("home");
        }
      });

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
      state_ = State::RETURNING_HOME;
      publish_status("returning_home");
      publish_coordinates(0.0, 0.0);
    } else if (command == "stop") {
      state_ = State::IDLE;
      publish_status("stopped");
      std_msgs::msg::Bool cancel;
      cancel.data = true;
      cancel_pub_->publish(cancel);
    } else {
      RCLCPP_WARN(get_logger(), "Unknown task command: '%s'", command.c_str());
      publish_status("invalid_command");
    }
  }

  void start_patrol()
  {
    state_ = State::PATROL;
    waypoint_index_ = 0;
    publish_status("patrolling");
    publish_goal(waypoint_index_);
  }

  void publish_goal(std::size_t index)
  {
    publish_coordinates(waypoint_x_[index], waypoint_y_[index]);
  }

  void publish_coordinates(double x, double y)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = frame_id_;
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.orientation.w = 1.0;
    goal_pub_->publish(goal);
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
  std::vector<double> waypoint_x_;
  std::vector<double> waypoint_y_;
  std::string frame_id_;
  bool auto_start_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cancel_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_sub_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskManager>());
  rclcpp::shutdown();
  return 0;
}
