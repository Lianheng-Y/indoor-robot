#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_interfaces/action/navigate_to_pose.hpp>
#include <robot_interfaces/action/navigate_waypoints.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace {
bool finite_pose(const geometry_msgs::msg::Pose & p) {
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) && std::isfinite(p.position.z) &&
         std::isfinite(p.orientation.x) && std::isfinite(p.orientation.y) &&
         std::isfinite(p.orientation.z) && std::isfinite(p.orientation.w);
}
}

class TaskManager final : public rclcpp::Node {
public:
  using Nav = robot_interfaces::action::NavigateToPose;
  using Waypoints = robot_interfaces::action::NavigateWaypoints;
  using NavClient = rclcpp_action::Client<Nav>;
  using NavHandle = NavClient::GoalHandle;
  using TaskHandle = rclcpp_action::ServerGoalHandle<Waypoints>;

  TaskManager() : Node("task_manager") {
    xs_ = declare_parameter<std::vector<double>>("waypoint_x", {1.0, 1.0, 0.0});
    ys_ = declare_parameter<std::vector<double>>("waypoint_y", {0.0, 1.0, 0.0});
    frame_ = declare_parameter("frame_id", "odom");
    auto_start_ = declare_parameter("auto_start", false);
    timeout_ = declare_parameter("waypoint_timeout", 60.0);
    max_retries_ = declare_parameter("max_retries", 2);
    skip_failed_ = declare_parameter("skip_on_failure", false);
    battery_threshold_ = declare_parameter("battery_threshold", 0.20);
    state_file_ = declare_parameter("state_file", "/tmp/robot_task_state");
    resume_on_start_ = declare_parameter("resume_on_start", false);
    if (xs_.empty() || xs_.size() != ys_.size() || timeout_ <= 0.0 || max_retries_ < 0 ||
      battery_threshold_ < 0.0 || battery_threshold_ > 1.0) throw std::invalid_argument("invalid task parameters");
    for (std::size_t i = 0; i < xs_.size(); ++i)
      if (!std::isfinite(xs_[i]) || !std::isfinite(ys_[i])) throw std::invalid_argument("waypoints must be finite");

    status_pub_ = create_publisher<std_msgs::msg::String>("task_status", rclcpp::QoS(1).transient_local());
    command_sub_ = create_subscription<std_msgs::msg::String>("task_command", 10,
      [this](std_msgs::msg::String::ConstSharedPtr m) { command(m->data); });
    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>("battery_state", 10,
      [this](sensor_msgs::msg::BatteryState::ConstSharedPtr m) {
        if (std::isfinite(m->percentage) && m->percentage >= 0.0 && m->percentage <= battery_threshold_ &&
          state_ != State::RETURNING_HOME) { RCLCPP_WARN(get_logger(), "Low battery; returning home"); home(); }
      });
    nav_client_ = rclcpp_action::create_client<Nav>(this, "navigate_to_pose");
    task_server_ = rclcpp_action::create_server<Waypoints>(this, "navigate_waypoints",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Waypoints::Goal> g) {
        if (g->waypoints.empty() || (!g->dwell_times.empty() && g->dwell_times.size() != g->waypoints.size()))
          return rclcpp_action::GoalResponse::REJECT;
        for (std::size_t i = 0; i < g->waypoints.size(); ++i)
          if ((!g->waypoints[i].header.frame_id.empty() && g->waypoints[i].header.frame_id != frame_) ||
            !finite_pose(g->waypoints[i].pose) || (!g->dwell_times.empty() &&
            (!std::isfinite(g->dwell_times[i]) || g->dwell_times[i] < 0.0F))) return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<TaskHandle> h) { return h == task_handle_ ? rclcpp_action::CancelResponse::ACCEPT : rclcpp_action::CancelResponse::REJECT; },
      [this](std::shared_ptr<TaskHandle> h) { start_action(h); });
    timer_ = create_wall_timer(100ms, [this]() { tick(); });
    publish("idle");
    if (auto_start_) startup_timer_ = create_wall_timer(500ms, [this]() {
      startup_timer_->cancel();
      if (resume_on_start_ && load()) { state_ = State::PATROL; publish("resuming"); send(); }
      else start_patrol();
    });
  }

private:
  enum class State { IDLE, PATROL, PAUSED, RETURNING_HOME };
  void command(const std::string & c) {
    if (c == "patrol") { if (state_ == State::PATROL || state_ == State::PAUSED) publish("patrol_ignored"); else start_patrol(); }
    else if (c == "pause") pause(); else if (c == "resume") resume(); else if (c == "home") home();
    else if (c == "stop") stop("stopped by operator"); else publish("invalid_command");
  }
  void start_patrol() {
    abort_task("preempted by patrol"); cancel_nav(); build_configured_waypoints();
    state_ = State::PATROL; index_ = completed_ = retries_ = 0; loop_ = false; ++generation_; persist(); publish("patrolling"); send();
  }
  void home() {
    abort_task("preempted by return-home"); cancel_nav(); waypoints_.clear(); dwell_ = {0.0F};
    geometry_msgs::msg::PoseStamped p; p.header.frame_id = frame_; p.pose.orientation.w = 1.0; waypoints_.push_back(p);
    state_ = State::RETURNING_HOME; index_ = completed_ = retries_ = 0; loop_ = false; ++generation_; publish("returning_home"); send();
  }
  void build_configured_waypoints() {
    waypoints_.clear(); dwell_.assign(xs_.size(), 0.0F);
    for (std::size_t i = 0; i < xs_.size(); ++i) { geometry_msgs::msg::PoseStamped p; p.header.frame_id = frame_; p.pose.position.x = xs_[i]; p.pose.position.y = ys_[i]; p.pose.orientation.w = 1.0; waypoints_.push_back(p); }
  }
  void start_action(std::shared_ptr<TaskHandle> h) {
    abort_task("preempted by newer waypoint task"); cancel_nav(); task_handle_ = h; const auto g = h->get_goal();
    waypoints_ = g->waypoints; dwell_ = g->dwell_times.empty() ? std::vector<float>(waypoints_.size(), 0.0F) : g->dwell_times;
    loop_ = g->loop; state_ = State::PATROL; index_ = completed_ = retries_ = 0; ++generation_; persist(); publish("patrolling"); send();
  }
  void pause() { if (state_ != State::PATROL) { publish("pause_ignored"); return; } cancel_nav(); state_ = State::PAUSED; persist(); publish("paused"); }
  void resume() { if (state_ != State::PAUSED) { publish("resume_ignored"); return; } state_ = State::PATROL; publish("patrolling"); send(); }
  void stop(const std::string & why) { ++generation_; cancel_nav(); abort_task(why); state_ = State::IDLE; persist(); publish("stopped"); }
  void send() {
    if (state_ == State::IDLE || state_ == State::PAUSED || index_ >= waypoints_.size()) return;
    if (!nav_client_->wait_for_action_server(1s)) { failure("navigation unavailable"); return; }
    Nav::Goal g; g.target_pose = waypoints_[index_]; g.target_pose.header.stamp = now(); const auto token = ++nav_token_; started_ = std::chrono::steady_clock::now();
    auto o = NavClient::SendGoalOptions();
    o.goal_response_callback = [this, token](std::shared_ptr<NavHandle> h) { if (token == nav_token_) { nav_goal_ = h; if (!h) failure("navigation goal rejected"); } };
    o.result_callback = [this, token](const NavClient::WrappedResult & r) { if (token != nav_token_) return; nav_goal_.reset(); if (r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result && r.result->success) dwell_until_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int>(dwell_[index_] * 1000.0F)); else failure(r.result ? r.result->message : "navigation failed"); };
    nav_client_->async_send_goal(g, o);
  }
  void tick() {
    feedback();
    if (task_handle_ && task_handle_->is_canceling()) {
      cancel_nav();
      auto r = std::make_shared<Waypoints::Result>(); r->success = false; r->completed_waypoints = completed_; r->message = "canceled by client";
      task_handle_->canceled(r); task_handle_.reset(); state_ = State::IDLE; persist(); publish("canceled");
      return;
    }
    if (state_ == State::IDLE || state_ == State::PAUSED) return;
    if (nav_goal_ && std::chrono::steady_clock::now() - started_ > std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(timeout_))) { cancel_nav(); failure("waypoint timeout"); return; }
    if (dwell_until_.time_since_epoch().count() && std::chrono::steady_clock::now() >= dwell_until_) { dwell_until_ = {}; ++completed_; ++index_; retries_ = 0; if (index_ >= waypoints_.size()) { if (loop_) index_ = 0; else { finish(true, state_ == State::RETURNING_HOME ? "home" : "completed"); return; } } persist(); send(); }
  }
  void failure(const std::string & reason) { if (++retries_ <= max_retries_) { publish("retrying_waypoint"); send(); } else if (skip_failed_ && index_ + 1 < waypoints_.size()) { ++index_; retries_ = 0; publish("skipping_waypoint"); persist(); send(); } else finish(false, reason); }
  void finish(bool ok, const std::string & msg) { cancel_nav(); if (task_handle_ && task_handle_->is_active()) { auto r = std::make_shared<Waypoints::Result>(); r->success = ok; r->completed_waypoints = completed_; r->message = msg; if (ok) task_handle_->succeed(r); else task_handle_->abort(r); } task_handle_.reset(); state_ = State::IDLE; persist(); publish(ok ? "completed" : "failed: " + msg); }
  void abort_task(const std::string & msg) { if (task_handle_ && task_handle_->is_active()) { auto r = std::make_shared<Waypoints::Result>(); r->success = false; r->completed_waypoints = completed_; r->message = msg; task_handle_->abort(r); } task_handle_.reset(); }
  void cancel_nav() { ++nav_token_; if (nav_goal_) nav_client_->async_cancel_goal(nav_goal_); nav_goal_.reset(); dwell_until_ = {}; }
  void feedback() { if (!task_handle_ || !task_handle_->is_active()) return; auto f = std::make_shared<Waypoints::Feedback>(); f->current_waypoint = static_cast<uint32_t>(std::min(index_, waypoints_.size())); f->completion_percentage = waypoints_.empty() ? 100.0F : 100.0F * completed_ / waypoints_.size(); f->state = state_ == State::PAUSED ? "paused" : (state_ == State::RETURNING_HOME ? "returning_home" : "patrolling"); task_handle_->publish_feedback(f); }
  void publish(const std::string & s) { std_msgs::msg::String m; m.data = s; status_pub_->publish(m); RCLCPP_INFO(get_logger(), "Task status: %s", s.c_str()); }
  void persist() { std::ofstream f(state_file_, std::ios::trunc); if (f) f << static_cast<int>(state_) << ' ' << index_ << ' ' << completed_ << '\n'; }
  bool load() { std::ifstream f(state_file_); int s; if (!(f >> s >> index_ >> completed_) || s != static_cast<int>(State::PATROL) || index_ >= xs_.size()) return false; build_configured_waypoints(); return true; }

  State state_{State::IDLE}; std::vector<double> xs_, ys_; std::vector<geometry_msgs::msg::PoseStamped> waypoints_; std::vector<float> dwell_; std::string frame_, state_file_; bool auto_start_{}, resume_on_start_{}, loop_{}, skip_failed_{}; double timeout_{60.0}, battery_threshold_{0.2}; int max_retries_{2}, retries_{}; std::size_t index_{}, completed_{}; uint64_t generation_{}, nav_token_{}; std::chrono::steady_clock::time_point started_, dwell_until_;
  NavClient::SharedPtr nav_client_; NavHandle::SharedPtr nav_goal_; std::shared_ptr<TaskHandle> task_handle_; rclcpp_action::Server<Waypoints>::SharedPtr task_server_; rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_; rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_; rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_; rclcpp::TimerBase::SharedPtr timer_, startup_timer_;
};

int main(int argc, char ** argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<TaskManager>()); rclcpp::shutdown(); return 0; }
