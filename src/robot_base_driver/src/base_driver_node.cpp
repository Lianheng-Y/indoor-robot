#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <algorithm>
using namespace std::chrono_literals;
class BaseDriver final : public rclcpp::Node { public: BaseDriver():Node("base_driver") { max_speed_=declare_parameter("max_speed",0.6); sub_=create_subscription<geometry_msgs::msg::Twist>("cmd_vel",10,[this](const auto m){ auto out=*m; out.linear.x=std::clamp(out.linear.x,-max_speed_,max_speed_); out.angular.z=std::clamp(out.angular.z,-max_speed_,max_speed_); pub_->publish(out); }); pub_=create_publisher<geometry_msgs::msg::Twist>("cmd_vel_safe",10); timer_=create_wall_timer(100ms,[this]{ if((now()-last_command_).seconds()>0.5){geometry_msgs::msg::Twist z; pub_->publish(z);} }); last_command_=now(); } private: double max_speed_; rclcpp::Time last_command_; rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_; rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_; rclcpp::TimerBase::SharedPtr timer_;}; int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<BaseDriver>());rclcpp::shutdown();}
