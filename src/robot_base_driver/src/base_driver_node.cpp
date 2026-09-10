#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <algorithm>
#include <cmath>
using namespace std::chrono_literals;
class BaseDriver final : public rclcpp::Node {
public: BaseDriver():Node("base_driver"), x_(0), y_(0), yaw_(0) {
 max_speed_=declare_parameter("max_speed",0.6); wheel_base_=declare_parameter("wheel_base",0.32);
 sub_=create_subscription<geometry_msgs::msg::Twist>("cmd_vel",10,[this](geometry_msgs::msg::Twist::ConstSharedPtr m){cmd_=*m; last_command_=now();});
 pub_=create_publisher<nav_msgs::msg::Odometry>("odom",10); safe_pub_=create_publisher<geometry_msgs::msg::Twist>("cmd_vel_safe",10); tf_=std::make_unique<tf2_ros::TransformBroadcaster>(*this); last_command_=now(); last_tick_=now(); timer_=create_wall_timer(20ms,[this]{tick();}); }
private: void tick(){auto t=now(); double dt=(t-last_tick_).seconds(); last_tick_=t; auto c=cmd_; if((t-last_command_).seconds()>0.5)c=geometry_msgs::msg::Twist(); c.linear.x=std::clamp(c.linear.x,-max_speed_,max_speed_); c.angular.z=std::clamp(c.angular.z,-2.0,2.0); safe_pub_->publish(c); yaw_+=c.angular.z*dt; x_+=c.linear.x*std::cos(yaw_)*dt; y_+=c.linear.x*std::sin(yaw_)*dt; nav_msgs::msg::Odometry o; o.header.stamp=t; o.header.frame_id="odom"; o.child_frame_id="base_link"; o.pose.pose.position.x=x_;o.pose.pose.position.y=y_; tf2::Quaternion q;q.setRPY(0,0,yaw_);o.pose.pose.orientation.x=q.x();o.pose.pose.orientation.y=q.y();o.pose.pose.orientation.z=q.z();o.pose.pose.orientation.w=q.w();o.twist.twist=c;pub_->publish(o); geometry_msgs::msg::TransformStamped tr;tr.header=o.header;tr.child_frame_id="base_link";tr.transform.translation.x=x_;tr.transform.translation.y=y_;tr.transform.rotation=o.pose.pose.orientation;tf_->sendTransform(tr);}
 double max_speed_,wheel_base_,x_,y_,yaw_; rclcpp::Time last_command_,last_tick_; geometry_msgs::msg::Twist cmd_; rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_; rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_; rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_pub_; std::unique_ptr<tf2_ros::TransformBroadcaster> tf_; rclcpp::TimerBase::SharedPtr timer_;};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<BaseDriver>());rclcpp::shutdown();}
