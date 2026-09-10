#include <rclcpp/rclcpp.hpp>
class Planner: public rclcpp::Node { public: Planner():Node("planner") { RCLCPP_INFO(get_logger(),"Planner extension point ready (Nav2/A* integration)"); }}; int main(int a,char**b){rclcpp::init(a,b);rclcpp::spin(std::make_shared<Planner>());rclcpp::shutdown();}
