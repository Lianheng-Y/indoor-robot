#include <rclcpp/rclcpp.hpp>
class TaskManager: public rclcpp::Node { public: TaskManager():Node("task_manager") { RCLCPP_INFO(get_logger(),"Task manager state machine ready"); }}; int main(int a,char**b){rclcpp::init(a,b);rclcpp::spin(std::make_shared<TaskManager>());rclcpp::shutdown();}
