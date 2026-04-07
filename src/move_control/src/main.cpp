#include <rclcpp/rclcpp.hpp>
#include "move_control/controller.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions options;
    auto node = rclcpp::Node::make_shared("move_control_node", options);
    RobotController controller(node);
    
    RCLCPP_INFO(node->get_logger(), "Move control node started");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
