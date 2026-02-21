#include <rclcpp/rclcpp.hpp>
#include <move_control/controller.hpp>

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("robot_controller_node");
    auto robot_calc=std::make_shared<RobotController>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
