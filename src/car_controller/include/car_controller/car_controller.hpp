#include <robot_interfaces/msg//wheel.hpp>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <string>
#include <chrono>

namespace car_controller {
class CarController : public controller_interface::ControllerInterface {
public:
    CarController();
    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
    rclcpp::Publisher<robot_interfaces::msg::Wheel>::SharedPtr state_publisher;
    rclcpp::Subscription<robot_interfaces::msg::Wheel>::SharedPtr target_subscriber;
    std::vector<std::string> wheel_name_;
    rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    robot_interfaces::msg::Wheel wheel_target;
    robot_interfaces::msg::Wheel wheel_state;

    double joint_torque_filter_gate{0.8};
    double joint_omega_filter_gate{0.8};
    double wheel_kd{0.0};
};
} // namespace dog_controller
