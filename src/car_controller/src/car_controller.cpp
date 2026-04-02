#include <algorithm>
#include <car_controller/car_controller.hpp>
#include <controller_interface/controller_interface.hpp>
#include <iomanip>
#include <pluginlib/class_list_macros.hpp>
#include <robot_interfaces/msg/detail/wheel_exp__struct.hpp>
#include <robot_interfaces/msg/wheel.hpp>
#include <robot_interfaces/msg/wheel_exp.hpp>

namespace car_controller {
namespace {
constexpr size_t kLeftWheelVelocityIndex = 0;
constexpr size_t kLeftWheelEffortIndex = 1;
constexpr size_t kRightWheelVelocityIndex = 2;
constexpr size_t kRightWheelEffortIndex = 3;
constexpr size_t kExpectedStateInterfaceCount = 4;
constexpr size_t kExpectedCommandInterfaceCount = 2;
}  // namespace

CarController::CarController() {}

controller_interface::CallbackReturn CarController::on_init() {
    state_publisher   = get_node()->create_publisher<robot_interfaces::msg::Wheel>("wheels_status", 10);
    target_subscriber = get_node()->create_subscription<robot_interfaces::msg::WheelExp>(
        "wheels_target", 10, [this](const robot_interfaces::msg::WheelExp& msg) { wheel_target = msg; });
    wheel_name_ = {"left_wheel_joint", "right_wheel_joint"};


    auto node = get_node();
    node->declare_parameter("joint_torque_filter_gate", 0.8);
    node->declare_parameter("joint_omega_filter_gate", 0.8);

    param_cb_ = node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& param : params) {
            if (param.get_name() == "joint_torque_filter_gate")                 // 设置电机力矩低通滤波器增益
                joint_torque_filter_gate = param.as_double();
            else if (param.get_name() == "joint_omega_filter_gate")                  // 设置电机转速低通滤波器增益
                joint_omega_filter_gate = param.as_double();
        }
        return result;
    });

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CarController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    auto node = get_node();

    wheel_target.left_omega=0.0f;
    wheel_target.right_omega=0.0f;
    wheel_target.left_torque=0.0f;
    wheel_target.right_torque=0.0f;
    wheel_target.kd=0.0f;
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn CarController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn CarController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CarController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    (void)period;

    if (state_interfaces_.size() != kExpectedStateInterfaceCount || command_interfaces_.size() != kExpectedCommandInterfaceCount) {
        RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(),
            *get_node()->get_clock(),
            2000,
            "Unexpected interface count. state=%zu command=%zu",
            state_interfaces_.size(),
            command_interfaces_.size());
        return controller_interface::return_type::ERROR;
    }

    wheel_state.left_omega = static_cast<float>(state_interfaces_[kLeftWheelVelocityIndex].get_value());
    wheel_state.left_torque = static_cast<float>(state_interfaces_[kLeftWheelEffortIndex].get_value());
    wheel_state.right_omega = static_cast<float>(state_interfaces_[kRightWheelVelocityIndex].get_value());
    wheel_state.right_torque = static_cast<float>(state_interfaces_[kRightWheelEffortIndex].get_value());
    state_publisher->publish(wheel_state);                                           // 发布轮子状态

    wheel_kd=wheel_target.kd;
    double left_effort  = wheel_kd * (wheel_target.left_omega - wheel_state.left_omega) + wheel_target.left_torque;
    double right_effort = wheel_kd * (wheel_target.right_omega - wheel_state.right_omega) + wheel_target.right_torque;
    left_effort         = std::clamp(left_effort, -5.0, 5.0);
    right_effort        = std::clamp(right_effort, -5.0, 5.0);

    command_interfaces_[0].set_value(left_effort);                                   // 写力矩
    command_interfaces_[1].set_value(right_effort);
    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration CarController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : wheel_name_) {
        cfg.names.push_back(name + "/effort");
    }
    return cfg;
}

controller_interface::InterfaceConfiguration CarController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : wheel_name_) {
        cfg.names.push_back(name + "/velocity");
        cfg.names.push_back(name + "/effort");
    }
    return cfg;
}
} // namespace car_controller

PLUGINLIB_EXPORT_CLASS(car_controller::CarController, controller_interface::ControllerInterface)
