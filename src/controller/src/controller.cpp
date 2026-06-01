#include <controller/controller.hpp>

#include <algorithm>
#include <array>

#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <robot_interfaces/msg/motor_state.hpp>
#include <robot_interfaces/msg/motor_target.hpp>

namespace car_controller {

CarController::CarController() = default;

controller_interface::CallbackReturn CarController::on_init() {
    state_publisher_ = get_node()->create_publisher<robot_interfaces::msg::RobotState>("robot_state", 10);
    target_subscriber_ = get_node()->create_subscription<robot_interfaces::msg::RobotTarget>(
        "robot_target", 10, [this](const robot_interfaces::msg::RobotTarget& msg) { robot_target_ = msg; });

    motor_joint_names_ = {
        "left_front_hip_joint",
        "left_rear_hip_joint",
        "left_wheel_joint",
        "right_front_hip_joint",
        "right_rear_hip_joint",
        "right_wheel_joint",
    };

    auto node = get_node();
    node->declare_parameter("torque_limit", torque_limit_);

    param_cb_ = node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& param : params) {
            if (param.get_name() == "torque_limit") {
                torque_limit_ = param.as_double();
            }
        }
        return result;
    });

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CarController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    for (size_t i = 0; i < kMotorCount; ++i) {
        auto& target = target_at(robot_target_, i);
        target.rad = 0.0F;
        target.omega = 0.0F;
        target.torque = 0.0F;
        target.kp = 0.0F;
        target.kd = 0.0F;
    }

    torque_limit_ = get_node()->get_parameter("torque_limit").as_double();
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CarController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CarController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CarController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    (void)period;

    }

    state_publisher_->publish(robot_state_);
    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration CarController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : motor_joint_names_) {
        cfg.names.push_back(name + "/effort");
    }
    return cfg;
}

controller_interface::InterfaceConfiguration CarController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : motor_joint_names_) {
        cfg.names.push_back(name + "/position");
        cfg.names.push_back(name + "/velocity");
        cfg.names.push_back(name + "/effort");
    }
    return cfg;
}

}  // namespace car_controller

PLUGINLIB_EXPORT_CLASS(car_controller::CarController, controller_interface::ControllerInterface)
