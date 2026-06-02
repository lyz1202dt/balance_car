#include <controller/real_hardware.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace lqr_controller {

hardware_interface::CallbackReturn RealHardware::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    joints_.clear();
    joints_.reserve(info.joints.size());
    for (const auto& joint_info : info.joints) {
        joints_.push_back(JointData{joint_info.name});
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RealHardware::export_state_interfaces() {
    state_interfaces_.clear();
    for (auto& joint : joints_) {
        state_interfaces_.emplace_back(joint.name, "position", &joint.position);
        state_interfaces_.emplace_back(joint.name, "velocity", &joint.velocity);
        state_interfaces_.emplace_back(joint.name, "effort", &joint.effort);
    }

    return std::move(state_interfaces_);
}

std::vector<hardware_interface::CommandInterface> RealHardware::export_command_interfaces() {
    command_interfaces_.clear();
    for (auto& joint : joints_) {
        command_interfaces_.emplace_back(joint.name, "position", &joint.position_command);
        command_interfaces_.emplace_back(joint.name, "velocity", &joint.velocity_command);
        command_interfaces_.emplace_back(joint.name, "effort", &joint.effort_command);
        command_interfaces_.emplace_back(joint.name, "kp", &joint.kp_command);
        command_interfaces_.emplace_back(joint.name, "kd", &joint.kd_command);
    }
    return std::move(command_interfaces_);
}

hardware_interface::CallbackReturn RealHardware::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RealHardware::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RealHardware::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    (void)period;

    // TODO(real hardware): Read motor encoders/velocity/torque, then fill joints_.
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RealHardware::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    (void)period;

    // TODO(real hardware): Send position/velocity/effort/kp/kd commands to the physical motor drivers.
    return hardware_interface::return_type::OK;
}

}  // namespace lqr_controller

PLUGINLIB_EXPORT_CLASS(lqr_controller::RealHardware, hardware_interface::SystemInterface)
