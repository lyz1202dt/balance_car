#include <controller/real_hardware.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace car_controller {

hardware_interface::CallbackReturn RealHardware::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto imu_name_it = info.hardware_parameters.find("imu_sensor_name");
    if (imu_name_it != info.hardware_parameters.end()) {
        imu_sensor_name_ = imu_name_it->second;
    }

    joints_.clear();
    joints_.reserve(info.joints.size());
    for (const auto& joint_info : info.joints) {
        joints_.push_back(JointData{joint_info.name});
    }

    imu_state_.fill(0.0);
    imu_state_[3] = 1.0;
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RealHardware::export_state_interfaces() {
    state_interfaces_.clear();
    for (auto& joint : joints_) {
        state_interfaces_.emplace_back(joint.name, "position", &joint.position);
        state_interfaces_.emplace_back(joint.name, "velocity", &joint.velocity);
        state_interfaces_.emplace_back(joint.name, "effort", &joint.effort);
    }

    state_interfaces_.emplace_back(imu_sensor_name_, "orientation.x", &imu_state_[0]);
    state_interfaces_.emplace_back(imu_sensor_name_, "orientation.y", &imu_state_[1]);
    state_interfaces_.emplace_back(imu_sensor_name_, "orientation.z", &imu_state_[2]);
    state_interfaces_.emplace_back(imu_sensor_name_, "orientation.w", &imu_state_[3]);
    state_interfaces_.emplace_back(imu_sensor_name_, "angular_velocity.x", &imu_state_[4]);
    state_interfaces_.emplace_back(imu_sensor_name_, "angular_velocity.y", &imu_state_[5]);
    state_interfaces_.emplace_back(imu_sensor_name_, "angular_velocity.z", &imu_state_[6]);
    state_interfaces_.emplace_back(imu_sensor_name_, "linear_acceleration.x", &imu_state_[7]);
    state_interfaces_.emplace_back(imu_sensor_name_, "linear_acceleration.y", &imu_state_[8]);
    state_interfaces_.emplace_back(imu_sensor_name_, "linear_acceleration.z", &imu_state_[9]);
    return std::move(state_interfaces_);
}

std::vector<hardware_interface::CommandInterface> RealHardware::export_command_interfaces() {
    command_interfaces_.clear();
    for (auto& joint : joints_) {
        command_interfaces_.emplace_back(joint.name, "effort", &joint.effort_command);
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

    // TODO(real hardware): Read motor encoders/velocity/torque and IMU, then fill joints_ and imu_state_.
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RealHardware::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    (void)period;

    // TODO(real hardware): Send joints_[i].effort_command to the physical motor drivers.
    return hardware_interface::return_type::OK;
}

}  // namespace car_controller

PLUGINLIB_EXPORT_CLASS(car_controller::RealHardware, hardware_interface::SystemInterface)
