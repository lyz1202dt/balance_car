#include <controller/mujoco_sim.hpp>

#include <utility>

#include <pluginlib/class_list_macros.hpp>

namespace car_controller {

hardware_interface::CallbackReturn MujocoSimSystem::on_init(const hardware_interface::HardwareInfo& system_info) {
    const auto result = mujoco_ros2_control::MujocoSystem::on_init(system_info);
    if (result != hardware_interface::CallbackReturn::SUCCESS) {
        return result;
    }

    const auto imu_name_it = system_info.hardware_parameters.find("imu_sensor_name");
    if (imu_name_it != system_info.hardware_parameters.end()) {
        imu_sensor_name_ = imu_name_it->second;
    }

    const auto imu_topic_it = system_info.hardware_parameters.find("imu_topic");
    if (imu_topic_it != system_info.hardware_parameters.end()) {
        imu_topic_ = imu_topic_it->second;
    }

    imu_state_.fill(0.0);
    imu_state_[3] = 1.0;

    imu_node_ = rclcpp::Node::make_shared(
        "mujoco_sim_interface", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));
    imu_subscriber_ = imu_node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::SharedPtr msg) { imu_callback(std::move(msg)); });
    imu_executor_.add_node(imu_node_);

    RCLCPP_INFO(
        imu_node_->get_logger(), "Subscribing MuJoCo IMU topic '%s' as ros2_control sensor '%s'",
        imu_topic_.c_str(), imu_sensor_name_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MujocoSimSystem::export_state_interfaces() {
    auto interfaces = mujoco_ros2_control::MujocoSystem::export_state_interfaces();

    imu_state_interfaces_.emplace_back(imu_sensor_name_, "orientation.x", &imu_state_[0]);
    imu_state_interfaces_.emplace_back(imu_sensor_name_, "orientation.y", &imu_state_[1]);
    imu_state_interfaces_.emplace_back(imu_sensor_name_, "orientation.z", &imu_state_[2]);
    imu_state_interfaces_.emplace_back(imu_sensor_name_, "orientation.w", &imu_state_[3]);
    imu_state_interfaces_.emplace_back(imu_sensor_name_, "angular_velocity.x", &imu_state_[4]);
    imu_state_interfaces_.emplace_back(imu_sensor_name_, "angular_velocity.y", &imu_state_[5]);
    imu_state_interfaces_.emplace_back(imu_sensor_name_, "angular_velocity.z", &imu_state_[6]);
    imu_state_interfaces_.emplace_back(imu_sensor_name_, "linear_acceleration.x", &imu_state_[7]);
    imu_state_interfaces_.emplace_back(imu_sensor_name_, "linear_acceleration.y", &imu_state_[8]);
    imu_state_interfaces_.emplace_back(imu_sensor_name_, "linear_acceleration.z", &imu_state_[9]);

    interfaces.reserve(interfaces.size() + imu_state_interfaces_.size());
    for (auto& interface : imu_state_interfaces_) {
        interfaces.emplace_back(std::move(interface));
    }
    return interfaces;
}

hardware_interface::return_type MujocoSimSystem::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    spin_imu_subscription();
    return mujoco_ros2_control::MujocoSystem::read(time, period);
}

void MujocoSimSystem::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_state_[0] = msg->orientation.x;
    imu_state_[1] = msg->orientation.y;
    imu_state_[2] = msg->orientation.z;
    imu_state_[3] = msg->orientation.w;
    imu_state_[4] = msg->angular_velocity.x;
    imu_state_[5] = msg->angular_velocity.y;
    imu_state_[6] = msg->angular_velocity.z;
    imu_state_[7] = msg->linear_acceleration.x;
    imu_state_[8] = msg->linear_acceleration.y;
    imu_state_[9] = msg->linear_acceleration.z;
}

void MujocoSimSystem::spin_imu_subscription() {
    if (imu_node_) {
        imu_executor_.spin_some();
    }
}

}  // namespace car_controller

PLUGINLIB_EXPORT_CLASS(car_controller::MujocoSimSystem, mujoco_ros2_control::MujocoSystemInterface)
