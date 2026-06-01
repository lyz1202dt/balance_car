#pragma once

#include <array>
#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <mujoco_ros2_control/mujoco_system.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace car_controller {

class MujocoSimSystem : public mujoco_ros2_control::MujocoSystem {
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    static constexpr size_t kImuInterfaceCount = 10;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void spin_imu_subscription();

    std::array<double, kImuInterfaceCount> imu_state_{};
    std::vector<hardware_interface::StateInterface> imu_state_interfaces_;

    rclcpp::Node::SharedPtr imu_node_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::executors::SingleThreadedExecutor imu_executor_;

    std::string imu_sensor_name_{"imu"};
    std::string imu_topic_{"/imu_imu_sensor/imu"};
};

}  // namespace car_controller
