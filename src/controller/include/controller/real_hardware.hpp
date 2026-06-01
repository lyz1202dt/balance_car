#pragma once

#include <array>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace car_controller {

class RealHardware : public hardware_interface::SystemInterface {
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    struct JointData {
        std::string name;
        double position{0.0};
        double velocity{0.0};
        double effort{0.0};
        double effort_command{0.0};
    };

    static constexpr size_t kImuInterfaceCount = 10;

    std::vector<JointData> joints_;
    std::array<double, kImuInterfaceCount> imu_state_{};

    std::vector<hardware_interface::StateInterface> state_interfaces_;
    std::vector<hardware_interface::CommandInterface> command_interfaces_;

    std::string imu_sensor_name_{"imu"};
};

}  // namespace car_controller
