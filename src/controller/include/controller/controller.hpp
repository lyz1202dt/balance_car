#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <controller_interface/semantic_components/imu_sensor.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <robot_interfaces/msg/robot_state.hpp>
#include <robot_interfaces/msg/robot_target.hpp>

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
    static constexpr size_t kMotorCount = 6;
    static constexpr size_t kStateInterfacesPerMotor = 3;

    struct MotorSample {
        double position{0.0};
        double velocity{0.0};
        double effort{0.0};
    };

    struct ImuSample {
        std::array<double, 4> orientation{0.0, 0.0, 0.0, 1.0};
        std::array<double, 3> angular_velocity{0.0, 0.0, 0.0};
        std::array<double, 3> linear_acceleration{0.0, 0.0, 0.0};
    };

    void read_state_interfaces();
    void update_motor_commands(const rclcpp::Duration& period);
    void publish_robot_state();
    double clamp_torque(double value) const;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_exp_vel;
    rclcpp::Subscription<robot_interfaces::msg::RobotTarget>::SharedPtr target_subscriber_;
    rclcpp_lifecycle::LifecyclePublisher<robot_interfaces::msg::RobotState>::SharedPtr state_publisher_;
    rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    std::array<std::string, kMotorCount> motor_joint_names_{};
    std::array<MotorSample, kMotorCount> motor_state_{};

    std::unique_ptr<semantic_components::IMUSensor> imu_sensor_;
    ImuSample imu_state_;
    robot_interfaces::msg::RobotState robot_state_;
    robot_interfaces::msg::RobotTarget robot_target_;
    geometry_msgs::msg::Twist expected_velocity_;

    double torque_limit_{20.0};
    std::string imu_sensor_name_{"imu"};
    std::string state_topic_{"robot_state"};
    std::string target_topic_{"robot_target"};
    std::string cmd_vel_topic_{"cmd_vel"};
};

}  // namespace car_controller
