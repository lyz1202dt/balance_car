#pragma once

#include "tools/leg_calc.hpp"
#include <array>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <robot_interfaces/msg/robot_state.hpp>
#include <robot_interfaces/msg/robot_target.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <Eigen/Dense>

namespace lqr_controller {

class LQRController : public controller_interface::ControllerInterface {
public:
    LQRController();

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
    void update_motor_commands(const rclcpp::Time& time,const rclcpp::Duration& period);
    void publish_robot_state();
    bool load_default_pd_gains();
    void imu_callback(const sensor_msgs::msg::Imu& msg);
    double clamp_torque(double value) const;
    bool use_mujoco_sim_chain() const;
    bool use_sim_time_parameter() const;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_exp_vel;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    std::array<std::string, kMotorCount> motor_joint_names_{};
    std::array<MotorSample, kMotorCount> motor_state_{};

    ImuSample imu_state_;
    robot_interfaces::msg::RobotState robot_state_;
    robot_interfaces::msg::RobotTarget robot_target_;
    geometry_msgs::msg::Twist expected_velocity_;

    double torque_limit_{20.0};
    std::array<double, kMotorCount> default_kp_{};
    std::array<double, kMotorCount> default_kd_{};
    std::string imu_topic_{"/imu_imu_sensor/imu"};
    bool use_mujoco_sim_chain_{false};


    //LQR自动控制相关的变量
    LegCalc leg;
};

}  // namespace lqr_controller
