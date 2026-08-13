#pragma once

#include "controller/controller_params.hpp"
#include "controller/lqr_gain_scheduler.hpp"
#include "tools/leg_calc.hpp"
#include "tools/pid.hpp"
#include <array>
#include <functional>
#include <string>
#include <unordered_map>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <robot_interfaces/msg/motor_state.hpp>
#include <robot_interfaces/msg/motor_target.hpp>
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
    static constexpr size_t kTargetInterfacesPerMotor = 5;

    struct MotorInterfaceBinding {
        std::reference_wrapper<robot_interfaces::msg::MotorState> state;
        std::reference_wrapper<robot_interfaces::msg::MotorTarget> target;
        std::array<size_t, kStateInterfacesPerMotor> state_interface_indices;
        std::array<size_t, kTargetInterfacesPerMotor> command_interface_indices;
    };

    void update_motor_commands(const rclcpp::Time& time,const rclcpp::Duration& period);
    void imu_pose_callback(const geometry_msgs::msg::PoseStamped& msg);
    void imu_callback(const sensor_msgs::msg::Imu& msg);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_exp_vel_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr imu_pose_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    sensor_msgs::msg::Imu imu_state_;
    robot_interfaces::msg::RobotState robot_state_;
    robot_interfaces::msg::RobotTarget robot_target_;
    geometry_msgs::msg::Twist expected_velocity_;
    std::unordered_map<std::string, MotorInterfaceBinding> motor_interface_map_;

    //控制器参数
    BalanceControllerParams params_;

    //状态切换
    int state{1};
    rclcpp::Time last_state_switch_time;

    //机器人实际输出的期望命令
    double exp_pos{0.0};
    double exp_omega{0.0};
    double exp_roll{0.0};
    

    //ROLL轴腿长控制
    LegCalc leg;
    LqrGainScheduler gain_scheduler_;
    PID left_leg_length_pid_;
    PID right_leg_length_pid_;
    PID leg_angle_diff_pid_;
    PID wheel_diff_pid_;


    std::array<double, 3> state_velocity_filtered_{};
    std::array<bool, 3> state_velocity_filter_initialized_{};
    double lateral_accel_filtered_{0.0};
    bool lateral_accel_filter_initialized_{false};
};

}  // namespace lqr_controller
