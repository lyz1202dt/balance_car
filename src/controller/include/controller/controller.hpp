#pragma once

#include "tools/leg_calc.hpp"
#include <array>
#include <string>
#include <utility>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <robot_interfaces/msg/robot_state.hpp>
#include <robot_interfaces/msg/robot_target.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <Eigen/Dense>

namespace lqr_controller {

using LqrGainMatrix = Eigen::Matrix<double, 2, 6>;
using KTableEntry = std::pair<double, LqrGainMatrix>;

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
    void update_motor_commands(const rclcpp::Time& time,const rclcpp::Duration& period);
    void imu_pose_callback(const geometry_msgs::msg::PoseStamped& msg);
    void imu_callback(const sensor_msgs::msg::Imu& msg);
    double clamp_torque(double value) const;
    bool use_mujoco_sim_chain() const;
    bool use_sim_time_parameter() const;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_exp_vel;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr imu_pose_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    sensor_msgs::msg::Imu imu_state_;
    robot_interfaces::msg::RobotState robot_state_;
    robot_interfaces::msg::RobotTarget robot_target_;
    geometry_msgs::msg::Twist expected_velocity_;

    double torque_limit_{20.0};
    std::string imu_topic_{"/imu_imu_sensor/imu"};
    std::string imu_pose_topic_{"/imu_pose_sensor/pose"};
    bool use_mujoco_sim_chain_{false};


    //LQR自动控制相关
    int state{1};
    rclcpp::Time last_state_switch_time;

    bool load_K_table(std::string& error);
    bool update_K(double leg_length);
    LegCalc leg;
    std::vector<KTableEntry> k_table_;
    LqrGainMatrix K;                //K矩阵（控制反馈增益矩阵）
    LqrGainMatrix air_K;
    Eigen::Vector2d u;              //控制向量
    double exp_x{0.0};              //期望位置
    double exp_omega{0.0};          //期望自旋角速度
    double exp_roll{0.0};           //期望roll角

    //ROLL轴腿长控制
    double body_width_{0.34};
    double leg_exp_length{0.28};
    double left_leg_exp_length{0.28};
    double right_leg_exp_length{0.28};

    //腿长VMC
    double vmc_kp{600.0};           
    double vmc_kd{40.0};

    //腿同步PD控制器
    double leg_angle_diff_kp_{30.0};    
    double leg_angle_diff_kd_{4.0};

    //YAW轴轮速控制器
    double wheel_diff_kp_{0.1};         
    double wheel_diff_ki_{0.002};
    double wheel_spin_error_integral_{0.0};


    std::array<double, 3> state_velocity_filtered_{};
    std::array<bool, 3> state_velocity_filter_initialized_{};
};

}  // namespace lqr_controller
