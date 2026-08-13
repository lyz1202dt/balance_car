#pragma once

#include <string>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace lqr_controller {

struct BalanceControllerParams {
    std::string imu_topic{"/imu_imu_sensor/imu"};
    std::string imu_pose_topic{"/imu_pose_sensor/pose"};

    double body_width{0.34};
    double base_link_com_height{0.1265};
    double centrifugal_accel_filter_alpha{0.2};
    double centrifugal_force_ff_gain{1.0};
    double centrifugal_force_ff_limit{40.0};

    double leg_exp_length{0.28};
    double vmc_kp{600.0};
    double vmc_kd{40.0};

    double leg_angle_diff_kp{30.0};
    double leg_angle_diff_kd{4.0};

    double wheel_diff_kp{0.1};
    double wheel_diff_ki{0.002};
};

void declare_balance_controller_parameters(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const BalanceControllerParams& defaults);

void load_balance_controller_parameters(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, BalanceControllerParams& params);

rcl_interfaces::msg::SetParametersResult update_balance_controller_parameters(
    BalanceControllerParams& params, const std::vector<rclcpp::Parameter>& updates);

} // namespace lqr_controller
