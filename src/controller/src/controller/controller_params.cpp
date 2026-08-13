#include <controller/controller_params.hpp>

#include <rclcpp/exceptions.hpp>

namespace lqr_controller {

namespace {

template <typename T>
void declare_parameter_if_missing(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& name, const T& default_value) {
    if (!node->has_parameter(name)) {
        node->declare_parameter<T>(name, default_value);
    }
}

} // namespace

void declare_balance_controller_parameters(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const BalanceControllerParams& defaults) {
    declare_parameter_if_missing(node, "imu_topic", defaults.imu_topic);
    declare_parameter_if_missing(node, "imu_pose_topic", defaults.imu_pose_topic);

    declare_parameter_if_missing(node, "body_width", defaults.body_width);
    declare_parameter_if_missing(node, "base_link_com_height", defaults.base_link_com_height);
    declare_parameter_if_missing(node, "centrifugal_accel_filter_alpha", defaults.centrifugal_accel_filter_alpha);
    declare_parameter_if_missing(node, "centrifugal_force_ff_gain", defaults.centrifugal_force_ff_gain);
    declare_parameter_if_missing(node, "centrifugal_force_ff_limit", defaults.centrifugal_force_ff_limit);

    declare_parameter_if_missing(node, "leg_exp_length", defaults.leg_exp_length);
    declare_parameter_if_missing(node, "vmc_kp", defaults.vmc_kp);
    declare_parameter_if_missing(node, "vmc_kd", defaults.vmc_kd);

    declare_parameter_if_missing(node, "leg_angle_diff_kp", defaults.leg_angle_diff_kp);
    declare_parameter_if_missing(node, "leg_angle_diff_kd", defaults.leg_angle_diff_kd);

    declare_parameter_if_missing(node, "wheel_diff_kp", defaults.wheel_diff_kp);
    declare_parameter_if_missing(node, "wheel_diff_ki", defaults.wheel_diff_ki);
}

void load_balance_controller_parameters(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, BalanceControllerParams& params) {
    params.imu_topic      = node->get_parameter("imu_topic").as_string();
    params.imu_pose_topic = node->get_parameter("imu_pose_topic").as_string();

    params.body_width                     = node->get_parameter("body_width").as_double();
    params.base_link_com_height           = node->get_parameter("base_link_com_height").as_double();
    params.centrifugal_accel_filter_alpha = node->get_parameter("centrifugal_accel_filter_alpha").as_double();
    params.centrifugal_force_ff_gain      = node->get_parameter("centrifugal_force_ff_gain").as_double();
    params.centrifugal_force_ff_limit     = node->get_parameter("centrifugal_force_ff_limit").as_double();

    params.leg_exp_length = node->get_parameter("leg_exp_length").as_double();
    params.vmc_kp         = node->get_parameter("vmc_kp").as_double();
    params.vmc_kd         = node->get_parameter("vmc_kd").as_double();

    params.leg_angle_diff_kp = node->get_parameter("leg_angle_diff_kp").as_double();
    params.leg_angle_diff_kd = node->get_parameter("leg_angle_diff_kd").as_double();

    params.wheel_diff_kp = node->get_parameter("wheel_diff_kp").as_double();
    params.wheel_diff_ki = node->get_parameter("wheel_diff_ki").as_double();
}

rcl_interfaces::msg::SetParametersResult update_balance_controller_parameters(
    BalanceControllerParams& params, const std::vector<rclcpp::Parameter>& updates) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    auto next = params;
    try {
        for (const auto& param : updates) {
            const auto& name = param.get_name();
            if (name == "imu_topic") {
                next.imu_topic = param.as_string();
            } else if (name == "imu_pose_topic") {
                next.imu_pose_topic = param.as_string();
            }else if (name == "body_width") {
                next.body_width = param.as_double();
            } else if (name == "base_link_com_height") {
                next.base_link_com_height = param.as_double();
            } else if (name == "centrifugal_accel_filter_alpha") {
                next.centrifugal_accel_filter_alpha = param.as_double();
            } else if (name == "centrifugal_force_ff_gain") {
                next.centrifugal_force_ff_gain = param.as_double();
            } else if (name == "centrifugal_force_ff_limit") {
                next.centrifugal_force_ff_limit = param.as_double();
            } else if (name == "leg_exp_length") {
                next.leg_exp_length = param.as_double();
            } else if (name == "vmc_kp") {
                next.vmc_kp = param.as_double();
            } else if (name == "vmc_kd") {
                next.vmc_kd = param.as_double();
            } else if (name == "leg_angle_diff_kp") {
                next.leg_angle_diff_kp = param.as_double();
            } else if (name == "leg_angle_diff_kd") {
                next.leg_angle_diff_kd = param.as_double();
            } else if (name == "wheel_diff_kp") {
                next.wheel_diff_kp = param.as_double();
            } else if (name == "wheel_diff_ki") {
                next.wheel_diff_ki = param.as_double();
            }
        }
    } catch (const rclcpp::ParameterTypeException& ex) {
        result.successful = false;
        result.reason     = ex.what();
        return result;
    }

    params = next;
    return result;
}

} // namespace lqr_controller
