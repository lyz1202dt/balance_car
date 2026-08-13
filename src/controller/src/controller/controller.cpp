#include <controller/controller.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <robot_interfaces/msg/motor_state.hpp>
#include <robot_interfaces/msg/motor_target.hpp>

using namespace std::chrono_literals;

namespace lqr_controller {

namespace {

constexpr size_t kMotorCount             = 6;
constexpr const char* kReferencePrefix   = "mujoco_sim_controller";
constexpr double kWheelRadius            = 0.1;
constexpr double kWheelSpinIntegralLimit = 20.0;
constexpr double kBaselinkMass           = 2.30;
constexpr double kNoPidOutputLimit       = std::numeric_limits<double>::infinity();

constexpr std::array<const char*, kMotorCount> kMotorJointNames = {
    "left_front_hip_joint", "left_rear_hip_joint", "left_wheel_joint", "right_front_hip_joint", "right_rear_hip_joint", "right_wheel_joint",
};

double normalize_angle(const double value) {
    double angle = value;
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double low_pass_filter(const double input, const double alpha, double& filtered_value, bool& initialized) {
    if (!std::isfinite(input)) {
        return initialized ? filtered_value : 0.0;
    }

    if (!initialized) {
        filtered_value = input;
        initialized    = true;
        return filtered_value;
    }

    const double clamped_alpha = std::clamp(std::isfinite(alpha) ? alpha : 1.0, 0.0, 1.0);
    filtered_value             = (1.0 - clamped_alpha) * filtered_value + clamped_alpha * input;
    return filtered_value;
}

bool get_numeric_array_parameter(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const char* name, std::vector<double>& values, std::string& error) {
    rclcpp::Parameter param;
    if (!node->get_parameter(name, param)) {
        error = std::string("Missing parameter ") + name;
        return false;
    }

    if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        values = param.as_double_array();
        return true;
    }
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        const auto int_values = param.as_integer_array();
        values.clear();
        values.reserve(int_values.size());
        for (const auto value : int_values) {
            values.push_back(static_cast<double>(value));
        }
        return true;
    }

    error = param.get_name() + " must be a numeric array";
    return false;
}

} // namespace

LQRController::LQRController()
    : leg() {
    imu_state_.orientation.w = 1.0;


    motor_interface_map_.clear();
    const std::array<std::tuple<const char*, robot_interfaces::msg::MotorState*, robot_interfaces::msg::MotorTarget*>, kMotorCount>
        bindings = {
            {
             {kMotorJointNames[0], &robot_state_.l1, &robot_target_.l1},
             {kMotorJointNames[1], &robot_state_.l2, &robot_target_.l2},
             {kMotorJointNames[2], &robot_state_.lw, &robot_target_.lw},
             {kMotorJointNames[3], &robot_state_.r1, &robot_target_.r1},
             {kMotorJointNames[4], &robot_state_.r2, &robot_target_.r2},
             {kMotorJointNames[5], &robot_state_.rw, &robot_target_.rw},
             }
    };
    for (size_t motor_index = 0; motor_index < bindings.size(); ++motor_index) {
        const auto& [joint_name, state, target] = bindings[motor_index];
        motor_interface_map_.emplace(
            joint_name, MotorInterfaceBinding{
                            *state,
                            *target,
                            {motor_index * kStateInterfacesPerMotor, motor_index * kStateInterfacesPerMotor + 1,
                                    motor_index * kStateInterfacesPerMotor + 2},
                            {motor_index * kTargetInterfacesPerMotor, motor_index * kTargetInterfacesPerMotor + 1,
                                    motor_index * kTargetInterfacesPerMotor + 2, motor_index * kTargetInterfacesPerMotor + 3,
                                    motor_index * kTargetInterfacesPerMotor + 4},
        });
    }
}

controller_interface::CallbackReturn LQRController::on_init() {
    auto node = get_node();
    declare_balance_controller_parameters(node, params_);
    auto_declare<std::vector<double>>("K.lengths", std::vector<double>{});
    auto_declare<std::vector<double>>("K.values", std::vector<double>{});

    std::string error;
    std::vector<double> lengths;
    std::vector<double> values;
    if (!get_numeric_array_parameter(node, "K.lengths", lengths, error) || !get_numeric_array_parameter(node, "K.values", values, error)
        || !gain_scheduler_.load_table(lengths, values, error)) {
        RCLCPP_ERROR(node->get_logger(), "Invalid K table parameters: %s", error.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }

    last_state_switch_time = get_node()->get_clock()->now();

    left_leg_length_pid_.set_gains(params_.vmc_kp, params_.vmc_kd, 0.0);
    right_leg_length_pid_.set_gains(params_.vmc_kp, params_.vmc_kd, 0.0);
    leg_angle_diff_pid_.set_gains(params_.leg_angle_diff_kp, params_.leg_angle_diff_kd, 0.0);
    wheel_diff_pid_.set_gains(params_.wheel_diff_kp, 0.0, params_.wheel_diff_ki);

    left_leg_length_pid_.set_limits(kNoPidOutputLimit, kNoPidOutputLimit);
    right_leg_length_pid_.set_limits(kNoPidOutputLimit, kNoPidOutputLimit);
    leg_angle_diff_pid_.set_limits(kNoPidOutputLimit, kNoPidOutputLimit);
    wheel_diff_pid_.set_limits(kWheelSpinIntegralLimit, kNoPidOutputLimit);

    param_cb_ = node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        auto result = update_balance_controller_parameters(params_, params);
        return result;
    });

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LQRController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    load_balance_controller_parameters(get_node(), params_);

    robot_exp_vel_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](const geometry_msgs::msg::Twist& msg) { expected_velocity_ = msg; });
    imu_pose_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        params_.imu_pose_topic, rclcpp::SensorDataQoS(), [this](const geometry_msgs::msg::PoseStamped& msg) { imu_pose_callback(msg); });
    imu_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
        params_.imu_topic, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu& msg) { imu_callback(msg); });

    for (auto& [name, binding] : motor_interface_map_) {
        (void)name;
        auto& target  = binding.target.get();
        target.rad    = 0.0F;
        target.omega  = 0.0F;
        target.torque = 0.0F;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LQRController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    state_velocity_filtered_.fill(0.0);
    state_velocity_filter_initialized_.fill(false);
    lateral_accel_filtered_           = 0.0;
    lateral_accel_filter_initialized_ = false;
    left_leg_length_pid_.reset();
    right_leg_length_pid_.reset();
    leg_angle_diff_pid_.reset();
    wheel_diff_pid_.reset();

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LQRController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LQRController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    for (auto& [joint_name, binding] : motor_interface_map_) {
        const auto& indices = binding.state_interface_indices;
        auto& state         = binding.state.get();
        state.rad           = static_cast<float>(state_interfaces_[indices[0]].get_value());
        state.omega         = static_cast<float>(state_interfaces_[indices[1]].get_value());
        state.torque        = static_cast<float>(state_interfaces_[indices[2]].get_value());
    }

    update_motor_commands(time, period);

    for (auto& [joint_name, binding] : motor_interface_map_) {
        const auto& indices = binding.command_interface_indices;
        const auto& target  = binding.target.get();
        command_interfaces_[indices[0]].set_value(static_cast<double>(target.rad));
        command_interfaces_[indices[1]].set_value(static_cast<double>(target.omega));
        command_interfaces_[indices[2]].set_value(static_cast<double>(target.torque));
        command_interfaces_[indices[3]].set_value(static_cast<double>(target.kp));
        command_interfaces_[indices[4]].set_value(static_cast<double>(target.kd));
    }

    // 清零控制命令
    robot_target_ = robot_interfaces::msg::RobotTarget{};
    return controller_interface::return_type::OK;
}

void LQRController::update_motor_commands(const rclcpp::Time& time, const rclcpp::Duration& period) {
    Eigen::Vector2d u;

    // 提取机体姿态，Eigen 返回顺序为 [yaw, pitch, roll]。
    Eigen::Quaterniond q;
    q.w() = imu_state_.orientation.w;
    q.x() = imu_state_.orientation.x;
    q.y() = imu_state_.orientation.y;
    q.z() = imu_state_.orientation.z;
    q.normalize();
    const Eigen::Matrix3d R   = q.toRotationMatrix();
    const Eigen::Vector3d ypr = R.eulerAngles(2, 1, 0);
    const double yaw          = ypr[0];
    const double pitch        = ypr[1];
    const double roll         = ypr[2];
    // RCLCPP_INFO_THROTTLE(get_node()->get_logger(),*get_node()->get_clock(),100,"(r=%.4f,p=%.4f,y=%.4f)",rpy[0],rpy[1],rpy[2]);

    const Eigen::Vector3d imu_accel(imu_state_.linear_acceleration.x, imu_state_.linear_acceleration.y, imu_state_.linear_acceleration.z);
    const Eigen::Matrix3d body_to_world  = R;
    const Eigen::Matrix3d level_to_world = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d level_accel    = level_to_world.transpose() * body_to_world * imu_accel;
    const double lateral_accel           = low_pass_filter(
        level_accel.y(), params_.centrifugal_accel_filter_alpha, lateral_accel_filtered_, lateral_accel_filter_initialized_);

    // 提取等效摆的角度和长度
    Eigen::Vector2d left_joint_pos(robot_state_.l1.rad, robot_state_.l2.rad), left_leg_pos(0.0, 0.0);
    const bool left_leg_ok = leg.forward_kinematics(left_joint_pos, left_leg_pos);
    Eigen::Vector2d left_joint_vel(robot_state_.l1.omega, robot_state_.l2.omega), left_leg_vel(0.0, 0.0);
    if (left_leg_ok) {
        leg.forward_velocity(left_joint_pos, left_joint_vel, left_leg_vel);
    }

    Eigen::Vector2d right_joint_pos(robot_state_.r1.rad, robot_state_.r2.rad), right_leg_pos(0.0, 0.0);
    const bool right_leg_ok = leg.forward_kinematics(right_joint_pos, right_leg_pos);
    Eigen::Vector2d right_joint_vel(robot_state_.r1.omega, robot_state_.r2.omega), right_leg_vel(0.0, 0.0);
    if (right_leg_ok) {
        leg.forward_velocity(right_joint_pos, right_joint_vel, right_leg_vel);
    }

    // 准备状态空间方程相关数据
    const double leg_angle          = 0.5 * (left_leg_pos[1] + right_leg_pos[1]);
    const double leg_angle_vel      = 0.5 * (left_leg_vel[1] + right_leg_vel[1]);
    const double average_leg_length = 0.5 * (left_leg_pos[0] + right_leg_pos[0]);
    double x                        = 0.5 * (robot_state_.lw.rad + robot_state_.rw.rad) * kWheelRadius;
    double dx                       = 0.5 * (robot_state_.lw.omega + robot_state_.rw.omega) * kWheelRadius;
    double theta                    = normalize_angle(pitch + leg_angle);
    double dtheta                   = imu_state_.angular_velocity.y + leg_angle_vel; // 将平均腿摆速度加入解算
    double phi                      = -pitch;
    double dphi                     = -imu_state_.angular_velocity.y;

    dx     = low_pass_filter(dx, 0.07, state_velocity_filtered_[0], state_velocity_filter_initialized_[0]);
    dtheta = low_pass_filter(dtheta, 0.7, state_velocity_filtered_[1], state_velocity_filter_initialized_[1]);
    dphi   = low_pass_filter(dphi, 0.7, state_velocity_filtered_[2], state_velocity_filter_initialized_[2]);

    // 支撑力计算
    Eigen::Vector2d left_leg_force, right_leg_force;
    left_leg_force.setZero();
    right_leg_force.setZero();
    leg.forward_dynamics(left_joint_pos, Eigen::Vector2d(robot_state_.l1.torque, robot_state_.l2.torque), left_leg_force);
    leg.forward_dynamics(right_joint_pos, Eigen::Vector2d(robot_state_.r1.torque, robot_state_.r2.torque), right_leg_force);


    // 评估机器人状态，选择控制策略
    if (pitch > 0.3 || pitch < -0.3) {                                // 姿态极大倾斜，系统已经失稳
        if (time - last_state_switch_time > 300ms)                    // 上一次状态切换在300ms前
        {
            if (state != 1)
                last_state_switch_time = time;

            state = 1;
        }
    } else if (left_leg_force[0] > 8.0 && right_leg_force[0] > 8.0) { // 左右腿与地面的压力都要大于8N，才认为可控
        if (time - last_state_switch_time > 300ms)                    // 上一次状态切换在300ms前
        {
            if (state != 2)
                last_state_switch_time = time;
            state = 2;
        }
    } else {
        if (time - last_state_switch_time > 300ms)                    // 上一次状态切换在300ms前
        {
            if (state != 3)
                last_state_switch_time = time;
            state = 3;
        }
    }

    if (!gain_scheduler_.update(average_leg_length)) {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "Failed to update K from leg length");
        u.setZero();
        return;
    }

    if (state == 0) {
        u.setZero();
    } else if (state == 1) {
        u.setZero();
    } else if (state == 2) {
        LqrStateVector X, exp_X;
        exp_X.setZero();

        if (exp_pos - x > 5.0)                                        // 防止x数值爆炸
            exp_X[0] = x + 5.0;
        else if (exp_pos - x < -5.0)
            exp_X[0] = x - 5.0;
        else
            exp_X[0] = exp_pos;

        X << x, dx, theta, dtheta, phi, dphi;                         // 填写状态向量

        u = gain_scheduler_.ground_gain() * (exp_X - X);
    } else if (state == 3) {
        LqrStateVector X;
        X << x, dx, theta, dtheta, phi, dphi;                         // 填写状态向量
        u = -gain_scheduler_.air_gain() * X;
    }


    // 腿长VMC部分，计算关节为了维持当前腿长所需要施加的力矩
    const double safe_body_width = std::max(std::abs(params_.body_width), 1.0e-6);
    const double leg_height_roll = std::atan2(right_leg_pos[0] - left_leg_pos[0], safe_body_width);
    const double combined_roll   = normalize_angle(roll + leg_height_roll);
    const double roll_error      = normalize_angle(exp_roll - combined_roll);
    const double leg_length_diff = safe_body_width * std::tan(roll_error);
    double left_leg_exp_length   = std::clamp(params_.leg_exp_length + 0.5 * leg_length_diff, 0.19, 0.34);
    double right_leg_exp_length  = std::clamp(params_.leg_exp_length - 0.5 * leg_length_diff, 0.19, 0.34);

    double vmc_mass_component = state == 2 ? (kBaselinkMass * 0.5 * 9.8 * cos(theta)) : 0.0;
    const double com_height   = average_leg_length + kWheelRadius + params_.base_link_com_height;
    const double centrifugal_force_ff_raw =
        state == 2 ? params_.centrifugal_force_ff_gain * kBaselinkMass * lateral_accel * com_height / safe_body_width : 0.0;
    const double centrifugal_force_ff_limit = std::max(0.0, params_.centrifugal_force_ff_limit);
    const double centrifugal_force_ff       = std::clamp(centrifugal_force_ff_raw, -centrifugal_force_ff_limit, centrifugal_force_ff_limit);
    double left_leg_dis_vmc_T = left_leg_length_pid_.update(left_leg_pos[0], left_leg_vel[0], left_leg_exp_length) + vmc_mass_component;
    double right_leg_dis_vmc_T =
        right_leg_length_pid_.update(right_leg_pos[0], right_leg_vel[0], right_leg_exp_length) + vmc_mass_component;
    if (state == 2) {
        left_leg_dis_vmc_T -= centrifugal_force_ff;
        right_leg_dis_vmc_T += centrifugal_force_ff;
    }

    const double leg_angle_diff     = normalize_angle(left_leg_pos[1] - right_leg_pos[1]);
    const double leg_angle_diff_vel = left_leg_vel[1] - right_leg_vel[1];
    const double spin_omega         = imu_state_.angular_velocity.z;
    double spin_control_torque_diff = 0.0;
    if (state == 2) {
        spin_control_torque_diff = wheel_diff_pid_.update_with_dt(spin_omega, exp_omega, period.seconds());
    } else {
        wheel_diff_pid_.reset();
    }
    const double spin_compensation_torque = -spin_control_torque_diff * average_leg_length / kWheelRadius;
    const double leg_angle_sync_torque    = leg_angle_diff_pid_.update(leg_angle_diff, leg_angle_diff_vel, 0.0) + spin_compensation_torque;
    const double left_leg_angle_torque    = u[1] + leg_angle_sync_torque;
    const double right_leg_angle_torque   = u[1] - leg_angle_sync_torque;
    const double left_wheel_torque        = u[0] - spin_control_torque_diff;
    const double right_wheel_torque       = u[0] + spin_control_torque_diff;

    // VMC映射
    Eigen::Vector2d left_torque, right_torque;
    leg.inverse_dynamics(left_joint_pos, Eigen::Vector2d(left_leg_dis_vmc_T, left_leg_angle_torque), left_torque);
    leg.inverse_dynamics(right_joint_pos, Eigen::Vector2d(right_leg_dis_vmc_T, right_leg_angle_torque), right_torque);

    // 设置各关节期望力矩
    robot_target_.l1.torque = std::clamp(static_cast<float>(left_torque[0]), -12.0f, 12.0f);
    robot_target_.l2.torque = std::clamp(static_cast<float>(left_torque[1]), -12.0f, 12.0f);
    robot_target_.r1.torque = std::clamp(static_cast<float>(right_torque[0]), -12.0f, 12.0f);
    robot_target_.r2.torque = std::clamp(static_cast<float>(right_torque[1]), -12.0f, 12.0f);
    robot_target_.lw.torque = std::clamp(static_cast<float>(left_wheel_torque), -2.0f, 2.0f);
    robot_target_.rw.torque = std::clamp(static_cast<float>(right_wheel_torque), -2.0f, 2.0f);


    RCLCPP_INFO_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 100,
        "state=%d\npos=%.4f\nroll=%.4f\nleg_roll=%.4f\ncombined_roll=%.4f\nay=%.4f\nay_raw=%.4f\ncentrifugal_ff=%.4f\nF=(%.4f,%.4f)\nu:(T=%"
        ".4f,Tp=%.4f)\nlength=(%.4f,%.4f)\nexp_length=(%.4f,%.4f)",
        state, x, roll, leg_height_roll, combined_roll, lateral_accel, level_accel.y(), centrifugal_force_ff, left_leg_force[0],
        right_leg_force[0], u[0], u[1], left_leg_pos[0], right_leg_pos[0], left_leg_exp_length, right_leg_exp_length);
}

void LQRController::imu_pose_callback(const geometry_msgs::msg::PoseStamped& msg) {
    imu_state_.orientation.x = msg.pose.orientation.x;
    imu_state_.orientation.y = msg.pose.orientation.y;
    imu_state_.orientation.z = msg.pose.orientation.z;
    imu_state_.orientation.w = msg.pose.orientation.w;
}

void LQRController::imu_callback(const sensor_msgs::msg::Imu& msg) {
    imu_state_.angular_velocity    = msg.angular_velocity;
    imu_state_.linear_acceleration = msg.linear_acceleration;
}

controller_interface::InterfaceConfiguration LQRController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& joint_name : kMotorJointNames) {
        const auto name = std::string(joint_name);
        if (get_node()->get_parameter("use_sim_time").as_bool()) {
            const auto prefix = std::string(kReferencePrefix) + "/" + name;
            cfg.names.push_back(prefix + "/position");
            cfg.names.push_back(prefix + "/velocity");
            cfg.names.push_back(prefix + "/effort");
            cfg.names.push_back(prefix + "/kp");
            cfg.names.push_back(prefix + "/kd");
        } else {
            cfg.names.push_back(name + "/position");
            cfg.names.push_back(name + "/velocity");
            cfg.names.push_back(name + "/effort");
            cfg.names.push_back(name + "/kp");
            cfg.names.push_back(name + "/kd");
        }
    }
    return cfg;
}

controller_interface::InterfaceConfiguration LQRController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& joint_name : kMotorJointNames) {
        const auto name = std::string(joint_name);
        cfg.names.push_back(name + "/position");
        cfg.names.push_back(name + "/velocity");
        cfg.names.push_back(name + "/effort");
    }

    return cfg;
}


} // namespace lqr_controller

PLUGINLIB_EXPORT_CLASS(lqr_controller::LQRController, controller_interface::ControllerInterface)
