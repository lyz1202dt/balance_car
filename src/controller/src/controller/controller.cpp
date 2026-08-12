#include <controller/controller.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <string>
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

constexpr size_t kMotorCount                                    = 6;
constexpr size_t kTargetInterfacesPerMotor                      = 5;
constexpr size_t kLqrStateSize                                  = 6;
constexpr size_t kLqrInputSize                                  = 2;
constexpr size_t kKValuesPerEntry                               = kLqrStateSize * kLqrInputSize;
constexpr const char* kReferencePrefix                          = "mujoco_sim_controller";
constexpr const char* kStateTopic                               = "robot_state";
constexpr const char* kTargetTopic                              = "robot_target";
constexpr const char* kCmdVelTopic                              = "cmd_vel";
constexpr double kHipHalfDistance                               = 0.11;
constexpr double kUpperLinkLength                               = 0.1844;
constexpr double kLowerLinkLength                               = 0.3130;
constexpr double kWheelRadius                                   = 0.1;
constexpr double kWheelSpinIntegralLimit                        = 20.0;
constexpr double kBaselinkMass                                  = 2.30;

constexpr std::array<const char*, kMotorCount> kMotorJointNames = {
    "left_front_hip_joint", "left_rear_hip_joint", "left_wheel_joint", "right_front_hip_joint", "right_rear_hip_joint", "right_wheel_joint",
};
constexpr std::array<const char*, 3> kStateInterfaceNames                          = {"position", "velocity", "effort"};
constexpr std::array<const char*, kTargetInterfacesPerMotor> kTargetInterfaceNames = {
    "position", "velocity", "effort", "kp", "kd",
};

robot_interfaces::msg::MotorState& motor_state_at(robot_interfaces::msg::RobotState& msg, const size_t index) {
    switch (index) {
    case 0: return msg.l1;
    case 1: return msg.l2;
    case 2: return msg.lw;
    case 3: return msg.r1;
    case 4: return msg.r2;
    default: return msg.rw;
    }
}

robot_interfaces::msg::MotorTarget& target_at(robot_interfaces::msg::RobotTarget& msg, const size_t index) {
    switch (index) {
    case 0: return msg.l1;
    case 1: return msg.l2;
    case 2: return msg.lw;
    case 3: return msg.r1;
    case 4: return msg.r2;
    default: return msg.rw;
    }
}

template <typename Interface>
Interface* find_interface(std::vector<Interface>& interfaces, const std::string& name) {
    const auto interface =
        std::find_if(interfaces.begin(), interfaces.end(), [&name](const auto& item) { return item.get_name() == name; });
    if (interface == interfaces.end()) {
        return nullptr;
    }
    return &(*interface);
}

double clamp_unit(const double value) { return std::clamp(value, -1.0, 1.0); }

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

double normalized_zyx_pitch(const Eigen::Quaterniond& q) {
    const Eigen::Matrix3d rotation = q.toRotationMatrix();
    return std::asin(clamp_unit(-rotation(2, 0)));
}

double normalized_zyx_roll(const Eigen::Quaterniond& q) {
    const Eigen::Matrix3d rotation = q.toRotationMatrix();
    return std::atan2(rotation(2, 1), rotation(2, 2));
}

double normalized_zyx_yaw(const Eigen::Quaterniond& q) {
    const Eigen::Matrix3d rotation = q.toRotationMatrix();
    return std::atan2(rotation(1, 0), rotation(0, 0));
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

std::string motor_state_interface_name(const size_t motor_index, const size_t interface_index) {
    return std::string(kMotorJointNames[motor_index]) + "/" + kStateInterfaceNames[interface_index];
}

std::string motor_command_interface_name(const size_t motor_index, const size_t interface_index, const bool use_mujoco_chain) {
    const auto joint_interface = std::string(kMotorJointNames[motor_index]) + "/" + kTargetInterfaceNames[interface_index];
    if (use_mujoco_chain) {
        return std::string(kReferencePrefix) + "/" + joint_interface;
    }
    return joint_interface;
}

bool parameter_to_double_vector(const rclcpp::Parameter& param, std::vector<double>& values, std::string& error) {
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

bool get_numeric_array_parameter(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const char* name, std::vector<double>& values, std::string& error) {
    rclcpp::Parameter param;
    if (!node->get_parameter(name, param)) {
        error = std::string("Missing parameter ") + name;
        return false;
    }

    return parameter_to_double_vector(param, values, error);
}

} // namespace

LQRController::LQRController()
    : leg(kHipHalfDistance, kUpperLinkLength, kLowerLinkLength) {
    imu_state_.orientation.w = 1.0;
    air_K.setZero();
    K.setZero();
}

controller_interface::CallbackReturn LQRController::on_init() {
    auto node = get_node();
    auto_declare<std::string>("imu_topic", imu_topic_);
    auto_declare<std::string>("imu_pose_topic", imu_pose_topic_);
    auto_declare<double>("torque_limit", torque_limit_);
    auto_declare<double>("vmc_kp", vmc_kp);
    auto_declare<double>("vmc_kd", vmc_kd);
    auto_declare<double>("body_width", body_width_);
    auto_declare<double>("base_link_com_height", base_link_com_height_);
    auto_declare<double>("centrifugal_accel_filter_alpha", centrifugal_accel_filter_alpha_);
    auto_declare<double>("centrifugal_force_ff_gain", centrifugal_force_ff_gain_);
    auto_declare<double>("centrifugal_force_ff_limit", centrifugal_force_ff_limit_);
    auto_declare<double>("leg_exp_length", leg_exp_length);
    auto_declare<double>("leg_angle_diff_kp", leg_angle_diff_kp_);
    auto_declare<double>("leg_angle_diff_kd", leg_angle_diff_kd_);
    auto_declare<double>("wheel_diff_kp", wheel_diff_kp_);
    auto_declare<double>("wheel_diff_ki", wheel_diff_ki_);
    auto_declare<std::vector<double>>("K.lengths", std::vector<double>{});
    auto_declare<std::vector<double>>("K.values", std::vector<double>{});

    state     = node->declare_parameter<int>("state", 1);
    exp_x     = node->declare_parameter<double>("exp_pos",0.0);
    exp_omega = node->declare_parameter<double>("exp_omega",0.0);
    exp_roll  = node->declare_parameter<double>("exp_roll",0.0);


    std::string error;
    if (!load_K_table(error)) {
        RCLCPP_ERROR(node->get_logger(), "Invalid K table parameters: %s", error.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }
    update_K(k_table_.front().first);

    last_state_switch_time = get_node()->get_clock()->now();

    param_cb_ = node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        double next_torque_limit    = torque_limit_;
        double next_vmc_kp          = vmc_kp;
        double next_vmc_kd          = vmc_kd;
        double next_body_width      = body_width_;
        double next_base_link_com_height = base_link_com_height_;
        double next_centrifugal_accel_filter_alpha = centrifugal_accel_filter_alpha_;
        double next_centrifugal_force_ff_gain = centrifugal_force_ff_gain_;
        double next_centrifugal_force_ff_limit = centrifugal_force_ff_limit_;
        double next_leg_exp_length  = leg_exp_length;
        double next_leg_diff_kp     = leg_angle_diff_kp_;
        double next_leg_diff_kd     = leg_angle_diff_kd_;
        double next_wheel_diff_kp   = wheel_diff_kp_;
        double next_wheel_diff_ki   = wheel_diff_ki_;
        int next_state              = state;

        for (const auto& param : params) {
            if (param.get_name() == "torque_limit") {
                next_torque_limit = param.as_double();
            } else if (param.get_name() == "vmc_kp") {
                next_vmc_kp = param.as_double();
            } else if (param.get_name() == "vmc_kd") {
                next_vmc_kd = param.as_double();
            } else if (param.get_name() == "body_width") {
                next_body_width = param.as_double();
            } else if (param.get_name() == "base_link_com_height") {
                next_base_link_com_height = param.as_double();
            } else if (param.get_name() == "centrifugal_accel_filter_alpha") {
                next_centrifugal_accel_filter_alpha = param.as_double();
            } else if (param.get_name() == "centrifugal_force_ff_gain") {
                next_centrifugal_force_ff_gain = param.as_double();
            } else if (param.get_name() == "centrifugal_force_ff_limit") {
                next_centrifugal_force_ff_limit = param.as_double();
            } else if (param.get_name() == "leg_exp_length") {
                next_leg_exp_length = param.as_double();
            } else if (param.get_name() == "leg_angle_diff_kp") {
                next_leg_diff_kp = param.as_double();
            } else if (param.get_name() == "leg_angle_diff_kd") {
                next_leg_diff_kd = param.as_double();
            } else if (param.get_name() == "wheel_diff_kp") {
                next_wheel_diff_kp = param.as_double();
            } else if (param.get_name() == "wheel_diff_ki") {
                next_wheel_diff_ki = param.as_double();
            } else if (param.get_name() == "state") {
                next_state = param.as_int();
            }
            else if(param.get_name() =="exp_pos")
            {
                exp_x=param.as_double();
            }
            else if(param.get_name() =="exp_omega")
            {
                exp_omega=param.as_double();
            }
            else if(param.get_name() =="exp_roll")
            {
                exp_roll=param.as_double();
            }
        }

        torque_limit_      = next_torque_limit;
        vmc_kp             = next_vmc_kp;
        vmc_kd             = next_vmc_kd;
        body_width_        = next_body_width;
        base_link_com_height_ = next_base_link_com_height;
        centrifugal_accel_filter_alpha_ = next_centrifugal_accel_filter_alpha;
        centrifugal_force_ff_gain_ = next_centrifugal_force_ff_gain;
        centrifugal_force_ff_limit_ = next_centrifugal_force_ff_limit;
        leg_exp_length     = next_leg_exp_length;
        leg_angle_diff_kp_ = next_leg_diff_kp;
        leg_angle_diff_kd_ = next_leg_diff_kd;
        wheel_diff_kp_     = next_wheel_diff_kp;
        wheel_diff_ki_     = next_wheel_diff_ki;
        state              = next_state;

        return result;
    });

    return controller_interface::CallbackReturn::SUCCESS;
}

bool LQRController::load_K_table(std::string& error) {
    auto node = get_node();
    std::vector<double> lengths;
    std::vector<double> values;
    if (!get_numeric_array_parameter(node, "K.lengths", lengths, error)
        || !get_numeric_array_parameter(node, "K.values", values, error)) {
        return false;
    }

    if (lengths.empty()) {
        error = "K.lengths must contain at least one leg length";
        return false;
    }
    if (values.size() != lengths.size() * kKValuesPerEntry) {
        error = "K.values must contain exactly 12 values for each K.lengths entry";
        return false;
    }

    k_table_.clear();
    k_table_.reserve(lengths.size());
    for (size_t entry_index = 0; entry_index < lengths.size(); ++entry_index) {
        const double length = lengths[entry_index];
        if (!std::isfinite(length)) {
            error = "K.lengths contains a non-finite value at index " + std::to_string(entry_index);
            return false;
        }

        LqrGainMatrix gain;
        for (size_t row = 0; row < kLqrInputSize; ++row) {
            for (size_t col = 0; col < kLqrStateSize; ++col) {
                const size_t value_index = entry_index * kKValuesPerEntry + row * kLqrStateSize + col;
                const double value       = values[value_index];
                if (!std::isfinite(value)) {
                    error = "K.values contains a non-finite value at index " + std::to_string(value_index);
                    return false;
                }
                gain(row, col) = value;
            }
        }
        k_table_.emplace_back(length, gain);
    }

    std::sort(k_table_.begin(), k_table_.end(), [](const KTableEntry& lhs, const KTableEntry& rhs) {
        return lhs.first < rhs.first;
    });

    for (size_t i = 1; i < k_table_.size(); ++i) {
        if (k_table_[i].first <= k_table_[i - 1].first) {
            error = "K.lengths entries must be unique";
            return false;
        }
    }
    return true;
}

bool LQRController::update_K(const double leg_length) {
    if (k_table_.empty() || !std::isfinite(leg_length)) {
        return false;
    }

    if (leg_length <= k_table_.front().first || k_table_.size() == 1) {
        K = k_table_.front().second;
    } else if (leg_length >= k_table_.back().first) {
        K = k_table_.back().second;
    } else {
        const auto upper = std::lower_bound(
            k_table_.begin(), k_table_.end(), leg_length,
            [](const KTableEntry& entry, const double length) { return entry.first < length; });
        const auto lower = std::prev(upper);
        const double ratio = (leg_length - lower->first) / (upper->first - lower->first);
        K                  = (1.0 - ratio) * lower->second + ratio * upper->second;
    }

    air_K.setZero();
    air_K(1, 2) = K(1, 2);
    air_K(1, 3) = K(1, 3);
    return true;
}

controller_interface::CallbackReturn LQRController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    imu_topic_            = get_node()->get_parameter("imu_topic").as_string();
    imu_pose_topic_       = get_node()->get_parameter("imu_pose_topic").as_string();
    use_mujoco_sim_chain_ = use_sim_time_parameter();
    torque_limit_         = get_node()->get_parameter("torque_limit").as_double();
    vmc_kp                = get_node()->get_parameter("vmc_kp").as_double();
    vmc_kd                = get_node()->get_parameter("vmc_kd").as_double();
    body_width_           = get_node()->get_parameter("body_width").as_double();
    base_link_com_height_ = get_node()->get_parameter("base_link_com_height").as_double();
    centrifugal_accel_filter_alpha_ = get_node()->get_parameter("centrifugal_accel_filter_alpha").as_double();
    centrifugal_force_ff_gain_ = get_node()->get_parameter("centrifugal_force_ff_gain").as_double();
    centrifugal_force_ff_limit_ = get_node()->get_parameter("centrifugal_force_ff_limit").as_double();
    leg_exp_length        = get_node()->get_parameter("leg_exp_length").as_double();
    leg_angle_diff_kp_    = get_node()->get_parameter("leg_angle_diff_kp").as_double();
    leg_angle_diff_kd_    = get_node()->get_parameter("leg_angle_diff_kd").as_double();
    wheel_diff_kp_        = get_node()->get_parameter("wheel_diff_kp").as_double();
    wheel_diff_ki_        = get_node()->get_parameter("wheel_diff_ki").as_double();

    robot_exp_vel = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        kCmdVelTopic, 10, [this](const geometry_msgs::msg::Twist& msg) { expected_velocity_ = msg; });
    imu_pose_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        imu_pose_topic_, rclcpp::SensorDataQoS(), [this](const geometry_msgs::msg::PoseStamped& msg) { imu_pose_callback(msg); });
    imu_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu& msg) { imu_callback(msg); });

    for (size_t i = 0; i < kMotorCount; ++i) {
        auto& target  = target_at(robot_target_, i);
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
    lateral_accel_filtered_ = 0.0;
    lateral_accel_filter_initialized_ = false;
    wheel_spin_error_integral_ = 0.0;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LQRController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LQRController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    for (size_t motor_index = 0; motor_index < kMotorCount; ++motor_index) {
        const auto position_interface = find_interface(state_interfaces_, motor_state_interface_name(motor_index, 0));
        const auto velocity_interface = find_interface(state_interfaces_, motor_state_interface_name(motor_index, 1));
        const auto effort_interface   = find_interface(state_interfaces_, motor_state_interface_name(motor_index, 2));
        if (position_interface == nullptr || velocity_interface == nullptr || effort_interface == nullptr) {
            RCLCPP_ERROR_THROTTLE(
                get_node()->get_logger(), *get_node()->get_clock(), 1000, "Missing state interfaces for %s", kMotorJointNames[motor_index]);
            return controller_interface::return_type::ERROR;
        }

        auto& state  = motor_state_at(robot_state_, motor_index);
        state.rad    = static_cast<float>(position_interface->get_value());
        state.omega  = static_cast<float>(velocity_interface->get_value());
        state.torque = static_cast<float>(effort_interface->get_value());
    }

    update_motor_commands(time, period);

    for (size_t i = 0; i < kMotorCount; ++i) {
        const auto position_interface = find_interface(command_interfaces_, motor_command_interface_name(i, 0, use_mujoco_sim_chain()));
        const auto velocity_interface = find_interface(command_interfaces_, motor_command_interface_name(i, 1, use_mujoco_sim_chain()));
        const auto effort_interface   = find_interface(command_interfaces_, motor_command_interface_name(i, 2, use_mujoco_sim_chain()));
        const auto kp_interface       = find_interface(command_interfaces_, motor_command_interface_name(i, 3, use_mujoco_sim_chain()));
        const auto kd_interface       = find_interface(command_interfaces_, motor_command_interface_name(i, 4, use_mujoco_sim_chain()));
        if (position_interface == nullptr || velocity_interface == nullptr || effort_interface == nullptr || kp_interface == nullptr
            || kd_interface == nullptr) {
            RCLCPP_ERROR_THROTTLE(
                get_node()->get_logger(), *get_node()->get_clock(), 1000, "Missing command interfaces for %s", kMotorJointNames[i]);
            return controller_interface::return_type::ERROR;
        }

        const auto& target = target_at(robot_target_, i);
        position_interface->set_value(static_cast<double>(target.rad));
        velocity_interface->set_value(static_cast<double>(target.omega));
        effort_interface->set_value(clamp_torque(static_cast<double>(target.torque)));
        kp_interface->set_value(static_cast<double>(target.kp));
        kd_interface->set_value(static_cast<double>(target.kd));
        // kp_interface->set_value(static_cast<double>(target.kp));
        // kd_interface->set_value(static_cast<double>(target.kd));
    }

    // 清零kp和kd指令
    robot_target_.l1.kp = robot_target_.l2.kp = robot_target_.r1.kp = robot_target_.r2.kp = 0.0;
    robot_target_.l1.kd = robot_target_.l2.kd = robot_target_.r1.kd = robot_target_.r2.kd = 0.0;
    robot_target_.lw.kp = robot_target_.rw.kp = 0.0;
    robot_target_.lw.kd = robot_target_.rw.kd = 0.0;

    // 清零torque指令
    robot_target_.l1.torque = robot_target_.l2.torque = robot_target_.r1.torque = robot_target_.r2.torque = 0.0;
    robot_target_.lw.torque = robot_target_.rw.torque = 0.0;

    return controller_interface::return_type::OK;
}

void LQRController::update_motor_commands(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;

    // 提取机体 pitch。Eigen::eulerAngles(2, 1, 0) 可能把接近直立的姿态表示成 pitch 接近 pi。
    Eigen::Quaterniond q;
    q.w() = imu_state_.orientation.w;
    q.x() = imu_state_.orientation.x;
    q.y() = imu_state_.orientation.y;
    q.z() = imu_state_.orientation.z;
    q.normalize();
    const double pitch = normalized_zyx_pitch(q);
    const double roll  = normalized_zyx_roll(q);
    const double yaw   = normalized_zyx_yaw(q);
    // RCLCPP_INFO_THROTTLE(get_node()->get_logger(),*get_node()->get_clock(),100,"(r=%.4f,p=%.4f,y=%.4f)",rpy[0],rpy[1],rpy[2]);

    const Eigen::Vector3d imu_accel(
        imu_state_.linear_acceleration.x, imu_state_.linear_acceleration.y, imu_state_.linear_acceleration.z);
    const Eigen::Matrix3d body_to_world = q.toRotationMatrix();
    const Eigen::Matrix3d level_to_world = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d level_accel = level_to_world.transpose() * body_to_world * imu_accel;
    const double lateral_accel = low_pass_filter(
        level_accel.y(), centrifugal_accel_filter_alpha_, lateral_accel_filtered_, lateral_accel_filter_initialized_);

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
    const double leg_angle     = 0.5 * (left_leg_pos[1] + right_leg_pos[1]);
    const double leg_angle_vel = 0.5 * (left_leg_vel[1] + right_leg_vel[1]);
    const double average_leg_length = 0.5 * (left_leg_pos[0] + right_leg_pos[0]);
    double x                   = 0.5 * (robot_state_.lw.rad + robot_state_.rw.rad) * kWheelRadius;
    double dx                  = 0.5 * (robot_state_.lw.omega + robot_state_.rw.omega) * kWheelRadius;
    double theta               = normalize_angle(pitch + leg_angle);
    double dtheta              = imu_state_.angular_velocity.y + leg_angle_vel; // 将平均腿摆速度加入解算
    double phi                 = -pitch;
    double dphi                = -imu_state_.angular_velocity.y;

    dx     = low_pass_filter(dx, 0.07, state_velocity_filtered_[0], state_velocity_filter_initialized_[0]);
    dtheta = low_pass_filter(dtheta, 0.7, state_velocity_filtered_[1], state_velocity_filter_initialized_[1]);
    dphi   = low_pass_filter(dphi, 0.7, state_velocity_filtered_[2], state_velocity_filter_initialized_[2]);

    // 支撑力计算
    Eigen::Vector2d left_leg_force, right_leg_force;
    left_leg_force.setZero();
    right_leg_force.setZero();
    leg.forward_dynamics(left_joint_pos, Eigen::Vector2d(robot_state_.l1.torque, robot_state_.l2.torque), left_leg_force);
    leg.forward_dynamics(right_joint_pos, Eigen::Vector2d(robot_state_.r1.torque, robot_state_.r2.torque), right_leg_force);


    //评估机器人状态，选择控制策略
    auto now = get_node()->get_clock()->now();
    if (pitch > 0.3 || pitch < -0.3) {                                // 姿态极大倾斜，系统已经失稳
        if (now - last_state_switch_time > 300ms)                     // 上一次状态切换在300ms前
        {
            if (state != 1)
                last_state_switch_time = now;

            state = 1;
        }
    } else if (left_leg_force[0] > 8.0 && right_leg_force[0] > 8.0) { // 左右腿与地面的压力都要大于8N，才认为可控
        if (now - last_state_switch_time > 300ms)                     // 上一次状态切换在300ms前
        {
            if (state != 2)
                last_state_switch_time = now;
            state = 2;
        }
    } else {
        if (now - last_state_switch_time > 300ms)                     // 上一次状态切换在300ms前
        {
            if (state != 3)
                last_state_switch_time = now;
            state = 3;
        }
    }

    if (!update_K(average_leg_length)) {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "Failed to update K from leg length");
        u.setZero();
        return;
    }

    if(state==0)
    {
        u.setZero();
    }else if(state==1)
    {
        u.setZero();
    }else if(state==2)
    {
        Eigen::Vector<double, 6> X, exp_X;
        exp_X.setZero();

        if(exp_x-x>5.0)         //防止x数值爆炸
            exp_X[0]=x+5.0;
        else if(exp_x-x<-5.0)
            exp_X[0]=x-5.0;
        else
            exp_X[0]=exp_x;

        X << x, dx, theta, dtheta, phi, dphi; //填写状态向量

        u = K * (exp_X - X);
    }
    else if(state==3)
    {
        Eigen::Vector<double, 6> X;
        X << x, dx, theta, dtheta, phi, dphi; //填写状态向量
        u = -air_K*X;
    }

    
    // 腿长VMC部分，计算关节为了维持当前腿长所需要施加的力矩
    const double safe_body_width = std::max(std::abs(body_width_), 1.0e-6);
    const double leg_height_roll = std::atan2(right_leg_pos[0] - left_leg_pos[0], safe_body_width);
    const double combined_roll = normalize_angle(roll + leg_height_roll);
    const double roll_error = normalize_angle(exp_roll - combined_roll);
    const double leg_length_diff = safe_body_width * std::tan(roll_error);
    left_leg_exp_length = leg_exp_length + 0.5 * leg_length_diff;
    right_leg_exp_length = leg_exp_length - 0.5 * leg_length_diff;

    double vmc_mass_component=state==2?(kBaselinkMass*0.5*9.8*cos(theta)):0.0;
    const double com_height = average_leg_length + kWheelRadius + base_link_com_height_;
    const double centrifugal_force_ff_raw =
        state == 2 ? centrifugal_force_ff_gain_ * kBaselinkMass * lateral_accel * com_height / safe_body_width : 0.0;
    const double centrifugal_force_ff_limit = std::max(0.0, centrifugal_force_ff_limit_);
    const double centrifugal_force_ff = std::clamp(
        centrifugal_force_ff_raw, -centrifugal_force_ff_limit, centrifugal_force_ff_limit);
    double left_leg_dis_vmc_T           = vmc_kp * (left_leg_exp_length - left_leg_pos[0]) - vmc_kd * left_leg_vel[0]+vmc_mass_component;
    double right_leg_dis_vmc_T          = vmc_kp * (right_leg_exp_length - right_leg_pos[0]) - vmc_kd * right_leg_vel[0]+vmc_mass_component;
    if(state==2)
    {
        left_leg_dis_vmc_T -= centrifugal_force_ff;
        right_leg_dis_vmc_T += centrifugal_force_ff;
    }
    
    const double leg_angle_diff         = normalize_angle(left_leg_pos[1] - right_leg_pos[1]);
    const double leg_angle_diff_vel     = left_leg_vel[1] - right_leg_vel[1];
    const double spin_omega             = imu_state_.angular_velocity.z;
    const double spin_error             = exp_omega - spin_omega;
    if (state == 2) {
        wheel_spin_error_integral_ = std::clamp(
            wheel_spin_error_integral_ + spin_error * period.seconds(), -kWheelSpinIntegralLimit, kWheelSpinIntegralLimit);
    } else {
        wheel_spin_error_integral_ = 0.0;
    }
    const double spin_control_torque_diff =
        state == 2 ? wheel_diff_kp_ * spin_error + wheel_diff_ki_ * wheel_spin_error_integral_ : 0.0;
    const double spin_compensation_torque = -spin_control_torque_diff * average_leg_length / kWheelRadius;
    const double leg_angle_sync_torque  =
        -leg_angle_diff_kp_ * leg_angle_diff - leg_angle_diff_kd_ * leg_angle_diff_vel + spin_compensation_torque;
    const double left_leg_angle_torque  = u[1] + leg_angle_sync_torque;
    const double right_leg_angle_torque = u[1] - leg_angle_sync_torque;
    const double left_wheel_torque      = u[0] - spin_control_torque_diff;
    const double right_wheel_torque     = u[0] + spin_control_torque_diff;

    //VMC映射
    Eigen::Vector2d left_torque, right_torque;
    leg.inverse_dynamics(left_joint_pos, Eigen::Vector2d(left_leg_dis_vmc_T, left_leg_angle_torque), left_torque);
    leg.inverse_dynamics(right_joint_pos, Eigen::Vector2d(right_leg_dis_vmc_T, right_leg_angle_torque), right_torque);

    //设置各关节期望力矩
    robot_target_.l1.torque = std::clamp<double>(left_torque[0], -12.0, 12.0);
    robot_target_.l2.torque = std::clamp<double>(left_torque[1], -12.0, 12.0);
    robot_target_.r1.torque = std::clamp<double>(right_torque[0], -12.0, 12.0);
    robot_target_.r2.torque = std::clamp<double>(right_torque[1], -12.0, 12.0);
    robot_target_.lw.torque = std::clamp<double>(left_wheel_torque, -10.0, 10.0);
    robot_target_.rw.torque = std::clamp<double>(right_wheel_torque, -10.0, 10.0);
    

    RCLCPP_INFO_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 100,
        "state=%d\npos=%.4f\nroll=%.4f\nleg_roll=%.4f\ncombined_roll=%.4f\nay=%.4f\nay_raw=%.4f\ncentrifugal_ff=%.4f\nF=(%.4f,%.4f)\nu:(T=%.4f,Tp=%.4f)\nlength=(%.4f,%.4f)\nexp_length=(%.4f,%.4f)",
        state, x, roll, leg_height_roll, combined_roll, lateral_accel, level_accel.y(), centrifugal_force_ff,
        left_leg_force[0], right_leg_force[0],u[0], u[1],left_leg_pos[0],right_leg_pos[0], left_leg_exp_length,
        right_leg_exp_length);
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

double LQRController::clamp_torque(const double value) const {
    if (!std::isfinite(value)) {
        return 0.0;
    }
    return std::clamp(value, -torque_limit_, torque_limit_);
}

bool LQRController::use_mujoco_sim_chain() const { return use_mujoco_sim_chain_; }

bool LQRController::use_sim_time_parameter() const {
    rclcpp::Parameter use_sim_time;
    if (get_node()->get_parameter("use_sim_time", use_sim_time)) {
        return use_sim_time.as_bool();
    }
    return false;
}

controller_interface::InterfaceConfiguration LQRController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& joint_name : kMotorJointNames) {
        const auto name = std::string(joint_name);
        if (use_mujoco_sim_chain()) {
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
