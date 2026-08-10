#include <controller/controller.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <string>
#include <vector>


#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <robot_interfaces/msg/motor_state.hpp>
#include <robot_interfaces/msg/motor_target.hpp>

namespace lqr_controller {

namespace {

constexpr size_t kMotorCount                                    = 6;
constexpr size_t kTargetInterfacesPerMotor                      = 5;
constexpr const char* kReferencePrefix                          = "mujoco_sim_controller";
constexpr const char* kStateTopic                               = "robot_state";
constexpr const char* kTargetTopic                              = "robot_target";
constexpr const char* kCmdVelTopic                              = "cmd_vel";
constexpr double kHipHalfDistance                               = 0.11;
constexpr double kUpperLinkLength                               = 0.1844;
constexpr double kLowerLinkLength                               = 0.3130;
constexpr double kWheelRadius                                   = 0.1;
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

double normalized_zyx_pitch(const Eigen::Quaterniond& q) {
    const Eigen::Matrix3d rotation = q.toRotationMatrix();
    return std::asin(clamp_unit(-rotation(2, 0)));
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

} // namespace

LQRController::LQRController()
    : leg(kHipHalfDistance, kUpperLinkLength, kLowerLinkLength) {
    imu_state_.orientation.w = 1.0;
}

controller_interface::CallbackReturn LQRController::on_init() {
    auto node = get_node();
    auto_declare<std::string>("imu_topic", imu_topic_);
    auto_declare<std::string>("imu_pose_topic", imu_pose_topic_);
    auto_declare<double>("torque_limit", torque_limit_);

    param_cb_ = node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& param : params) {
            if (param.get_name() == "torque_limit") {
                torque_limit_ = param.as_double();
            }
        }
        return result;
    });

    K << -21.2083, -19.4842, -36.7996, -7.6400, 37.3930, 4.9429,
        14.1713,  12.3267  ,  22.5044,  4.7825, 127.4392, 1.31;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LQRController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    imu_topic_            = get_node()->get_parameter("imu_topic").as_string();
    imu_pose_topic_       = get_node()->get_parameter("imu_pose_topic").as_string();
    use_mujoco_sim_chain_ = use_sim_time_parameter();
    torque_limit_         = get_node()->get_parameter("torque_limit").as_double();

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
    (void)period;

    // 提取机体 pitch。Eigen::eulerAngles(2, 1, 0) 可能把接近直立的姿态表示成 pitch 接近 pi。
    Eigen::Quaterniond q;
    q.w() = imu_state_.orientation.w;
    q.x() = imu_state_.orientation.x;
    q.y() = imu_state_.orientation.y;
    q.z() = imu_state_.orientation.z;
    q.normalize();
    const double pitch = normalized_zyx_pitch(q);
    // RCLCPP_INFO_THROTTLE(get_node()->get_logger(),*get_node()->get_clock(),100,"(r=%.4f,p=%.4f,y=%.4f)",rpy[0],rpy[1],rpy[2]);

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

    // 准备填写状态空间方程
    double x     = (robot_state_.lw.rad + robot_state_.rw.rad) * kWheelRadius;
    double dx    = (robot_state_.lw.omega + robot_state_.rw.omega) * kWheelRadius;
    double theta = (pitch + right_leg_pos[1]);
    if (theta > M_PI)
        theta -= 2.0 * M_PI;
    else if (theta < -M_PI)
        theta += 2.0 * M_PI;

    double dtheta = (imu_state_.angular_velocity.y + right_leg_vel[1]); // 将速度加入解算
    double phi    = -pitch;
    double dphi   = -imu_state_.angular_velocity.y;

    Eigen::Vector<double, 6> X, exp_X;                                    // X<<fai取反为fai，theta为正，轮子方向正确。fai+theta=rad
    exp_X.setZero();
    exp_X[0] = exp_x;

    X << x, dx, theta, dtheta, phi, dphi;                                 // 填写当前状态向量
    u = K * (X-exp_X);                                                  // 计算得到控制量u

    // 腿长VMC部分，计算关节为了维持当前腿长所需要施加的力矩
    double left_leg_dis_vmc_T  = vmc_kp * (0.27 - left_leg_pos[0]) - vmc_kd * left_joint_vel[0];
    double right_leg_dis_vmc_T = vmc_kp * (0.27 - right_leg_pos[0]) - vmc_kd * right_joint_vel[0];

    Eigen::Vector2d left_torque, right_torque;
    leg.inverse_dynamics(left_joint_pos, Eigen::Vector2d(left_leg_dis_vmc_T, u[1]), left_torque);
    leg.inverse_dynamics(right_joint_pos, Eigen::Vector2d(right_leg_dis_vmc_T, u[1]), right_torque);



    // 状态切换安全检测，倾倒时切换位控
    if (pitch > 0.1 || pitch < -0.1) {
        state = 1;
    } else {
        state = 2;
    }



    if (state == 0)        // 什么也不做
    {

    } else if (state == 1) // 位控控制
    {
        Eigen::Vector2d rad = {0.0, 0.0};
        //if (leg.inverse_kinematics({0.27, 0.2}, rad)) {
        if (leg.inverse_kinematics({0.27, 0.0}, rad)) {
            robot_target_.l1.rad = robot_target_.r1.rad = static_cast<float>(rad[0]);
            robot_target_.l2.rad = robot_target_.r2.rad = static_cast<float>(rad[1]);
        }
        robot_target_.l1.kp = robot_target_.l2.kp = robot_target_.r1.kp = robot_target_.r2.kp = 50.0;
        robot_target_.l1.kd = robot_target_.l2.kd = robot_target_.r1.kd = robot_target_.r2.kd = 2.0;
        // robot_target_.lw.omega=10.0f;
        // robot_target_.rw.omega=10.0f;
    } else if (state == 2) // 平衡控制
    {
        robot_target_.l1.torque = std::clamp<double>(left_torque[0],-5.0,5.0);
        robot_target_.l2.torque = std::clamp<double>(left_torque[1],-5.0,5.0);
        robot_target_.r1.torque = std::clamp<double>(right_torque[0],-5.0,5.0);
        robot_target_.r2.torque = std::clamp<double>(right_torque[1],-5.0,5.0);
        robot_target_.lw.torque = std::clamp<double>(u[0],-1.0f,1.0f);
        robot_target_.rw.torque = std::clamp<double>(u[0],-1.0f,1.0f);
    } else if (state == 3) // 离地状态
    {
    }

    RCLCPP_INFO_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 100, "X=(%.4f,%.4f,%.4f,%.4f,%.4f,%.4f)\nu:T=%.4f,Tp=%.4f\nstate=%d,(left=%.4f,right=%.4f)", X[0],
        X[1], X[2], X[3], X[4], X[5], u[0],u[1],state, left_leg_dis_vmc_T, right_leg_dis_vmc_T);
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
