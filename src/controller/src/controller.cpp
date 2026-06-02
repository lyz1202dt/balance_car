#include <controller/controller.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <robot_interfaces/msg/motor_state.hpp>
#include <robot_interfaces/msg/motor_target.hpp>

namespace lqr_controller {

namespace {

constexpr size_t kMotorCount               = 6;
constexpr size_t kTargetInterfacesPerMotor = 5;
constexpr const char* kReferencePrefix     = "mujoco_sim_controller";
constexpr const char* kStateTopic          = "robot_state";
constexpr const char* kTargetTopic         = "robot_target";
constexpr const char* kCmdVelTopic         = "cmd_vel";

size_t motor_target_index(const size_t motor_index, const size_t interface_index) {
    return motor_index * kTargetInterfacesPerMotor + interface_index;
}

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

double leg_position_direction_sign(const Eigen::Vector2d& radian, const double l0, const double l1) {
    const auto p1 = Eigen::Vector2d(l0 + l1 * std::cos(radian[0]), l1 * std::sin(radian[0]));
    const auto p2 = Eigen::Vector2d(-l0 + l1 * std::cos(radian[1]), l1 * std::sin(radian[1]));

    if (std::abs(p2[0] - p1[0]) < 1e-6) {
        throw std::runtime_error("五连杆运动学正解时除0异常");
    }

    const double k_ = (p2[1] - p1[1]) / (p2[0] - p1[0]);
    const double k  = -1.0 / k_;
    const auto dir  = Eigen::Vector2d(1.0, k).normalized();
    return dir[1] < 0.0 ? -1.0 : 1.0;
}

template <typename Scalar>
Eigen::Matrix<Scalar, 2, 1> calc_leg_position(
    const Eigen::Matrix<Scalar, 2, 1>& radian, const double l0, const double l1, const double l2,
    const double direction_sign) {
    using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
    using std::cos;
    using std::sin;
    using std::sqrt;

    const auto l0_scalar = static_cast<Scalar>(l0);
    const auto l1_scalar = static_cast<Scalar>(l1);
    const auto l2_scalar = static_cast<Scalar>(l2);
    const auto sign      = static_cast<Scalar>(direction_sign);

    const auto p1 = Vector2(l0_scalar + l1_scalar * cos(radian[0]), l1_scalar * sin(radian[0]));
    const auto p2 = Vector2(-l0_scalar + l1_scalar * cos(radian[1]), l1_scalar * sin(radian[1]));

    const auto k_ = (p2[1] - p1[1]) / (p2[0] - p1[0]);
    const auto k  = -static_cast<Scalar>(1.0) / k_;
    auto dir      = Vector2(static_cast<Scalar>(1.0), k);
    dir           = sign * dir / sqrt(dir.dot(dir));

    const auto delta  = p2 - p1;
    const auto length = sqrt(l2_scalar * l2_scalar - delta.dot(delta));
    return dir * length + static_cast<Scalar>(0.5) * (p1 + p2);
}

} // namespace

LQRController::LQRController() = default;

controller_interface::CallbackReturn LQRController::on_init() {
    motor_joint_names_ = {
        "left_front_hip_joint",  "left_rear_hip_joint",  "left_wheel_joint",
        "right_front_hip_joint", "right_rear_hip_joint", "right_wheel_joint",
    };

    auto node = get_node();
    auto_declare<std::string>("imu_topic", imu_topic_);
    auto_declare<double>("torque_limit", torque_limit_);
    auto_declare<std::vector<double>>("default_kp", std::vector<double>(kMotorCount, 0.0));
    auto_declare<std::vector<double>>("default_kd", std::vector<double>(kMotorCount, 0.0));

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

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LQRController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    imu_topic_            = get_node()->get_parameter("imu_topic").as_string();
    use_mujoco_sim_chain_ = use_sim_time_parameter();
    torque_limit_         = get_node()->get_parameter("torque_limit").as_double();
    if (!load_default_pd_gains()) {
        return controller_interface::CallbackReturn::ERROR;
    }

    state_publisher_   = get_node()->create_publisher<robot_interfaces::msg::RobotState>(kStateTopic, 10);
    target_subscriber_ = get_node()->create_subscription<robot_interfaces::msg::RobotTarget>(
        kTargetTopic, 10, [this](const robot_interfaces::msg::RobotTarget& msg) { robot_target_ = msg; });
    robot_exp_vel = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        kCmdVelTopic, 10, [this](const geometry_msgs::msg::Twist& msg) { expected_velocity_ = msg; });
    imu_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu& msg) { imu_callback(msg); });

    for (size_t i = 0; i < kMotorCount; ++i) {
        auto& target  = target_at(robot_target_, i);
        target.rad    = 0.0F;
        target.omega  = 0.0F;
        target.torque = 0.0F;
        target.kp     = static_cast<float>(default_kp_[i]);
        target.kd     = static_cast<float>(default_kd_[i]);
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LQRController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    if (state_publisher_) {
        state_publisher_->on_activate();
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LQRController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    if (state_publisher_) {
        state_publisher_->on_deactivate();
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LQRController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    read_state_interfaces();
    update_motor_commands(period);
    publish_robot_state();
    return controller_interface::return_type::OK;
}

void LQRController::read_state_interfaces() {
    for (size_t motor_index = 0; motor_index < kMotorCount; ++motor_index) {
        const size_t state_index           = motor_index * kStateInterfacesPerMotor;
        motor_state_[motor_index].position = state_interfaces_[state_index].get_value();
        motor_state_[motor_index].velocity = state_interfaces_[state_index + 1].get_value();
        motor_state_[motor_index].effort   = state_interfaces_[state_index + 2].get_value();
    }
}

void LQRController::update_motor_commands(const rclcpp::Duration& period) {
    (void)period;

    // TODO(LQR-VMC): Replace this passthrough with the balance controller.
    // Inputs are motor_state_, imu_state_ and expected_velocity_; outputs are the six motor targets.
    for (size_t i = 0; i < kMotorCount; ++i) {
        const auto& target         = target_at(robot_target_, i);
        const size_t command_index = motor_target_index(i, 0);
        command_interfaces_[command_index].set_value(static_cast<double>(target.rad));
        command_interfaces_[command_index + 1].set_value(static_cast<double>(target.omega));
        command_interfaces_[command_index + 2].set_value(clamp_torque(static_cast<double>(target.torque)));
        command_interfaces_[command_index + 3].set_value(static_cast<double>(target.kp));
        command_interfaces_[command_index + 4].set_value(static_cast<double>(target.kd));
    }
}

void LQRController::publish_robot_state() {
    if (!state_publisher_ || !state_publisher_->is_activated()) {
        return;
    }

    for (size_t i = 0; i < kMotorCount; ++i) {
        auto& motor_msg  = motor_state_at(robot_state_, i);
        motor_msg.rad    = static_cast<float>(motor_state_[i].position);
        motor_msg.omega  = static_cast<float>(motor_state_[i].velocity);
        motor_msg.torque = static_cast<float>(motor_state_[i].effort);
    }
    state_publisher_->publish(robot_state_);
}

bool LQRController::load_default_pd_gains() {
    const auto kp_values = get_node()->get_parameter("default_kp").as_double_array();
    const auto kd_values = get_node()->get_parameter("default_kd").as_double_array();
    if (kp_values.size() != kMotorCount || kd_values.size() != kMotorCount) {
        RCLCPP_ERROR(
            get_node()->get_logger(), "Expected %zu default_kp and default_kd values, got %zu and %zu", kMotorCount, kp_values.size(),
            kd_values.size());
        return false;
    }

    std::copy(kp_values.begin(), kp_values.end(), default_kp_.begin());
    std::copy(kd_values.begin(), kd_values.end(), default_kd_.begin());
    return true;
}

void LQRController::imu_callback(const sensor_msgs::msg::Imu& msg) {
    imu_state_.orientation = {
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w,
    };
    imu_state_.angular_velocity = {
        msg.angular_velocity.x,
        msg.angular_velocity.y,
        msg.angular_velocity.z,
    };
    imu_state_.linear_acceleration = {
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z,
    };
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

    for (const auto& name : motor_joint_names_) {
        if (use_sim_time_parameter()) {
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

    for (const auto& name : motor_joint_names_) {
        cfg.names.push_back(name + "/position");
        cfg.names.push_back(name + "/velocity");
        cfg.names.push_back(name + "/effort");
    }

    return cfg;
}


LegCalc::LegCalc(const double& l0, const double& l1, const double& l2): l0(l0),l1(l1),l2(l2) {

}
Eigen::Vector2d LegCalc::calc_position(const Eigen::Vector2d& radian) {
    const auto direction_sign = leg_position_direction_sign(radian, l0, l1);
    return calc_leg_position(radian, l0, l1, l2, direction_sign);
}

Eigen::Vector2d LegCalc::calc_radian(const Eigen::Vector2d& position) {
    double x=position[0];
    double y=position[1];

    double a1=std::atan2(y,l0-x);
    double d1=std::sqrt(y*y+(l0-x)*(l0-x));
    double b1=std::atan2(d1*d1+l1*l1-l2*l2,2.0*d1*l1);
    double a2=std::atan2(y,l0+x);
    double d2=std::sqrt(y*y+(l0+x)*(l0+x));
    double b2=std::atan2(d2*d2+l1*l1-l2*l2,2.0*d1*l1);

    return {M_PI-a1-b1,a2+b2};
}
Eigen::Vector2d LegCalc::calc_torque(const Eigen::Vector2d& radian, const Eigen::Vector2d& force) {
    const auto jacobian = calc_jacobian(radian);
    return jacobian.transpose() * force;
}
Eigen::Vector2d LegCalc::calc_force(const Eigen::Vector2d& radian, const Eigen::Vector2d& torque) {
    const auto jacobian = calc_jacobian(radian);
    const auto jt       = jacobian.transpose();

    if (std::abs(jt.determinant()) < 1e-9) {
        throw std::runtime_error("五连杆雅可比矩阵奇异，无法由力矩求解足端力");
    }

    return jt.fullPivLu().solve(torque);
}

Eigen::Matrix2d LegCalc::calc_jacobian(const Eigen::Vector2d& radian) const {
    const auto direction_sign = leg_position_direction_sign(radian, l0, l1);

    autodiff::Vector2real q;
    q << radian[0], radian[1];

    const auto position_func = [this, direction_sign](const autodiff::Vector2real& q_auto) {
        return calc_leg_position(q_auto, l0, l1, l2, direction_sign);
    };

    autodiff::Vector2real position;
    Eigen::Matrix2d jacobian;
    autodiff::jacobian(position_func, autodiff::wrt(q), autodiff::at(q), position, jacobian);

    return jacobian;
}


} // namespace lqr_controller

PLUGINLIB_EXPORT_CLASS(lqr_controller::LQRController, controller_interface::ControllerInterface)
