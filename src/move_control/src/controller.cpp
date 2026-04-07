#include "move_control/controller.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

RobotController::RobotController(const rclcpp::Node::SharedPtr node)
    : node_(node) {
    controller_init();
}

RobotController::~RobotController() {}


void RobotController::controller_init() {
    imu_topic_ = node_->declare_parameter<std::string>("imu_topic", "/imu");
    posture_topic_ = node_->declare_parameter<std::string>("posture_topic", "/pose");
    wheel_topic_ = node_->declare_parameter<std::string>("wheel_topic", "/wheels_status");
    target_topic_ = node_->declare_parameter<std::string>("target_topic", "/wheels_target");
    control_period_s_ = node_->declare_parameter<double>("control_period", 0.01);
    command_limit_ = node_->declare_parameter<double>("command_limit", 5.0);

    imu_state_sub = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imu_callback(msg); });
    posture_sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        posture_topic_, 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { posture_callback(msg); });
    wheel_state_sub = node_->create_subscription<robot_interfaces::msg::Wheel>(
        wheel_topic_, 10, [this](const robot_interfaces::msg::Wheel::SharedPtr msg) { wheel_state_callback(msg); });
    target_pub = node_->create_publisher<robot_interfaces::msg::WheelExp>(target_topic_, 10);

    solver_.initialize();

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(control_period_s_));
    update_timer = node_->create_wall_timer(period, [this]() { update(); });
}

void RobotController::update() {
    if (!(imu_ready_ && posture_ready_ && wheel_ready_)) {
        return;
    }

    const rclcpp::Time now = node_->get_clock()->now();
    double dt = control_period_s_;
    if (last_update_time_.nanoseconds() > 0) {
        dt = std::max(1e-4, (now - last_update_time_).seconds());
    }
    last_update_time_ = now;

    if (!wheel_angle_initialized_) {
        wheel_angle_initialized_ = true;
    } else {
        left_wheel_angle_ += static_cast<double>(wheel_state.left_omega) * dt;
        right_wheel_angle_ += static_cast<double>(wheel_state.right_omega) * dt;
    }

    const auto current_state = build_current_state(dt);
    const auto reference_state = build_reference_state(current_state);

    NmpcSolver::InputVector control = NmpcSolver::InputVector::Zero();
    if (!solver_.solve(current_state, reference_state, control)) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000, "NMPC solve failed, publishing zero command.");
        control.setZero();
    }

    publish_command(control);
}

void RobotController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    robot_imu = *msg;
    imu_ready_ = true;
}

void RobotController::posture_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    previous_posture_ = robot_posture;
    robot_posture = *msg;
    robot_rotation.setX(robot_posture.pose.orientation.x);
    robot_rotation.setY(robot_posture.pose.orientation.y);
    robot_rotation.setZ(robot_posture.pose.orientation.z);
    robot_rotation.setW(robot_posture.pose.orientation.w);
    posture_ready_ = true;
}

void RobotController::wheel_state_callback(const robot_interfaces::msg::Wheel::SharedPtr msg) {
    wheel_state = *msg;
    wheel_ready_ = true;
}

NmpcSolver::StateVector RobotController::build_current_state(double dt) const {
    NmpcSolver::StateVector state = NmpcSolver::StateVector::Zero();

    state(0) = robot_posture.pose.position.x;
    state(1) = robot_posture.pose.position.y;
    state(2) = robot_posture.pose.position.z;
    state(3) = robot_posture.pose.orientation.x;
    state(4) = robot_posture.pose.orientation.y;
    state(5) = robot_posture.pose.orientation.z;
    state(6) = robot_posture.pose.orientation.w;
    state(7) = std::cos(left_wheel_angle_);
    state(8) = std::sin(left_wheel_angle_);
    state(9) = std::cos(right_wheel_angle_);
    state(10) = std::sin(right_wheel_angle_);

    if (dt > 0.0 && (previous_posture_.header.stamp.sec != 0 || previous_posture_.header.stamp.nanosec != 0)) {
        state(11) = (robot_posture.pose.position.x - previous_posture_.pose.position.x) / dt;
        state(12) = (robot_posture.pose.position.y - previous_posture_.pose.position.y) / dt;
        state(13) = (robot_posture.pose.position.z - previous_posture_.pose.position.z) / dt;
    }

    state(14) = robot_imu.angular_velocity.x;
    state(15) = robot_imu.angular_velocity.y;
    state(16) = robot_imu.angular_velocity.z;
    state(17) = wheel_state.left_omega;
    state(18) = wheel_state.right_omega;

    return state;
}

NmpcSolver::StateVector RobotController::build_reference_state(const NmpcSolver::StateVector & current_state) const {
    NmpcSolver::StateVector reference = current_state;

    reference(11) = 0.0;
    reference(12) = 0.0;
    reference(13) = 0.0;
    reference(14) = 0.0;
    reference(15) = 0.0;
    reference(16) = 0.0;
    reference(17) = 0.0;
    reference(18) = 0.0;

    return reference;
}

void RobotController::publish_command(const NmpcSolver::InputVector & control) const {
    robot_interfaces::msg::WheelExp msg;
    msg.left_omega = 0.0F;
    msg.right_omega = 0.0F;
    msg.left_torque = static_cast<float>(std::clamp(control(0), -command_limit_, command_limit_));
    msg.right_torque = static_cast<float>(std::clamp(control(1), -command_limit_, command_limit_));
    msg.kd = 0.0F;
    target_pub->publish(msg);
}
