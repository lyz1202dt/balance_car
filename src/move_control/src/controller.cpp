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
    imu_topic_ = node_->declare_parameter<std::string>("imu_topic", "/imu_imu_sensor/imu");
    posture_topic_ = node_->declare_parameter<std::string>("posture_topic", "/base_link_pose_sensor/pose");
    wheel_topic_ = node_->declare_parameter<std::string>("wheel_topic", "/wheels_status");
    target_topic_ = node_->declare_parameter<std::string>("target_topic", "/wheels_target");
    control_period_s_ = node_->declare_parameter<double>("control_period", 0.01);
    command_limit_ = node_->declare_parameter<double>("command_limit", 5.0);
    wheel_radius_ = node_->declare_parameter<double>("wheel_radius", 0.05);

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

    const auto current_state = build_current_state(dt);
    const auto reference_state = build_reference_state(current_state);

    NmpcSolver::InputVector control = NmpcSolver::InputVector::Zero();
    int ret=solver_.solve(current_state, reference_state, control);
    if (ret!=0) {
        RCLCPP_WARN(
            node_->get_logger(), "NMPC solve failed, publishing zero command.err=%d",ret);
        control.setZero();
    }
    else {
        RCLCPP_INFO(
            node_->get_logger(), "NMPC solve succeeded, control: [%f, %f]", control(0), control(1));
    }

    RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        500,
        "balance state position=%.4f m, pitch=%.4f rad, velocity=%.4f m/s, pitch_rate=%.4f rad/s, torque_cmd=[%.4f, %.4f]",
        current_state(0),
        current_state(1),
        current_state(2),
        current_state(3),
        control(0),
        control(1));

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
    if (robot_rotation.length2() > 1e-12) {
        robot_rotation.normalize();
    } else {
        robot_rotation.setValue(0.0, 0.0, 0.0, 1.0);
    }
    robot_posture.pose.orientation.x = robot_rotation.x();
    robot_posture.pose.orientation.y = robot_rotation.y();
    robot_posture.pose.orientation.z = robot_rotation.z();
    robot_posture.pose.orientation.w = robot_rotation.w();
    posture_ready_ = true;
}

void RobotController::wheel_state_callback(const robot_interfaces::msg::Wheel::SharedPtr msg) {
    wheel_state = *msg;
    wheel_ready_ = true;
}

NmpcSolver::StateVector RobotController::build_current_state(double dt) const {
    static_cast<void>(dt);
    NmpcSolver::StateVector state = NmpcSolver::StateVector::Zero();

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(robot_rotation).getRPY(roll, pitch, yaw);
    static_cast<void>(roll);
    static_cast<void>(yaw);

    const double wheel_forward_velocity =
        0.5 * (static_cast<double>(wheel_state.left_omega) + static_cast<double>(wheel_state.right_omega)) *
        wheel_radius_;

    state(0) = robot_posture.pose.position.x;
    state(1) = pitch;
    state(2) = wheel_forward_velocity;
    state(3) = robot_imu.angular_velocity.y;

    return state;
}

NmpcSolver::StateVector RobotController::build_reference_state(const NmpcSolver::StateVector & current_state) const {
    NmpcSolver::StateVector reference = current_state;

    // Hold the current longitudinal position while regulating back to the
    // upright, zero-velocity equilibrium of the planar balance model.
    reference(1) = 0.0;
    reference(2) = 0.0;
    reference(3) = 0.0;

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
