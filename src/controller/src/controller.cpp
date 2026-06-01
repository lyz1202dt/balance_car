#include <controller/controller.hpp>

#include <algorithm>
#include <cmath>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <robot_interfaces/msg/motor_state.hpp>
#include <robot_interfaces/msg/motor_target.hpp>

namespace car_controller {

namespace {

robot_interfaces::msg::MotorState& motor_state_at(robot_interfaces::msg::RobotState& msg, const size_t index) {
    switch (index) {
        case 0:
            return msg.l1;
        case 1:
            return msg.l2;
        case 2:
            return msg.lw;
        case 3:
            return msg.r1;
        case 4:
            return msg.r2;
        default:
            return msg.rw;
    }
}

robot_interfaces::msg::MotorTarget& target_at(robot_interfaces::msg::RobotTarget& msg, const size_t index) {
    switch (index) {
        case 0:
            return msg.l1;
        case 1:
            return msg.l2;
        case 2:
            return msg.lw;
        case 3:
            return msg.r1;
        case 4:
            return msg.r2;
        default:
            return msg.rw;
    }
}

}  // namespace

CarController::CarController() = default;

controller_interface::CallbackReturn CarController::on_init() {
    motor_joint_names_ = {
        "left_front_hip_joint",
        "left_rear_hip_joint",
        "left_wheel_joint",
        "right_front_hip_joint",
        "right_rear_hip_joint",
        "right_wheel_joint",
    };

    auto node = get_node();
    node->declare_parameter("joints", std::vector<std::string>(motor_joint_names_.begin(), motor_joint_names_.end()));
    node->declare_parameter("imu_sensor_name", imu_sensor_name_);
    node->declare_parameter("torque_limit", torque_limit_);
    node->declare_parameter("state_topic", state_topic_);
    node->declare_parameter("target_topic", target_topic_);
    node->declare_parameter("cmd_vel_topic", cmd_vel_topic_);

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

controller_interface::CallbackReturn CarController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    const auto joint_names = get_node()->get_parameter("joints").as_string_array();
    if (joint_names.size() != kMotorCount) {
        RCLCPP_ERROR(
            get_node()->get_logger(), "Expected %zu controlled joints, got %zu", kMotorCount, joint_names.size());
        return controller_interface::CallbackReturn::ERROR;
    }
    std::copy(joint_names.begin(), joint_names.end(), motor_joint_names_.begin());

    imu_sensor_name_ = get_node()->get_parameter("imu_sensor_name").as_string();
    torque_limit_ = get_node()->get_parameter("torque_limit").as_double();
    state_topic_ = get_node()->get_parameter("state_topic").as_string();
    target_topic_ = get_node()->get_parameter("target_topic").as_string();
    cmd_vel_topic_ = get_node()->get_parameter("cmd_vel_topic").as_string();
    imu_sensor_ = std::make_unique<semantic_components::IMUSensor>(imu_sensor_name_);

    state_publisher_ = get_node()->create_publisher<robot_interfaces::msg::RobotState>(state_topic_, 10);
    target_subscriber_ = get_node()->create_subscription<robot_interfaces::msg::RobotTarget>(
        target_topic_, 10, [this](const robot_interfaces::msg::RobotTarget& msg) { robot_target_ = msg; });
    robot_exp_vel = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 10, [this](const geometry_msgs::msg::Twist& msg) { expected_velocity_ = msg; });

    for (size_t i = 0; i < kMotorCount; ++i) {
        auto& target = target_at(robot_target_, i);
        target.rad = 0.0F;
        target.omega = 0.0F;
        target.torque = 0.0F;
        target.kp = 0.0F;
        target.kd = 0.0F;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CarController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    if (imu_sensor_ && !imu_sensor_->assign_loaned_state_interfaces(state_interfaces_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to assign IMU state interfaces for '%s'", imu_sensor_name_.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }

    if (state_publisher_) {
        state_publisher_->on_activate();
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CarController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    if (state_publisher_) {
        state_publisher_->on_deactivate();
    }
    if (imu_sensor_) {
        imu_sensor_->release_interfaces();
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CarController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    read_state_interfaces();
    update_motor_commands(period);
    publish_robot_state();
    return controller_interface::return_type::OK;
}

void CarController::read_state_interfaces() {
    for (size_t motor_index = 0; motor_index < kMotorCount; ++motor_index) {
        const size_t state_index = motor_index * kStateInterfacesPerMotor;
        motor_state_[motor_index].position = state_interfaces_[state_index].get_value();
        motor_state_[motor_index].velocity = state_interfaces_[state_index + 1].get_value();
        motor_state_[motor_index].effort = state_interfaces_[state_index + 2].get_value();
    }

    if (imu_sensor_) {
        imu_state_.orientation = imu_sensor_->get_orientation();
        imu_state_.angular_velocity = imu_sensor_->get_angular_velocity();
        imu_state_.linear_acceleration = imu_sensor_->get_linear_acceleration();
    }
}

void CarController::update_motor_commands(const rclcpp::Duration& period) {
    (void)period;

    // TODO(LQR-VMC): Replace this passthrough with the balance controller.
    // Inputs are motor_state_, imu_state_ and expected_velocity_; outputs are the six effort commands.
    for (size_t i = 0; i < kMotorCount; ++i) {
        const auto& target = target_at(robot_target_, i);
        command_interfaces_[i].set_value(clamp_torque(static_cast<double>(target.torque)));
    }
}

void CarController::publish_robot_state() {
    if (!state_publisher_ || !state_publisher_->is_activated()) {
        return;
    }

    for (size_t i = 0; i < kMotorCount; ++i) {
        auto& motor_msg = motor_state_at(robot_state_, i);
        motor_msg.rad = static_cast<float>(motor_state_[i].position);
        motor_msg.omega = static_cast<float>(motor_state_[i].velocity);
        motor_msg.torque = static_cast<float>(motor_state_[i].effort);
    }
    state_publisher_->publish(robot_state_);
}

double CarController::clamp_torque(const double value) const {
    if (!std::isfinite(value)) {
        return 0.0;
    }
    return std::clamp(value, -torque_limit_, torque_limit_);
}

controller_interface::InterfaceConfiguration CarController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : motor_joint_names_) {
        cfg.names.push_back(name + "/effort");
    }
    return cfg;
}

controller_interface::InterfaceConfiguration CarController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : motor_joint_names_) {
        cfg.names.push_back(name + "/position");
        cfg.names.push_back(name + "/velocity");
        cfg.names.push_back(name + "/effort");
    }

    semantic_components::IMUSensor imu_sensor(imu_sensor_name_);
    const auto imu_interfaces = imu_sensor.get_state_interface_names();
    cfg.names.insert(cfg.names.end(), imu_interfaces.begin(), imu_interfaces.end());
    return cfg;
}

}  // namespace car_controller

PLUGINLIB_EXPORT_CLASS(car_controller::CarController, controller_interface::ControllerInterface)
