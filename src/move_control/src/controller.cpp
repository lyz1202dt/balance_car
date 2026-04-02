#include <chrono>
#include <move_control/controller.hpp>
#include <robot_interfaces/msg/detail/wheel_exp__struct.hpp>

using namespace std::chrono_literals;

RobotController::RobotController(const rclcpp::Node::SharedPtr node) {
    node_ = node;

    robot_rotation.setRPY(0.0, 0.0, 0.0);

    node_->declare_parameter("direction_filter_gate", 0.5);

    param_server_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        RCLCPP_INFO(node_->get_logger(), "更新参数");
        std::string name;
        for (const auto& param : params) {
            name = param.get_name();
            if (name == "direction_filter_gate") {
                direction_filter_gate = param.as_double();
            }
        }
        return result;
    });

    node_->get_parameter("direction_filter_gate", direction_filter_gate);                       // 设置参数初始值


    target_pub = node_->create_publisher<robot_interfaces::msg::WheelExp>("wheels_target", 10); // 创建期望位置发布者

    posture_sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/imu_pose_sensor/pose", rclcpp::SensorDataQoS(), [this](const geometry_msgs::msg::PoseStamped& msg) {
            robot_posture = msg;
            // 发布world到base_link的纯姿态TF
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp    = robot_posture.header.stamp;
            transform.header.frame_id = "world";
            transform.child_frame_id  = "base_link";

            // 设置位置为原点（纯姿态）
            transform.transform.translation.x = 0.0;
            transform.transform.translation.y = 0.0;
            transform.transform.translation.z = 0.0;

            // 设置旋转（从posture消息中提取）
            transform.transform.rotation = robot_posture.pose.orientation;

            tf_broadcaster_->sendTransform(transform);
        });

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    imu_state_sub = node_->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_imu_sensor/imu", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu& msg) { robot_imu = msg; });
    wheel_state_sub =
        node_->create_subscription<robot_interfaces::msg::Wheel>("wheels_status", 10, [this](const robot_interfaces::msg::Wheel& msg) {
            wheel_state = msg;
            update(); // 进行控制指令更新
        });


    controller_init();

    // update_timer = node_->create_wall_timer(5ms,[this](){

    // });
}

RobotController::~RobotController() {}

void RobotController::controller_init() {}

void RobotController::update() {

    // TODO:计算
    robot_interfaces::msg::WheelExp target;
    // TODO:填写期望值
    target_pub->publish(target);
}
