#include <chrono>
#include <move_control/controller.hpp>
#include <robot_interfaces/msg/detail/wheel_exp__struct.hpp>

using namespace std::chrono_literals;

RobotController::RobotController(const rclcpp::Node::SharedPtr node) {
    node_ = node;

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

    node_->get_parameter("direction_filter_gate",direction_filter_gate);    //设置参数初始值


    target_pub = node_->create_publisher<robot_interfaces::msg::WheelExp>("wheels_target", 10); // 创建期望位置发布者

    // pose_sensor 发布的是 PoseStamped（见 mujoco_ros2_control/src/pose_sensor.cpp），这里必须用 PoseStamped 才能收到消息
    // 同时 QoS 需要与发布端兼容（发布端当前是 RELIABLE + TRANSIENT_LOCAL）
    imu_sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/imu_pose_sensor/pose", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
        [this](const geometry_msgs::msg::PoseStamped& msg) {
            const auto& q_msg = msg.pose.orientation;

            double qw = robot_rotation.getW();
            double qx = robot_rotation.getX();
            double qy = robot_rotation.getY();
            double qz = robot_rotation.getZ();

            quaternionLowPassFilter(qw, qx, qy, qz, q_msg.w, q_msg.x, q_msg.y, q_msg.z, direction_filter_gate); //(0-强滤波、1-不滤波)

            robot_rotation.setW(qw);
            robot_rotation.setX(qx);
            robot_rotation.setY(qy);
            robot_rotation.setZ(qz);
        });

    imu_angular_vel_sub = node_->create_subscription<geometry_msgs::msg::Vector3>(
        "/imu_imu_sensor/imu", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
        [this](const geometry_msgs::msg::Vector3& msg) {
            robot_velocity.angular.x = robot_velocity.angular.x + direction_filter_gate * (msg.x - robot_velocity.angular.x);
            robot_velocity.angular.y = robot_velocity.angular.y + direction_filter_gate * (msg.y - robot_velocity.angular.y);
            robot_velocity.angular.z = robot_velocity.angular.z + direction_filter_gate * (msg.z - robot_velocity.angular.z);
        });
    state_sub =
        node_->create_subscription<robot_interfaces::msg::Wheel>("wheels_status", 10, [this](const robot_interfaces::msg::Wheel& msg) {
            // TODO:填写系统状态矩阵
            update(); // 进行控制指令更新
        });


    controller_init();

    // update_timer = node_->create_wall_timer(2ms, std::bind(&RobotController::update, this));
}

RobotController::~RobotController() {}

void RobotController::controller_init() {

}

void RobotController::update() {

}


void RobotController::quaternionLowPassFilter(
    double& w, double& x, double& y, double& z, double w1, double x1, double y1, double z1, double alpha) {

    auto normalizeQuaternion = [](double& w, double& x, double& y, double& z) {
        double norm = std::sqrt(w * w + x * x + y * y + z * z);
        if (norm < 1e-12) {
            // 退化情况，回到单位四元数
            w = 1.0;
            x = y = z = 0.0;
            return;
        }
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    };


    // 1. 单位化输入（防御性编程）
    normalizeQuaternion(w, x, y, z);
    normalizeQuaternion(w1, x1, y1, z1);

    // 2. 点积，判断是否需要取反（双覆盖问题）
    double dot = w * w1 + x * x1 + y * y1 + z * z1;
    if (dot < 0.0) {
        w1  = -w1;
        x1  = -x1;
        y1  = -y1;
        z1  = -z1;
        dot = -dot;
    }

    // 3. 小角度近似（避免 acos / sin 数值不稳定）
    const double DOT_THRESHOLD = 0.9995;
    if (dot > DOT_THRESHOLD) {
        // 线性插值
        w = w + alpha * (w1 - w);
        x = x + alpha * (x1 - x);
        y = y + alpha * (y1 - y);
        z = z + alpha * (z1 - z);
        normalizeQuaternion(w, x, y, z);
        return;
    }

    // 4. 标准 SLERP
    double theta     = std::acos(dot);
    double sin_theta = std::sin(theta);

    double w_a = std::sin((1.0 - alpha) * theta) / sin_theta;
    double w_b = std::sin(alpha * theta) / sin_theta;

    double w_new = w_a * w + w_b * w1;
    double x_new = w_a * x + w_b * x1;
    double y_new = w_a * y + w_b * y1;
    double z_new = w_a * z + w_b * z1;

    w = w_new;
    x = x_new;
    y = y_new;
    z = z_new;

    // 5. 再次单位化（强烈建议保留）
    normalizeQuaternion(w, x, y, z);
}
