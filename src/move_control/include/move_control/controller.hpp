#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <robot_interfaces/msg/detail/wheel__struct.hpp>
#include <robot_interfaces/msg/wheel.hpp>
#include <robot_interfaces/msg/wheel_exp.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <std_msgs/msg/string.hpp>
#include <memory>
#include <string>

#include "move_control/nmpc_solver.hpp"

class RobotController
{
    public:
    RobotController(const rclcpp::Node::SharedPtr node);
    ~RobotController();


    private:
    void controller_init();
    void update();
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void posture_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void wheel_state_callback(const robot_interfaces::msg::Wheel::SharedPtr msg);
    NmpcSolver::StateVector build_current_state(double dt) const;
    NmpcSolver::StateVector build_reference_state(const NmpcSolver::StateVector & current_state) const;
    void publish_command(const NmpcSolver::InputVector & control) const;

    rclcpp::Node::SharedPtr node_;
    bool model_loaded_ = false;
    bool imu_ready_ = false;
    bool posture_ready_ = false;
    bool wheel_ready_ = false;
    rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
    geometry_msgs::msg::PoseStamped previous_posture_;

    //参数服务服务端
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_server_;
    //更新定时器
    rclcpp::TimerBase::SharedPtr update_timer;
    //IMU信息订阅
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_state_sub;
    //位姿信息订阅
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr posture_sub;
    //轮子状态订阅
    rclcpp::Subscription<robot_interfaces::msg::Wheel>::SharedPtr wheel_state_sub;
    //轮子期望发布
    rclcpp::Publisher<robot_interfaces::msg::WheelExp>::SharedPtr target_pub;
    //TF广播器
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    //robot_description订阅
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;

    tf2::Quaternion robot_rotation;                    //机器人状态信息
    geometry_msgs::msg::PoseStamped robot_posture;
    geometry_msgs::msg::Twist robot_velocity;
    sensor_msgs::msg::Imu robot_imu;
    robot_interfaces::msg::Wheel wheel_state;
    NmpcSolver solver_;
    std::string imu_topic_;
    std::string posture_topic_;
    std::string wheel_topic_;
    std::string target_topic_;
    double control_period_s_ = 0.01;
    double command_limit_ = 5.0;
    double wheel_radius_ = 0.05;
};
