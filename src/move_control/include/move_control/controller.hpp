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


class RobotController
{
    public:
    RobotController(const rclcpp::Node::SharedPtr node);
    ~RobotController();


    private:
    void controller_init();
    void update();

    rclcpp::Node::SharedPtr node_;

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

    tf2::Quaternion robot_rotation;                    //机器人状态信息
    geometry_msgs::msg::PoseStamped robot_posture;
    geometry_msgs::msg::Twist robot_velocity;
    sensor_msgs::msg::Imu robot_imu;
    robot_interfaces::msg::Wheel wheel_state;
    double direction_filter_gate{0.5};
};

