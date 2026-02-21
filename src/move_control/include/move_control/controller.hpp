#pragma once

#include <Eigen/Dense>
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


class RobotController
{
    public:
    RobotController(const rclcpp::Node::SharedPtr node);
    ~RobotController();


    private:
    void controller_init();
    void update();
    void quaternionLowPassFilter(double& w, double& x, double& y, double& z, double w1, double x1, double y1, double z1, double alpha);

    rclcpp::Node::SharedPtr node_;

    //参数服务服务端
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_server_;
    //更新定时器
    rclcpp::TimerBase::SharedPtr update_timer;
    //IMU信息订阅
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr imu_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr imu_angular_vel_sub;
    //轮子状态订阅
    rclcpp::Subscription<robot_interfaces::msg::Wheel>::SharedPtr state_sub;
    //轮子期望发布
    rclcpp::Publisher<robot_interfaces::msg::WheelExp>::SharedPtr target_pub;


    tf2::Quaternion robot_rotation;                    //机器人姿态
    geometry_msgs::msg::Twist robot_velocity;
    double direction_filter_gate{0.5};
};

