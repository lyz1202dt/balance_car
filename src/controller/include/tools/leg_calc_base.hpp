#pragma once

#include <Eigen/Dense>

class LegCalcBase {
public:
    virtual ~LegCalcBase() = default;

    //正运动学，输入{前髋关节角, 后髋关节角}，输出{腿长, 腿角度}，腿竖直向下为0
    virtual bool forward_kinematics(const Eigen::Vector2d& rad, Eigen::Vector2d& leg) const = 0;

    //逆运动学，输入{腿长, 腿角度}，输出{前髋关节角, 后髋关节角}，腿竖直向下为0
    virtual bool inverse_kinematics(const Eigen::Vector2d& leg, Eigen::Vector2d& rad) const = 0;

    //正速度映射，输入{前髋关节角, 后髋关节角}和{前髋角速度, 后髋角速度}，输出{腿长速度, 腿角速度}
    bool forward_velocity(const Eigen::Vector2d& rad, const Eigen::Vector2d& joint_velocity, Eigen::Vector2d& leg_velocity) const;

    //逆速度映射，输入{前髋关节角, 后髋关节角}和{腿长速度, 腿角速度}，输出{前髋角速度, 后髋角速度}
    bool inverse_velocity(const Eigen::Vector2d& rad, const Eigen::Vector2d& leg_velocity, Eigen::Vector2d& joint_velocity) const;

    //正动力学，输入{t1,t2}，输出{F,T}
    bool forward_dynamics(const Eigen::Vector2d& rad, const Eigen::Vector2d& torque, Eigen::Vector2d& effort) const;

    //逆动力学，输入{F,T}，输出{t1,t2}
    bool inverse_dynamics(const Eigen::Vector2d& rad, const Eigen::Vector2d& effort, Eigen::Vector2d& torque) const;

protected:
    virtual Eigen::Matrix2d calc_jacobian(const Eigen::Vector2d& radian) const = 0;
};
