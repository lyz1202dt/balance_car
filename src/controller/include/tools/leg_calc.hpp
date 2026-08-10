#pragma once


#include <cmath>
#include <Eigen/Dense>
#include <stdexcept>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

class LegCalc{
public:
    LegCalc(const double &l0,const double &l1,const double &l2);

    //正运动学，输入(前髋关节角, 后髋关节角)，输出(轮轴到髋中心的距离, 角度)
    template <typename Scalar>
    bool forward_kinematics(const Eigen::Matrix<Scalar, 2, 1> &rad,Eigen::Matrix<Scalar, 2, 1> &pos) const {
        using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
        using std::atan2;
        using std::cos;
        using std::sin;
        using std::sqrt;

        const auto l0_scalar = static_cast<Scalar>(l0);
        const auto l1_scalar = static_cast<Scalar>(l1);
        const auto l2_scalar = static_cast<Scalar>(l2);

        const auto front_elbow = Vector2(l0_scalar + l1_scalar * cos(rad[0]), l1_scalar * sin(rad[0]));
        const auto rear_elbow  = Vector2(-l0_scalar + l1_scalar * cos(rad[1]), l1_scalar * sin(rad[1]));

        const auto delta = rear_elbow - front_elbow;
        const auto distance = sqrt(delta.dot(delta));
        if (distance < static_cast<Scalar>(1e-6) || distance > static_cast<Scalar>(2.0) * l2_scalar) {
            return false;
        }

        const auto mid = static_cast<Scalar>(0.5) * (front_elbow + rear_elbow);
        const auto half_delta = static_cast<Scalar>(0.5) * delta;
        const auto height = sqrt(l2_scalar * l2_scalar - half_delta.dot(half_delta));
        const auto normal = Vector2(-delta[1], delta[0]) / distance;

        const auto candidate_a = mid + height * normal;
        const auto candidate_b = mid - height * normal;
        Vector2 wheel = candidate_a;
        if (candidate_b[1] > candidate_a[1]) {
            wheel = candidate_b;
        }

        pos[0] = sqrt(wheel.dot(wheel));
        pos[1] = atan2(wheel[1], wheel[0]);
        return true;
    }

    //逆运动学，输入{轮轴到髋中心的距离, 角度}，输出{前髋关节角, 后髋关节角}
    bool inverse_kinematics(const Eigen::Matrix<double, 2, 1> &pos,Eigen::Matrix<double, 2, 1> &rad);

    //正动力学，输入{t1,t2}，输出(F,T)
    bool forward_dynamics(const Eigen::Vector2d &rad,const Eigen::Vector2d &torque,Eigen::Vector2d &effort);
    
    //逆动力学，输入{F,T}，输出(t1,t2)
    bool inverse_dynamics(const Eigen::Vector2d &rad,const Eigen::Vector2d &effort,Eigen::Vector2d &torque);
    
private:
    Eigen::Matrix2d calc_jacobian(const Eigen::Vector2d& radian);

    double l0,l1,l2;
};
