#pragma once


#include <cmath>
#include <Eigen/Dense>
#include <stdexcept>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

class LegCalc{
public:
    LegCalc(const double &l0,const double &l1,const double &l2);

    //正运动学，输入(q1,q2)，输出(l,fai)
    template <typename Scalar>
    bool forward_kinematics(const Eigen::Matrix<Scalar, 2, 1> &rad,Eigen::Matrix<Scalar, 2, 1> pos) const {
        using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
        using std::abs;
        using std::cos;
        using std::sin;
        using std::sqrt;

        const auto l0_scalar = static_cast<Scalar>(l0);
        const auto l1_scalar = static_cast<Scalar>(l1);
        const auto l2_scalar = static_cast<Scalar>(l2);

        const auto p1 = Vector2(l0_scalar + l1_scalar * cos(rad[0]), l1_scalar * sin(rad[0]));
        const auto p2 = Vector2(-l0_scalar + l1_scalar * cos(rad[1]), l1_scalar * sin(rad[1]));

        const auto delta_x = p2[0] - p1[0];
        if (abs(delta_x) < static_cast<Scalar>(1e-6)) {
            return false;
        }

        const auto k_ = (p2[1] - p1[1]) / delta_x;
        const auto k  = -static_cast<Scalar>(1.0) / k_;
        auto dir      = Vector2(static_cast<Scalar>(1.0), k);
        const auto direction_sign = dir[1] < static_cast<Scalar>(0.0)
            ? static_cast<Scalar>(-1.0)
            : static_cast<Scalar>(1.0);
        dir = direction_sign * dir / sqrt(dir.dot(dir));

        const auto delta  = p2 - p1;
        const auto length = sqrt(l2_scalar * l2_scalar - delta.dot(delta));
        pos= dir * length + static_cast<Scalar>(0.5) * (p1 + p2);
        return true;
    }

    //逆运动学，输入{l,fai}，输出(q1,q2)
    bool inverse_kinematics(const Eigen::Matrix<double, 2, 1> &pos,Eigen::Matrix<double, 2, 1> &rad);

    //正动力学，输入{t1,t2}，输出(F,T)
    bool forward_dynamics(const Eigen::Vector2d &rad,const Eigen::Vector2d &torque,Eigen::Vector2d &effort);
    
    //逆动力学，输入{F,T}，输出(t1,t2)
    bool inverse_dynamics(const Eigen::Vector2d &rad,const Eigen::Vector2d &effort,Eigen::Vector2d &torque);
    
private:
    Eigen::Matrix2d calc_jacobian(const Eigen::Vector2d& radian);

    double l0,l1,l2;
};
