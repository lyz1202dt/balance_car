#include "tools/leg_calc.hpp"

#include <algorithm>
#include <cmath>

namespace {

double clamp_unit(const double value) {
    return std::clamp(value, -1.0, 1.0);
}

bool reachable(const double distance, const double link_a, const double link_b) {
    constexpr double kEpsilon = 1e-9;
    return distance > kEpsilon && distance <= link_a + link_b && distance >= std::abs(link_a - link_b);
}

} // namespace

Eigen::Matrix2d LegCalc::calc_jacobian(const Eigen::Vector2d& radian){
    autodiff::Vector2real q;
    q << radian[0], radian[1];

    const auto position_func = [this](const autodiff::Vector2real& q_auto) {
        autodiff::Vector2real out;
        forward_kinematics(q_auto,out);
        return out;
    };

    autodiff::Vector2real position;
    Eigen::Matrix2d jacobian;
    autodiff::jacobian(position_func, autodiff::wrt(q), autodiff::at(q), position, jacobian);

    return jacobian;
}


LegCalc::LegCalc(const double& l0, const double& l1, const double& l2): l0(l0),l1(l1),l2(l2) {

}

bool LegCalc::inverse_kinematics(const Eigen::Matrix<double, 2, 1> &pos,Eigen::Matrix<double, 2, 1> &rad) {
    const double x = pos[0] * std::cos(pos[1]);
    const double y = pos[0] * std::sin(pos[1]);

    const double front_dx = x - l0;
    const double front_dy = y;
    const double front_distance = std::hypot(front_dx, front_dy);
    if (!reachable(front_distance, l1, l2)) {
        return false;
    }
    const double front_angle = std::atan2(front_dy, front_dx);
    const double front_offset = std::acos(
        clamp_unit((l1 * l1 + front_distance * front_distance - l2 * l2) / (2.0 * l1 * front_distance)));

    const double rear_dx = x + l0;
    const double rear_dy = y;
    const double rear_distance = std::hypot(rear_dx, rear_dy);
    if (!reachable(rear_distance, l1, l2)) {
        return false;
    }
    const double rear_angle = std::atan2(rear_dy, rear_dx);
    const double rear_offset = std::acos(
        clamp_unit((l1 * l1 + rear_distance * rear_distance - l2 * l2) / (2.0 * l1 * rear_distance)));

    rad = {front_angle - front_offset, rear_angle + rear_offset};
    return true;
}


    //正动力学
bool LegCalc::forward_dynamics(const Eigen::Vector2d &rad,const Eigen::Vector2d &torque,Eigen::Vector2d &effort) {
    auto jacobian = calc_jacobian(rad);
    effort= jacobian.transpose().fullPivLu().solve(torque);
    return true;
}

    //逆动力学
bool LegCalc::inverse_dynamics(const Eigen::Vector2d &rad,const Eigen::Vector2d &effort,Eigen::Vector2d &torque) {
    const auto jacobian = calc_jacobian(rad);
    torque= jacobian.transpose() * effort;
    
    return true;
}
