#include "controller/leg_calc.hpp"

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

Eigen::Matrix2d LegCalc::calc_jacobian(const Eigen::Vector2d& radian) const {
    autodiff::Vector2real q;
    q << radian[0], radian[1];

    const auto leg_state_func = [this](const autodiff::Vector2real& q_auto) {
        autodiff::Vector2real out;
        forward_kinematics_impl(q_auto,out);
        return out;
    };

    autodiff::Vector2real leg_state;
    Eigen::Matrix2d jacobian;
    autodiff::jacobian(leg_state_func, autodiff::wrt(q), autodiff::at(q), leg_state, jacobian);

    return jacobian;
}


LegCalc::LegCalc(): l0(0.11), l1(0.1844), l2(0.3130) {

}

bool LegCalc::forward_kinematics(const Eigen::Vector2d& rad, Eigen::Vector2d& leg) const {
    return forward_kinematics_impl(rad, leg);
}

bool LegCalc::inverse_kinematics(const Eigen::Vector2d& leg, Eigen::Vector2d& rad) const {
    const double leg_length = leg[0];
    const double leg_angle = leg[1];
    const double x = -leg_length * std::sin(leg_angle);
    const double y = leg_length * std::cos(leg_angle);

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
