#include "tools/leg_calc_base.hpp"

#include <cmath>

namespace {

bool is_finite_vector(const Eigen::Vector2d& vector) {
    return std::isfinite(vector[0]) && std::isfinite(vector[1]);
}

bool is_finite_matrix(const Eigen::Matrix2d& matrix) {
    return matrix.array().isFinite().all();
}

bool solve_map(const Eigen::Matrix2d& matrix, const Eigen::Vector2d& input, Eigen::Vector2d& output) {
    if (!is_finite_matrix(matrix)) {
        output.setZero();
        return false;
    }

    Eigen::FullPivLU<Eigen::Matrix2d> solver(matrix);
    solver.setThreshold(1e-9);
    if (!solver.isInvertible()) {
        output.setZero();
        return false;
    }

    output = solver.solve(input);
    return is_finite_vector(output);
}

} // namespace

bool LegCalcBase::forward_velocity(
    const Eigen::Vector2d& rad, const Eigen::Vector2d& joint_velocity, Eigen::Vector2d& leg_velocity) const {
    if (!is_finite_vector(rad) || !is_finite_vector(joint_velocity)) {
        leg_velocity.setZero();
        return false;
    }

    const Eigen::Matrix2d jacobian = calc_jacobian(rad);
    if (!is_finite_matrix(jacobian)) {
        leg_velocity.setZero();
        return false;
    }

    leg_velocity = jacobian * joint_velocity;
    return is_finite_vector(leg_velocity);
}

bool LegCalcBase::inverse_velocity(
    const Eigen::Vector2d& rad, const Eigen::Vector2d& leg_velocity, Eigen::Vector2d& joint_velocity) const {
    if (!is_finite_vector(rad) || !is_finite_vector(leg_velocity)) {
        joint_velocity.setZero();
        return false;
    }

    return solve_map(calc_jacobian(rad), leg_velocity, joint_velocity);
}

bool LegCalcBase::forward_dynamics(
    const Eigen::Vector2d& rad, const Eigen::Vector2d& torque, Eigen::Vector2d& effort) const {
    if (!is_finite_vector(rad) || !is_finite_vector(torque)) {
        effort.setZero();
        return false;
    }

    return solve_map(calc_jacobian(rad).transpose(), torque, effort);
}

bool LegCalcBase::inverse_dynamics(
    const Eigen::Vector2d& rad, const Eigen::Vector2d& effort, Eigen::Vector2d& torque) const {
    if (!is_finite_vector(rad) || !is_finite_vector(effort)) {
        torque.setZero();
        return false;
    }

    const Eigen::Matrix2d jacobian = calc_jacobian(rad);
    if (!is_finite_matrix(jacobian)) {
        torque.setZero();
        return false;
    }

    torque = jacobian.transpose() * effort;
    return is_finite_vector(torque);
}
