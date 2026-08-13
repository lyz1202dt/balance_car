#include "tools/pid.hpp"

#include <algorithm>
#include <cmath>

namespace {

double sanitize_limit(const double limit) {
    if (!std::isfinite(limit)) {
        return std::numeric_limits<double>::infinity();
    }
    return std::max(0.0, std::abs(limit));
}

double clamp_symmetric(const double value, const double limit) {
    if (!std::isfinite(value)) {
        return 0.0;
    }
    if (!std::isfinite(limit)) {
        return value;
    }
    return std::clamp(value, -limit, limit);
}

} // namespace

PID::PID(
    const double kp, const double kd, const double ki, const double inter_limit,
    const double output_limit)
    : kp_(kp),
      kd_(kd),
      ki_(ki),
      inter_limit_(sanitize_limit(inter_limit)),
      output_limit_(sanitize_limit(output_limit)) {
}

void PID::set_gains(const double kp, const double kd, const double ki) {
    kp_ = kp;
    kd_ = kd;
    ki_ = ki;
}

void PID::set_limits(const double inter_limit, const double output_limit) {
    inter_limit_ = sanitize_limit(inter_limit);
    output_limit_ = sanitize_limit(output_limit);
    integral_ = clamp_symmetric(integral_, inter_limit_);
}

void PID::reset() {
    integral_ = 0.0;
}

double PID::update(const double current, const double expect) {
    return update_with_dt(current, expect, 1.0);
}

double PID::update(
    const double current_pos, const double current_vel, const double expect_pos,
    const double expect_vel) {
    return update_with_dt(current_pos, current_vel, expect_pos, expect_vel, 1.0);
}

double PID::update_with_dt(const double current, const double expect, const double dt) {
    return calculate(expect - current, 0.0, dt);
}

double PID::update_with_dt(
    const double current_pos, const double current_vel, const double expect_pos,
    const double expect_vel, const double dt) {
    return calculate(expect_pos - current_pos, expect_vel - current_vel, dt);
}

double PID::calculate(const double position_error, const double velocity_error, const double dt) {
    if (!std::isfinite(position_error) || !std::isfinite(velocity_error)) {
        return 0.0;
    }

    const double safe_dt = std::isfinite(dt) && dt > 0.0 ? dt : 0.0;
    integral_ = clamp_symmetric(integral_ + position_error * safe_dt, inter_limit_);

    const double output = kp_ * position_error + kd_ * velocity_error + ki_ * integral_;
    return clamp_symmetric(output, output_limit_);
}
