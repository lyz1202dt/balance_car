#pragma once

#include <limits>

class PID {
public:
    PID(
        double kp = 0.0, double kd = 0.0, double ki = 0.0,
        double inter_limit = std::numeric_limits<double>::infinity(),
        double output_limit = std::numeric_limits<double>::infinity());  // kp,kd,ki,积分限幅，输出限幅

    void set_gains(double kp, double kd, double ki);
    void set_limits(double inter_limit, double output_limit);
    void reset();

    double update(double current, double expect);
    double update(double current_pos, double current_vel, double expect_pos, double expect_vel = 0.0);
    double update_with_dt(double current, double expect, double dt);
    double update_with_dt(double current_pos, double current_vel, double expect_pos, double expect_vel, double dt);

private:
    double calculate(double position_error, double velocity_error, double dt);

    double kp_{0.0};
    double kd_{0.0};
    double ki_{0.0};
    double inter_limit_{std::numeric_limits<double>::infinity()};
    double output_limit_{std::numeric_limits<double>::infinity()};
    double integral_{0.0};
};
