#include "tools/leg_calc.hpp"


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
    double x=pos[0]*std::cos(pos[1]);
    double y=pos[0]*std::sin(pos[1]);

    double a1=std::atan2(y,l0-x);
    double d1=std::sqrt(y*y+(l0-x)*(l0-x));
    double b1=std::atan2(d1*d1+l1*l1-l2*l2,2.0*d1*l1);
    double a2=std::atan2(y,l0+x);
    double d2=std::sqrt(y*y+(l0+x)*(l0+x));
    double b2=std::atan2(d2*d2+l1*l1-l2*l2,2.0*d1*l1);

    rad= {M_PI-a1-b1,a2+b2};
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
