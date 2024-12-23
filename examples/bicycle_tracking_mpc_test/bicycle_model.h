#pragma once
#include <ArduinoEigenDense.h>
#include <altro_solver.hpp>
#include "altro/solver/typedefs.hpp"
#include "altro/utils/formatting.hpp"

using ContinuousDynamicsFunction = std::function<void(double *, const double *, const double *)>;
using ContinuousDynamicsJacobian = std::function<void(double *, const double *, const double *)>;

altro::ExplicitDynamicsFunction MidpointDynamics(int n, int m, ContinuousDynamicsFunction f)
{
    auto fd = [n, m, f](double *xn, const double *x, const double *u, float h)
    {
        static Eigen::VectorXd xm(n);
        Eigen::Map<Eigen::VectorXd> xn_vec(xn, n);
        Eigen::Map<const Eigen::VectorXd> x_vec(x, n);
        Eigen::Map<const Eigen::VectorXd> u_vec(u, n);
        f(xm.data(), x, u);
        xm *= h / 2;
        xm.noalias() += x_vec;
        f(xn, xm.data(), u);
        xn_vec = x_vec + h * xn_vec;
    };
    return fd;
}

altro::ExplicitDynamicsJacobian MidpointJacobian(int n, int m, ContinuousDynamicsFunction f,
                                                 ContinuousDynamicsJacobian df)
{
    auto fd = [n, m, f, df](double *jac, const double *x, const double *u, float h)
    {
        static Eigen::MatrixXd A(n, n);
        static Eigen::MatrixXd B(n, m);
        static Eigen::MatrixXd Am(n, n);
        static Eigen::MatrixXd Bm(n, m);
        static Eigen::VectorXd xm(n);
        static Eigen::MatrixXd In = Eigen::MatrixXd::Identity(n, n);

        Eigen::Map<Eigen::MatrixXd> J(jac, n, n + m);
        Eigen::Map<const Eigen::VectorXd> x_vec(x, n);
        Eigen::Map<const Eigen::VectorXd> u_vec(u, n);

        // Evaluate the midpoint
        f(xm.data(), x, u);
        xm = x_vec + h / 2 * xm;

        // Evaluate the Jacobian
        df(J.data(), x, u);
        A = J.leftCols(n);
        B = J.rightCols(m);

        // Evaluate the Jacobian at the midpoint
        df(J.data(), xm.data(), u);
        Am = J.leftCols(n);
        Bm = J.rightCols(m);

        // Apply the chain rule
        J.leftCols(n) = In + h * Am * (In + h / 2 * A);
        J.rightCols(m) = h * (Am * h / 2 * B + Bm);
    };
    return fd;
}

class BicycleModel
{
public:
    enum class ReferenceFrame
    {
        CenterOfGravity,
        Rear,
        Front
    };

    explicit BicycleModel(ReferenceFrame frame = ReferenceFrame::CenterOfGravity)
        : reference_frame_(frame) {}

    void Dynamics(double *x_dot, const double *x, const double *u) const;
    void Jacobian(double *jac, const double *x, const double *u) const;

    void SetLengths(double length, double length_to_rear_wheel_from_cg)
    {
        length_ = length;
        distance_to_rear_wheels_ = length_to_rear_wheel_from_cg;
    }

    static constexpr int NumStates = 4;
    static constexpr int NumInputs = 2;

private:
    ReferenceFrame reference_frame_;
    double length_ = 2.7;
    double distance_to_rear_wheels_ = 1.5;
};

void BicycleModel::Dynamics(double *x_dot, const double *x, const double *u) const
{
    double v = u[0];         // longitudinal velocity (m/s)
    double delta_dot = u[1]; // steering angle rage (rad/s)
    double theta = x[2];     // heading angle (rad) relative to x-axis
    double delta = x[3];     // steering angle (rad)

    double beta = 0;
    double omega = 0;
    double stheta = 0;
    double ctheta = 0;
    switch (reference_frame_)
    {
    case ReferenceFrame::CenterOfGravity:
        beta = std::atan2(distance_to_rear_wheels_ * delta, length_);
        omega = v * std::cos(beta) * std::tan(delta) / length_;
        stheta = std::sin(theta + beta);
        ctheta = std::cos(theta + beta);
        break;
    case ReferenceFrame::Rear:
        omega = v * tan(delta) / length_;
        stheta = std::sin(theta);
        ctheta = std::cos(theta);
        break;
    case ReferenceFrame::Front:
        omega = v * std::sin(delta) / length_;
        stheta = std::sin(theta + delta);
        ctheta = std::cos(theta + delta);
        break;
    };
    double px_dot = v * ctheta;
    double py_dot = v * stheta;
    x_dot[0] = px_dot;
    x_dot[1] = py_dot;
    x_dot[2] = omega;
    x_dot[3] = delta_dot;
}

void BicycleModel::Jacobian(double *jac, const double *x, const double *u) const
{
    double v = u[0];     // longitudinal velocity (m/s)
    double theta = x[2]; // heading angle (rad) relative to x-axis
    double delta = x[3]; // steering angle (rad)

    Eigen::Map<Eigen::Matrix<double, 4, 6>> J(jac);
    double beta = 0;
    double dbeta_ddelta = 0;
    double by = 0;
    double bx = 0;
    double domega_ddelta = 0;
    double domega_dv = 0;

    double stheta = 0;
    double ctheta = 0;
    double ds_dtheta = 0;
    double dc_dtheta = 0;
    double ds_ddelta = 0;
    double dc_ddelta = 0;
    switch (reference_frame_)
    {
    case ReferenceFrame::CenterOfGravity:
        by = distance_to_rear_wheels_ * delta;
        bx = length_;
        beta = std::atan2(by, bx);
        dbeta_ddelta = bx / (bx * bx + by * by) * distance_to_rear_wheels_;
        domega_ddelta = v / length_ *
                        (-std::sin(beta) * std::tan(delta) * dbeta_ddelta +
                         std::cos(beta) / (std::cos(delta) * std::cos(delta)));
        domega_dv = std::cos(beta) * std::tan(delta) / length_;

        stheta = std::sin(theta + beta);
        ctheta = std::cos(theta + beta);
        ds_dtheta = +std::cos(theta + beta);
        dc_dtheta = -std::sin(theta + beta);
        ds_ddelta = +std::cos(theta + beta) * dbeta_ddelta;
        dc_ddelta = -std::sin(theta + beta) * dbeta_ddelta;
        break;
    case ReferenceFrame::Rear:
        domega_ddelta = v / length_ / (std::cos(delta) * std::cos(delta));
        domega_dv = std::tan(delta) / length_;

        stheta = std::sin(theta);
        ctheta = std::cos(theta);
        ds_dtheta = +std::cos(theta);
        dc_dtheta = -std::sin(theta);
        break;
    case ReferenceFrame::Front:
        domega_ddelta = v / length_ * std::cos(delta);
        domega_dv = std::sin(delta) / length_;

        stheta = std::sin(theta + delta);
        ctheta = std::cos(theta + delta);
        ds_dtheta = +std::cos(theta + delta);
        dc_dtheta = -std::sin(theta + delta);
        ds_ddelta = ds_dtheta;
        dc_ddelta = dc_dtheta;
        break;
    };
    J.setZero();
    J(0, 2) = v * dc_dtheta; // dxdot_dtheta
    J(0, 3) = v * dc_ddelta; // dxdot_ddelta
    J(0, 4) = ctheta;        // dxdot_dv
    J(1, 2) = v * ds_dtheta; // dydot_dtheta
    J(1, 3) = v * ds_ddelta; // dydot_ddelta
    J(1, 4) = stheta;        // dydot_dv
    J(2, 3) = domega_ddelta;
    J(2, 4) = domega_dv;
    J(3, 5) = 1.0;
}
