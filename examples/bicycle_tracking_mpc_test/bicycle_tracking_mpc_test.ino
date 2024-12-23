#include <altro_solver.hpp>
#include "altro/solver/solver.hpp"
#include "altro/utils/formatting.hpp"

#include <vector>
#include <ArduinoEigenDense.h>
#include "bicycle_model.h"
#include "scotty_trajectory.h"

#define M_PI 3.14159265358979323846
using namespace altro;

constexpr int n = BicycleModel::NumStates;
constexpr int m = BicycleModel::NumInputs;
constexpr int N = 30;
float h;

Vector Qd;
Vector Rd;
Vector Qdf;

ALTROSolver solver(N);

// Reference Trajectory (the "Scotty Dog")
std::vector<Eigen::Vector4d> x_ref;
std::vector<Eigen::Vector2d> u_ref;
Vector u0;

ErrorCodes err;

// Dynamics
auto model_ptr = std::make_shared<BicycleModel>();
ContinuousDynamicsFunction dyn0 = [model_ptr](double *x_dot, const double *x, const double *u)
{
    model_ptr->Dynamics(x_dot, x, u);
};
ContinuousDynamicsJacobian jac0 = [model_ptr](double *jac, const double *x, const double *u)
{
    model_ptr->Jacobian(jac, x, u);
};
auto dyn = MidpointDynamics(n, m, dyn0);
auto jac = MidpointJacobian(n, m, dyn0, jac0);

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println(F("Starting MPC Test"));
    Qd = Vector::Constant(n, 1e-2);
    Rd = Vector::Constant(m, 1e-3);
    Qdf = Vector::Constant(n, 1e1);
    // Dimension and Time step
    err = solver.SetDimension(n, m);
    if (err != ErrorCodes::NoError)
        for (Serial.println(F("Set Dimension Error"));;)
            ;

    // Dynamics
    err = solver.SetExplicitDynamics(dyn, jac);
    if (err != ErrorCodes::NoError)
        for (Serial.println(F("Set Explicit Dynamics Error"));;)
            ;

    // Read Reference Trajectory
    int N_ref;
    float t_ref;
    ReadScottyTrajectory(&N_ref, &t_ref, &x_ref, &u_ref);

    // Set time step equal to the reference trajectory
    h = t_ref / static_cast<double>(N_ref);
    err = solver.SetTimeStep(h);
    if (err != ErrorCodes::NoError)
        for (Serial.println(F("Set Time Step Error"));;)
            ;

    // Set Tracking Cost Function
    for (int k = 0; k <= N; ++k)
    {
        solver.SetLQRCost(n, m, Qd.data(), Rd.data(), x_ref.at(k).data(), u_ref.at(k).data(), k);
    }

    // Constraints
    auto steering_angle_con = [](a_float *c, const a_float *x, const a_float *u)
    {
        (void)u;
        a_float delta_max = 60 * M_PI / 180.0;
        c[0] = x[3] - delta_max;
        c[1] = -delta_max - x[3];
    };
    auto steering_angle_jac = [](a_float *jac, const a_float *x, const a_float *u)
    {
        (void)x;
        (void)u;
        Eigen::Map<Eigen::Matrix<a_float, 2, 6>> J(jac);
        J.setZero();
        J(0, 3) = 1.0;
        J(1, 3) = -1.0;
    };
    err = solver.SetConstraint(steering_angle_con, steering_angle_jac, 2, ConstraintType::INEQUALITY,
                               "steering angle bound", 0, N + 1);
    if (err != ErrorCodes::NoError)
        for (Serial.println(F("Set Constraint Error"));;)
            ;
    // Initial State
    err = solver.SetInitialState(x_ref[0].data(), n);
    if (err != ErrorCodes::NoError)
        for (Serial.println(F("Set Initial State Error"));;)
            ;

    // Initialize Solver
    err = solver.Initialize();
    if (err != ErrorCodes::NoError)
        for (Serial.println(F("Solver Initialization Error"));;)
            ;

    // Set Initial Trajectory
    a_float average_speed = u_ref[0][0];
    u0 = Vector::Zero(m);
    u0 << average_speed, 0.0;
    err = solver.SetInput(u0.data(), m);
    if (err != ErrorCodes::NoError)
        for (Serial.println(F("Set Input Error"));;)
            ;
    for (int k = 0; k <= N; ++k)
    {
        err = solver.SetState(x_ref.at(k).data(), n, k);
        if (err != ErrorCodes::NoError)
            for (Serial.println("Set State Error at step" + String(k));;)
                ;
    }

    // Solve
    // MPC Setup
    int Nsim = 200;
    int mpc_iter = 0;
    std::vector<Vector> x_sim;
    std::vector<Vector> u_sim;
    std::vector<int> solve_iters;
    std::vector<double> tracking_error;
    x_sim.reserve(Nsim + 1);
    u_sim.reserve(Nsim);
    solve_iters.reserve(Nsim);
    tracking_error.reserve(Nsim);
    x_sim.emplace_back(x_ref[0]); // push initial state to the front
    for (int i = 0; i < Nsim; ++i)
    {
        x_sim.emplace_back(Vector::Zero(n));
        u_sim.emplace_back(Vector::Zero(m));
    }

    // Solve Options
    AltroOptions opts;
    opts.verbose = Verbosity::Silent;
    opts.iterations_max = 80;
    opts.use_backtracking_linesearch = true;
    solver.SetOptions(opts);

    // Initialize variables
    SolveStatus status;
    a_float c_u = 0.5 * u0.dot(Rd.asDiagonal() * u0);
    a_float c;
    Vector q(n);
    Vector u_mpc(m);
    unsigned long t_start = millis();
    while (mpc_iter < Nsim)
    {
        // Solve nonlinear MPC problem
        status = solver.Solve();
        if (status != SolveStatus::Success)
            for (Serial.println(F("MPC Solve Failed"));;)
                ;
        solve_iters.emplace_back(solver.GetIterations());

        // Get control
        solver.GetInput(u_sim[mpc_iter].data(), 0);

        // Simulate the system forward
        dyn(x_sim[mpc_iter + 1].data(), x_sim[mpc_iter].data(), u_sim[mpc_iter].data(), h);

        // Get error from reference
        tracking_error.emplace_back((x_sim[mpc_iter + 1] - x_ref[mpc_iter + 1]).norm());
        //    fmt::print("mpc iter {}: err = {}, solve iters = {}\n", mpc_iter, tracking_error, solve_iters);

        // Set new reference trajectory
        ++mpc_iter;
        for (int k = 0; k <= N; ++k)
        {
            const Vector &xk_ref = x_ref[k + mpc_iter];
            q.noalias() = Qd.asDiagonal() * xk_ref;
            q *= -1;
            c = 0.5 * q.dot(xk_ref);
            c *= -1;
            if (k < N)
            {
                c += c_u;
            }
            solver.UpdateLinearCosts(q.data(), nullptr, c, k);
        }

        // Set Initial state
        solver.SetInitialState(x_sim[mpc_iter].data(), n);

        // Shift the trajectory
        solver.ShiftTrajectory();
    }
    unsigned long t_end = millis();
    Serial.println(F("MPC Solve Completed!"));
    Serial.print(F("Time Spent = "));
    Serial.print(t_end - t_start);
    Serial.println(F(" ms"));
    Serial.print(F("Average Rate = "));
    Serial.print(Nsim / (t_end / 1000 - t_start / 1000));
    Serial.println(F(" Hz"));
}

void loop()
{
    // put your main code here, to run repeatedly:
}
