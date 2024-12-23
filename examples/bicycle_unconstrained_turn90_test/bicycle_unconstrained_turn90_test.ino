#include <altro_solver.hpp>
#include "altro/solver/solver.hpp"
#include "altro/utils/formatting.hpp"

#include <ArduinoEigenDense.h>
#include "bicycle_model.h"
#define M_PI_2 1.57079632679489661923
using namespace altro;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  const int n = BicycleModel::NumStates;
  const int m = BicycleModel::NumInputs;
  const int N = 30;
  const float tf = 3.0;
  const float h = tf / static_cast<double>(N);

  // Objective
  Vector Qd = Vector::Constant(n, 1e-2);
  Vector Rd = Vector::Constant(m, 1e-3);
  Vector Qdf = Vector::Constant(n, 1e1);
  Vector x0(n);
  Vector xf(n);
  Vector uf(m);
  x0.setZero();
  xf << 1, 2, M_PI_2, 0.0;
  uf.setZero();

  // Dynamics
  auto model_ptr = std::make_shared<BicycleModel>();
  ContinuousDynamicsFunction dyn0 = [model_ptr](double *x_dot, const double *x, const double *u) {
    model_ptr->Dynamics(x_dot, x, u);
  };
  ContinuousDynamicsJacobian jac0 = [model_ptr](double *jac, const double *x, const double *u) {
    model_ptr->Jacobian(jac, x, u);
  };
  auto dyn = MidpointDynamics(n, m, dyn0);
  auto jac = MidpointJacobian(n, m, dyn0, jac0);

  // Define the problem
  ALTROSolver solver(N);

  // Dimension and Time step
  solver.SetDimension(n, m);
  solver.SetTimeStep(h);

  // Dynamics
  solver.SetExplicitDynamics(dyn, jac);

  // Set Cost Function
  solver.SetLQRCost(n, m, Qd.data(), Rd.data(), xf.data(), uf.data(), 0, N);
  solver.SetLQRCost(n, m, Qdf.data(), Rd.data(), xf.data(), uf.data(), N);

  // Initial State
  solver.SetInitialState(x0.data(), n);
  // Initial Solver
  solver.Initialize();

  // Set initial trajectory
  Vector u0 = Vector::Constant(m, 0.0);
  u0 << 0.5, 0;
  solver.SetInput(u0.data(), m);
  solver.OpenLoopRollout();
  Vector x(n);
  solver.GetState(x.data(), N);
  PrintVectorRow("xN = ", x);

  // Initial Cost
  a_float cost = solver.CalcCost();
  Serial.print(F("Initial cost = "));
  Serial.println(cost);

  // Solve
  unsigned long t_start = millis();
  AltroOptions opts;
  opts.verbose = Verbosity::Outer;
  opts.iterations_max = 80;
  opts.use_backtracking_linesearch = true;
  solver.SetOptions(opts);
  SolveStatus status = solver.Solve();
  unsigned long t_end = millis();
  Serial.print(F("status = "));
  Serial.println((int)status);
  Serial.print(F"Time Spent = ");
  Serial.print(t_end - t_start)
  Serial.println(F(" ms"));

  // Final state
  Vector xN(n);
  solver.GetState(xN.data(), N);
  PrintVectorRow("xN = ", xN);
  if ((xN - xf).norm() < 1e-2)
  {
    Serial.println(F("Test passed"));
  }
  else
  {
    Serial.println(F("Test failed"));
  }
}

void loop()
{
  // put your main code here, to run repeatedly:
}
