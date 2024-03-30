 
// This file was automatically generated by autogenu-jupyter (https://github.com/ohtsukalab/autogenu-jupyter). 
// The autogenu-jupyter copyright holders make no ownership claim of its contents. 

#include "ocp.hpp"
#include "cgmres/zero_horizon_ocp_solver.hpp"
#include "cgmres/single_shooting_cgmres_solver.hpp" 

#include "cgmres/logger.hpp"
#include "cgmres/integrator.hpp"
#include <string>

int main() {
  // Define the optimal control problem.
  cgmres::OCP_trajectory_following ocp;

  // Define the horizon.
  const double Tf = 0.1;
  const double alpha = 0.0;
  cgmres::Horizon horizon(Tf, alpha);

  // Define the solver settings.
  cgmres::SolverSettings settings;
  settings.sampling_time = 0.03; // sampling period 
  settings.zeta = 33.3;
  settings.finite_difference_epsilon = 1e-08;
  // For initialization.
  settings.max_iter = 50;
  settings.opterr_tol = 1e-06;

  // Define the initial time and initial state.
  const double t0 = 0;
  cgmres::Vector<3> x0;
  x0 << 0, 0, 0;

  // Initialize the solution of the C/GMRES method.
  constexpr int kmax_init = 1;
  cgmres::ZeroHorizonOCPSolver<cgmres::OCP_trajectory_following, kmax_init> initializer(ocp, settings);
  cgmres::Vector<1> uc0;
  uc0 << 0.0;
  initializer.set_uc(uc0);
  initializer.solve(t0, x0);

  // Define the C/GMRES solver.
  constexpr int N = 50;
  constexpr int kmax = 5;
  cgmres::SingleShootingCGMRESSolver<cgmres::OCP_trajectory_following, N, kmax> mpc(ocp, horizon, settings);
  mpc.set_uc(initializer.ucopt());
  mpc.init_dummy_mu();

  // Perform a numerical simulation.
  const double tsim = 100; 
  const double sampling_time = settings.sampling_time;
  const unsigned int sim_steps = std::floor(tsim / sampling_time);

  double t = t0;
  cgmres::VectorX x = x0;
  cgmres::VectorX dx = cgmres::VectorX::Zero(x0.size());

  const std::string log_name("../log/trajectory_following"); 
  cgmres::Logger logger(log_name);

  std::cout << "Start a simulation..." << std::endl;
  for (unsigned int i=0; i<sim_steps; ++i) {
    std::cerr << "sim_steps: " << i << std::endl;
    const auto& u = mpc.uopt()[0]; // const reference to the initial optimal control input 
    const cgmres::VectorX x1 = cgmres::RK4(ocp, t, sampling_time, x, u); // the next state
    mpc.update(t, x); // update the MPC solution

    mpc.optError(t,x,settings.verbose_level); // compute the optimal error
    logger.save(t, x, u, mpc.uopt(), mpc.optError(), mpc.normDiff(),mpc.StandardDeviation());
    x = x1;
    t = t + sampling_time;
  }
  std::cout << "End the simulation" << std::endl;
  std::cout << std::endl;

  logger.save(mpc.getProfile());

  std::cout << "MPC used in this simulation:" << std::endl;
  std::cout << mpc << std::endl;

  return 0;
}
