#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.1; // change in time in seconds

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t steer_angle_start = epsi_start + N;
size_t throttle_start = steer_angle_start + (N - 1);

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    fg[0] = 0;

    double target_vel = 30; //target car velocity
    // cost based of the reference states
    for(int i=0; i < N; i++) {
      fg[0] += 2000*CppAD::pow(vars[cte_start + i], 2);
      fg[0] += 2000*CppAD::pow(vars[epsi_start + i], 2);
      fg[0] += CppAD::pow(vars[v_start + i] - target_vel, 2);
    }

    // minimise the use of the controls
    for(int i=0; i < (N-1); i++) {
      fg[0] += 5*CppAD::pow(vars[steer_angle_start + i], 2);
      fg[0] += 5*CppAD::pow(vars[throttle_start + i], 2);
    }
    
    // minimise the rate of change in the controls
    for(int i=0; i < (N-2); i++) {
      fg[0] += 200*CppAD::pow(vars[steer_angle_start+i+1] - vars[steer_angle_start+i], 2);
      fg[0] += 10*CppAD::pow(vars[throttle_start+i+1] - vars[throttle_start+i], 2);
    }

    // initial constraints
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    for(int i=0; i < N-1; i++) {
      // next time step t+1
      AD<double> x1 = vars[x_start + i+1];
      AD<double> y1 = vars[y_start + i+1];
      AD<double> psi1 = vars[psi_start + i+1];
      AD<double> v1 = vars[v_start + i+1];
      AD<double> cte1 = vars[cte_start + i+1];
      AD<double> epsi1 = vars[epsi_start + i+1];
      // current time step t-1
      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> psi0 = vars[psi_start + i];
      AD<double> v0 = vars[v_start + i];
      AD<double> cte0 = vars[cte_start + i];
      AD<double> epsi0 = vars[epsi_start + i];
      
      AD<double> steer_angle0 = vars[steer_angle_start + i];
      AD<double> throttle0 = vars[throttle_start + i]; // acceleration

      // vehicle motion model function f = v + a*x
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
      AD<double> psi_target = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0);
      
      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // NOTE: The use of `AD<double>` and use of `CppAD`!
      // This is also CppAD can compute derivatives and pass
      // these to the solver.
 
      //std::cout<<"operator() 6"<<std::endl;
      // TODO: Setup the rest of the model constraints
      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 + v0 / Lf * steer_angle0 * dt);
      fg[2 + v_start + i] = v1 - (v0 + throttle0 * dt);
      fg[2 + cte_start + i] = cte1 - (f0 - y0 + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] = epsi1 - ((psi0 - psi_target) + (v0 / Lf * steer_angle0 * dt));
    }
    
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = 6 * N + 2 * (N-1);
  // TODO: Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  
  // TODO: Set lower and upper limits for variables.
  // Lower and upper limits for the constraints and variables
  // Should be 0 besides initial constraints state.
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    vars_lowerbound[x_start + i] = -1.0e19;
    vars_upperbound[x_start + i] = 1.0e19;
    
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  // Lower and upper limits for the control variables
  for(int i=0; i < (N-1); i++) {
    // control steering angle limits
    vars_lowerbound[steer_angle_start + i] = -25;//0.436332 * Lf;
    vars_upperbound[steer_angle_start + i] = 25;//0.436332 * Lf;
    // control throttle limits
    vars_lowerbound[throttle_start + i] = -1.0;
    vars_upperbound[throttle_start + i] = 1.0;
  }
  
  constraints_lowerbound[x_start] = state[0];   // x0
  constraints_lowerbound[y_start] = state[1];   // y0
  constraints_lowerbound[psi_start] = state[2]; // psi0 
  constraints_lowerbound[v_start] = state[3];   // v0
  constraints_lowerbound[cte_start] = state[4]; // cte0
  constraints_lowerbound[epsi_start] = state[5];// epsi0 

  constraints_upperbound[x_start] = state[0];   // x0
  constraints_upperbound[y_start] = state[1];   // y0
  constraints_upperbound[psi_start] = state[2]; // psi0 
  constraints_upperbound[v_start] = state[3];   // v0
  constraints_upperbound[cte_start] = state[4]; // cte0
  constraints_upperbound[epsi_start] = state[5];// epsi0 

  // Set the initial variable values
  //vars[x_start] = state[0];   // x0
  //vars[y_start] = state[1];   // y0
  //vars[psi_start] = state[2]; // psi0 
  //vars[v_start] = state[3];   // v0
  //vars[cte_start] = state[4]; // cte0
  //vars[epsi_start] = state[5];// epsi0 
  
  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> result;
  result.push_back(solution.x[steer_angle_start]);
  result.push_back(solution.x[throttle_start]);
  for(int i=1; i < N; i++) {
    //std::cout<<"Solve() 8, "<<i<<std::endl;
    result.push_back(solution.x[x_start + i]);
    result.push_back(solution.x[y_start + i]);
  }
  
  return result;//{solution.x[x_start + 1], solution.x[y_start + 1], solution.x[psi_start + 1],
      //solution.x[v_start + 1], solution.x[cte_start + 1], solution.x[epsi_start + 1],
      //solution.x[steer_angle_start + 1], solution.x[throttle_start + 1]};
}
