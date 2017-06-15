#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

size_t N = 10; // number of future states we will predict
double dt = 0.1; // time duration between two adjacent states, in second.
const double Lf = 2.67;
double v_ref = 70.0; // reference speed

size_t n_state = 6;         // number of state variables: x, y, psi, v, cte, and epsi
size_t n_actuators = 2;     // number of actuators: delta and a
size_t n_vars = n_state * N + n_actuators * (N - 1);  // size of vars vector (as explained in the FG_eval class)
size_t n_constraints = n_state * N;     // size of fg vector (as explained in the FG_eval class)

// weights of different cost terms
double w_cte = 500.0;      // weight of cross track error
double w_epsi = 500.0;     // weight of orientation error
double w_v = 1.0;           // weight of velocity error
double w_delta = 100.0;    // weight of large steering cost
double w_a = 5.0;          // weight of large throttle cost
double w_ddelta = 100.0;   // weight of large steering changing rate cost
double w_da = 1.0;       // weight of large throttle changing rate cost

// starting position of different state variables
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

// upper and lower limits of variables
double l_state = 1.0e19;           // limit of x, y, v, cte, epsi
double l_psi = 1.9635;             // limit of psi (5/8 * PI)
double l_delta = 0.4363 * Lf;           // limit of steer (25 degree)
double l_a = 1.0;                  // limit of acceleration

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // fg is a vector of constraints. if the state has 6 elements (4 state variables and 2 errors),
    // fg would have 6N + 1 elements.
    // fg[0] stores the cost function; fg[1...6N] are constrains for all state variables at all
    // N steps.
    // vars is a vector of variables. if the state is a 6 element vector and the actuator is a 2
    // element vector, vars would have 6N + 2(N - 1) elements, storing all state and actuator
    // variables at all N steps.
    // the operator() method is to assign all fg elements given vars.

    //*********************************************
    // Calculate cost and assign the value to fg[0]
    //*********************************************
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
        fg[0] += w_cte * CppAD::pow(vars[cte_start + t], 2);
        fg[0] += w_epsi * CppAD::pow(vars[epsi_start + t], 2);
        fg[0] += w_v * CppAD::pow(vars[v_start + t] - v_ref, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
        fg[0] += w_delta * CppAD::pow(vars[delta_start + t], 2);
        fg[0] += w_a * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
        fg[0] += w_ddelta * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        fg[0] += w_da * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    //**********************************************************
    // Setup Constraints and assign them to fg[1...n_constrains]
    //**********************************************************

    // Constraints of initial state (which are simply the corresponding state variables)
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 0; t < N - 1; t++) {
        // The state at time t .
        AD<double> x0 = vars[x_start + t];
        AD<double> y0 = vars[y_start + t];
        AD<double> psi0 = vars[psi_start + t];
        AD<double> v0 = vars[v_start + t];
        AD<double> cte0 = vars[cte_start + t];
        AD<double> epsi0 = vars[epsi_start + t];

        // The state at time t + 1.
        AD<double> x1 = vars[x_start + t + 1];
        AD<double> y1 = vars[y_start + t + 1];
        AD<double> psi1 = vars[psi_start + t + 1];
        AD<double> v1 = vars[v_start + t + 1];
        AD<double> cte1 = vars[cte_start + t + 1];
        AD<double> epsi1 = vars[epsi_start + t + 1];

        // Only consider the actuation at time t.
        AD<double> delta0 = vars[delta_start + t];
        AD<double> a0 = vars[a_start + t];

        AD<double> f0 = coeffs[0] + coeffs[1] * x0  + coeffs[2] * x0 * x0\
                        + coeffs[3] * x0 * x0 * x0;
        AD<double> psides0 = CppAD::atan(coeffs[1] + 2.0 * coeffs[2] * x0 + 3.0 * coeffs[3] * x0 * x0);

        // Here's `x` to get you started.
        // The idea here is to constraint this value to be 0.
        //
        // Recall the equations for the model:
        // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
        // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
        // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
        // v_[t+1] = v[t] + a[t] * dt
        // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
        // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
        fg[2 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
        fg[2 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
        fg[2 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
        fg[2 + v_start + t] = v1 - (v0 + a0 * dt);
        fg[2 + cte_start + t] =
                cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
        fg[2 + epsi_start + t] =
                epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  typedef CPPAD_TESTVECTOR(double) Dvector;

  //*******************************
  // Initialize vars
  //*******************************
  Dvector vars(n_vars);

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  for (int i = 0; i < n_vars; i++) {
      vars[i] = 0;
  }
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  //*************************************
  // Set lower and upper bounds for vars
  //*************************************
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set limits to state variables (they essentially don't have bounds),
  // therefore l_state is a very large number
  for (size_t i = 0; i < delta_start; ++i) {
      vars_lowerbound[i] = -l_state;
      vars_upperbound[i] = l_state;
  }

  // Limit psi to screen out some unrealistic solutions.
  for (size_t i = psi_start; i < v_start; ++i) {
      vars_lowerbound[i] = -l_psi;
      vars_upperbound[i] = l_psi;
  }

  // Limit steering
  for (size_t i = delta_start; i < a_start; ++i) {
      vars_lowerbound[i] = -l_delta;
      vars_upperbound[i] = l_delta;
  }

  // Limit throttle
  for (size_t i = a_start; i < n_vars; i++) {
      vars_lowerbound[i] = -l_a;
      vars_upperbound[i] = l_a;
  }

  //********************************************
  // Set lower and upper bounds for constraints
  //********************************************
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  // Constrains should be 0 except initial state.
  for (int i = 0; i < n_constraints; i++) {
      constraints_lowerbound[i] = 0;
      constraints_upperbound[i] = 0;
  }
  // Constraints of initial states
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  //********************************************
  // Create a FG_eval object and solve
  //********************************************
  FG_eval fg_eval(coeffs);

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
  /*
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;
  */

  // Store the steer and throttle values
  steer = solution.x[delta_start];
  throttle = solution.x[a_start];

  // Store all N x and y values for visualization
  mpc_x = {};
  mpc_y = {};
  for (int i = 0; i < N; i++) {
      mpc_x.push_back(solution.x[x_start + i]);
      mpc_y.push_back(solution.x[y_start + i]);
  }

  return {};
}
