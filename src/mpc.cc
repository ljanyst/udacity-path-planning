//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   26.05.2017
//------------------------------------------------------------------------------

#include "mpc.hh"
#define HAVE_CSTDDEF
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

//------------------------------------------------------------------------------
// Vector and accessors definitions
//------------------------------------------------------------------------------
typedef CPPAD_TESTVECTOR(double) VectorD;

#define S_X     0
#define S_Y     1
#define S_PSI   2
#define S_V     3
#define S_CTE   4
#define S_EPSI  5
#define A_DELTA 0
#define A_A     1

#define IND_S(VAL, N, I) ((VAL)*(N)+(I))
#define IND_A(VAL, N, I) ((6*N)+(VAL)*(N-1)+(I))

//------------------------------------------------------------------------------
//! Model evaluator
//------------------------------------------------------------------------------
class FGEval {
  public:
    //--------------------------------------------------------------------------
    //! Constructor
    //--------------------------------------------------------------------------
    FGEval(const Eigen::VectorXd &trajectory, double n, double dt, double lf,
           double ref_speed):
      trajectory_(trajectory), n_(n), dt_(dt), lf_(lf), ref_speed_(ref_speed) {}

    //--------------------------------------------------------------------------
    // Vector type typedef - needed by the solver at this exact name
    //--------------------------------------------------------------------------
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    //--------------------------------------------------------------------------
    //! Evaluate the model
    //!
    //! @param fg   vector for the result of cost function and model values
    //! @param vars variables (state and actuators)
    //--------------------------------------------------------------------------
    void operator()(ADvector &fg, const ADvector &vars) {
      //------------------------------------------------------------------------
      // Compute the cost function, the cost is stored in the first element of
      // fg.
      //------------------------------------------------------------------------
      fg[0] = 0;

      // The part of the cost based on the reference state. We punish every
      // cross track error, deviation from expected orientation, end deviation
      // from the desired speed
      for(int i = 0; i < n_; ++i) {
        fg[0] += CppAD::pow(vars[IND_S(S_CTE,  n_, i)], 2);
        fg[0] += CppAD::pow(vars[IND_S(S_EPSI, n_, i)], 2);
        fg[0] += CppAD::pow(vars[IND_S(S_V,    n_, i)] - ref_speed_, 2);
      }

      // Minimize the value gap between sequential actuations.
      for(int i = 0; i < n_-2; ++i) {
        fg[0] += 500000*CppAD::pow(vars[IND_A(A_DELTA, n_, i+1)] - vars[IND_A(A_DELTA, n_, i)], 2);
        fg[0] += 250000*CppAD::pow(vars[IND_A(A_A,     n_, i+1)] - vars[IND_A(A_A,     n_, i)], 2);
      }

      //------------------------------------------------------------------------
      // The state at time 0; we use the AD thing here because the solver needs
      // to know the derivatives
      //------------------------------------------------------------------------
      AD<double> x0    = vars[IND_S(S_X,    n_, 0)];
      AD<double> y0    = vars[IND_S(S_Y,    n_, 0)];
      AD<double> psi0  = vars[IND_S(S_PSI,  n_, 0)];
      AD<double> v0    = vars[IND_S(S_V,    n_, 0)];
      AD<double> cte0  = vars[IND_S(S_CTE,  n_, 0)];
      AD<double> epsi0 = vars[IND_S(S_EPSI, n_, 0)];

      //------------------------------------------------------------------------
      // Iterate over time steps to compute constraints
      //------------------------------------------------------------------------
      for(int i = 0; i < n_-1; ++i) {
        // The state at time t+1
        AD<double> x1      = vars[IND_S(S_X,    n_, i+1)];
        AD<double> y1      = vars[IND_S(S_Y,    n_, i+1)];
        AD<double> psi1    = vars[IND_S(S_PSI,  n_, i+1)];
        AD<double> v1      = vars[IND_S(S_V,    n_, i+1)];
        AD<double> cte1    = vars[IND_S(S_CTE,  n_, i+1)];
        AD<double> epsi1   = vars[IND_S(S_EPSI, n_, i+1)];

        // The actuation at time t
        AD<double> delta0  = vars[IND_A(A_DELTA, n_, i)];
        AD<double> a0      = vars[IND_A(A_A,     n_, i)];

        // evaluate the polynomial
        AD<double> f0      = trajectory_[0] +
                             trajectory_[1] * x0 +
                             trajectory_[2] * x0 * x0 +
                             trajectory_[3] * x0 * x0 * x0;

        // compute the desired orientation
        AD<double> psides0 = CppAD::atan(    trajectory_[1] +
                                         2 * trajectory_[2] * x0 +
                                         3 * trajectory_[3] * x0 * x0);

        //----------------------------------------------------------------------
        // Compute the model and the difference between the actual state and the
        // expected state as the value for the constraint
        //----------------------------------------------------------------------
        fg[1 + IND_S(S_X,    n_-1, i)] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt_);
        fg[1 + IND_S(S_Y,    n_-1, i)] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt_);
        fg[1 + IND_S(S_PSI,  n_-1, i)] = psi1 - (psi0 + v0 * delta0 / lf_ * dt_);
        fg[1 + IND_S(S_V,    n_-1, i)] = v1 - (v0 + a0 * dt_);
        fg[1 + IND_S(S_CTE,  n_-1, i)] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt_));
        fg[1 + IND_S(S_EPSI, n_-1, i)] = epsi1 - ((psi0 - psides0) + v0 * delta0 / lf_ * dt_);

        //----------------------------------------------------------------------
        // Remember the state
        //----------------------------------------------------------------------
        x0    = x1;
        y0    = y1;
        psi0  = psi1;
        v0    = v1;
        cte0  = cte1;
        epsi0 = epsi1;
      }
    }
  private:
    Eigen::VectorXd trajectory_;
    double          n_;
    double          dt_;
    double          lf_;
    double          ref_speed_;
};

//------------------------------------------------------------------------------
// Solve the model given an initial state
//------------------------------------------------------------------------------
MPC::Result MPC::Solve(const Eigen::VectorXd &state,
                       const Eigen::VectorXd &trajectory) {

  //----------------------------------------------------------------------------
  // We use a model where we are predicting the state for n_ steps. This gives
  // us n_ vectors of 6 elements. We also want to predict n_-1 actuations - ie.
  // transitions between steps, this gives us n_-1 vectors of 2 elements (the
  // steering angle and the accelleration/throttle).
  //
  // We start with the previously known state to ensure smoothnes of transition
  // between the previously computed path and the extended one
  //----------------------------------------------------------------------------
  size_t n_vars = n_ * 6 + (n_-1) * 2;

  VectorD vars(n_vars);
  for(int i = 0; i < n_vars; ++i)
    vars[i] = 0.0;

  vars[IND_S(S_X,    n_, 0)] = state[0];
  vars[IND_S(S_Y,    n_, 0)] = state[1];
  vars[IND_S(S_PSI,  n_, 0)] = state[2];
  vars[IND_S(S_V,    n_, 0)] = state[3];
  vars[IND_S(S_CTE,  n_, 0)] = state[4];
  vars[IND_S(S_EPSI, n_, 0)] = state[5];

  //----------------------------------------------------------------------------
  // The variables of the model are independently unconstrained, except for the
  // first k steps, where they must stay what they are; the acutators must stay
  // within boundries though. The upper and lower limits of delta are [-25, 25]
  // degrees (values in radians). Accelleration must be within [-3, 3].
  //----------------------------------------------------------------------------
  VectorD vars_lowerbound(n_vars);
  VectorD vars_upperbound(n_vars);

  for(int i = 0; i < n_; ++i) {
    for(int j = S_X; j <= S_EPSI; ++j) {
      vars_lowerbound[IND_S(j, n_, i)] = -1.0e19;
      vars_upperbound[IND_S(j, n_, i)] =  1.0e19;
    }
  }

  vars_lowerbound[IND_S(S_X,    n_, 0)] = vars_upperbound[IND_S(S_X,    n_, 0)] = state[0];
  vars_lowerbound[IND_S(S_Y,    n_, 0)] = vars_upperbound[IND_S(S_Y,    n_, 0)] = state[1];
  vars_lowerbound[IND_S(S_PSI,  n_, 0)] = vars_upperbound[IND_S(S_PSI,  n_, 0)] = state[2];
  vars_lowerbound[IND_S(S_V,    n_, 0)] = vars_upperbound[IND_S(S_V,    n_, 0)] = state[3];
  vars_lowerbound[IND_S(S_CTE,  n_, 0)] = vars_upperbound[IND_S(S_CTE,  n_, 0)] = state[4];
  vars_lowerbound[IND_S(S_EPSI, n_, 0)] = vars_upperbound[IND_S(S_EPSI, n_, 0)] = state[5];

  for(int i = 0; i < n_-1; ++i) {
    vars_lowerbound[IND_A(A_DELTA, n_, i)] = -0.05;
    vars_upperbound[IND_A(A_DELTA, n_, i)] =  0.05;
    vars_lowerbound[IND_A(A_A,     n_, i)] = -3.0;
    vars_upperbound[IND_A(A_A,     n_, i)] =  3.0;
  }

  //----------------------------------------------------------------------------
  // Furthermore, the variables of the model, must follow the model at each
  // step, and must be derived from the variables of the model at the previous
  // step. This gives use n_-1 costraints. We will represent this as a
  // difference between the actual variable and the value predicted according
  // to the model with variable actuators for the given state. The values must
  // be the same, so the difference (and, threfore, the constraint) is 0.
  //----------------------------------------------------------------------------
  size_t n_constraints = (n_-1) * 6;

  VectorD constraints_lowerbound(n_constraints);
  VectorD constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  //----------------------------------------------------------------------------
  // Set up the solver an run it
  //----------------------------------------------------------------------------
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true         forward\n";
  options += "Sparse  true         reverse\n";
  options += "Numeric max_cpu_time 1.\n";

  CppAD::ipopt::solve_result<VectorD> solution;
  FGEval eval(trajectory, n_, dt_, lf_, speed_);

  CppAD::ipopt::solve<VectorD, FGEval>(
    options, vars, vars_lowerbound, vars_upperbound,
    constraints_lowerbound, constraints_upperbound,
    eval, solution);

  if(solution.status != CppAD::ipopt::solve_result<VectorD>::success)
    throw std::runtime_error("solver failed");

  //----------------------------------------------------------------------------
  // Convert the result
  //----------------------------------------------------------------------------
  Result ret;

  for(int i = 0; i < n_; ++i) {
    Eigen::VectorXd pt(2);
    pt << solution.x[IND_S(S_X,    n_, i)],
          solution.x[IND_S(S_Y,    n_, i)],
    ret.states.push_back(pt);
  }

  for(int i = 0; i < n_-1; ++i) {
    Eigen::VectorXd pt(2);
    pt << solution.x[IND_A(A_A,     n_, i)],
          solution.x[IND_A(A_DELTA, n_, i)];
    ret.actuations.push_back(pt);
  }

  return ret;
}
