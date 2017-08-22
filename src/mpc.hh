//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   26.05.2017
//------------------------------------------------------------------------------

#pragma once

#include <vector>
#include <Eigen/Core>
#include <cstdint>

//------------------------------------------------------------------------------
//! Model predictive controller
//------------------------------------------------------------------------------
class MPC {
  public:
    struct Result {
      std::vector<Eigen::VectorXd> states;
      std::vector<Eigen::VectorXd> actuations;
    };
    //--------------------------------------------------------------------------
    //! Constructor
    //!
    //! @param speed desired speed
    //! @param n     number of prediction steps
    //! @param dt    time step
    //! @param lf    distance between the front of the vehicle and its center
    //!              of gravity
    //--------------------------------------------------------------------------
    MPC(double speed, uint16_t n, double dt, double lf = 2.6):
        speed_(speed), n_(n), dt_(dt), lf_(lf) {};

    //--------------------------------------------------------------------------
    //! Destructor
    //--------------------------------------------------------------------------
    virtual ~MPC() {};

    //--------------------------------------------------------------------------
    //! Solve the model given an initial state. Return the next state and
    //! actuations as a vector.
    //--------------------------------------------------------------------------
    Result Solve(const Eigen::VectorXd &state,
                 const Eigen::VectorXd &trajectory);
  private:
    double   speed_;
    uint16_t n_;
    double   dt_;
    double   lf_;
};
