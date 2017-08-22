//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   06.08.2017
//------------------------------------------------------------------------------

#pragma once

#include "map.hh"
#include "behavior_planner.hh"
#include "path_predictor.hh"

#include <chrono>
#include <Eigen/Core>

//------------------------------------------------------------------------------
//! Auto pilot
//------------------------------------------------------------------------------
class AutoPilot {
  public:
    //--------------------------------------------------------------------------
    //! Constructor
    //!
    //! @param map         map of the road
    //! @param speed_limit speed_limit in m/s
    //--------------------------------------------------------------------------
    AutoPilot(const Map &map, double speed_limit):
        map_(map),  predictor_(map, 100),
        planner_(map, predictor_, speed_limit), speed_limit_(speed_limit) {}

    //--------------------------------------------------------------------------
    //! Destructor
    //--------------------------------------------------------------------------
    virtual ~AutoPilot() {}

    //--------------------------------------------------------------------------
    //! Compute a new path based on a behavior plan and the current state of
    //! the system.
    //--------------------------------------------------------------------------
    std::vector<Eigen::VectorXd> ComputePath(
        const Vehicle                      &v,
        const std::vector<SFObject>        &sensor_fusion,
        const std::vector<Eigen::VectorXd> &previous_path);

  private:
    //--------------------------------------------------------------------------
    // Compute the actual path
    //--------------------------------------------------------------------------
    std::vector<Eigen::VectorXd> ComputePath(
        const Vehicle                      &v,
        const std::vector<SFObject>        &sensor_fusion);

    //--------------------------------------------------------------------------
    // Data members
    //--------------------------------------------------------------------------
    const Map                                          &map_;
    PathPredictor                                       predictor_;
    BehaviorPlanner                                     planner_;
    Plan                                                plan_ = Plan::kStartUp;
    double                                              speed_limit_;
    int                                                 current_lane_;
    std::vector<Eigen::VectorXd>                        previous_path_;
    std::chrono::time_point<std::chrono::steady_clock>  previous_path_ts_;
    std::chrono::time_point<std::chrono::steady_clock>  previous_behavior_ts_;
};
