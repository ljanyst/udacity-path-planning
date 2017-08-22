//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   06.08.2017
//------------------------------------------------------------------------------

#include "auto_pilot.hh"
#include "poly.hh"
#include "utils.hh"
#include "mpc.hh"
#include "path_predictor.hh"

#include <chrono>
#include <iostream>

//------------------------------------------------------------------------------
// Constants
//------------------------------------------------------------------------------
const double kDT               = 0.02; // seconds
const int    kNumSharedPoints  = 25;
const int    kNumPoints        = 75;
const double kLF               = 2.6;  // distance in meters between the axles
                                       // of the car
const auto   kPathInterval     = std::chrono::milliseconds(250);
const auto   kBehaviorInterval = std::chrono::milliseconds(3000);
const double kObjectDistance   = 100;  // meters

//------------------------------------------------------------------------------
// Compute a new path
//------------------------------------------------------------------------------
std::vector<Eigen::VectorXd> AutoPilot::ComputePath(
    const Vehicle                      &v,
    const std::vector<SFObject>        &sensor_fusion,
    const std::vector<Eigen::VectorXd> &previous_path) {

  //----------------------------------------------------------------------------
  // Check whether we should compute a new path at this point or not
  //----------------------------------------------------------------------------
  using namespace std::chrono;
  previous_path_ = previous_path;
  auto now = steady_clock::now();

  if(duration_cast<milliseconds>(now-previous_path_ts_) < kPathInterval)
    return previous_path_;

  if(plan_ == Plan::kStartUp)
    current_lane_ = map_.Lane(v);

  //----------------------------------------------------------------------------
  // Do we need a new plan?
  //----------------------------------------------------------------------------
  predictor_.AddObjects(sensor_fusion, v);
  if(duration_cast<milliseconds>(now-previous_behavior_ts_) >= kBehaviorInterval) {
    plan_ = planner_.ComputeBehavior(v);
    previous_behavior_ts_ = now;
    std::cout << "Selected behavior: " << BehaviorPlanner::PlanToString(plan_);
    std::cout << std::endl;
    if(plan_ == Plan::kChangeLeft)
      current_lane_ -= 1;
    else if(plan_ == Plan::kChangeRight)
      current_lane_ += 1;
    plan_ = Plan::kKeepLane;
  }

  previous_path_ts_ = now;
  previous_path_    = ComputePath(v, sensor_fusion);
  return previous_path_;
}

//------------------------------------------------------------------------------
//! Compute the actual path
//------------------------------------------------------------------------------
std::vector<Eigen::VectorXd> AutoPilot::ComputePath(
    const Vehicle               &v,
    const std::vector<SFObject> &sensor_fusion) {

  //----------------------------------------------------------------------------
  // Declare some helpers
  //----------------------------------------------------------------------------
  Eigen::VectorXd              initial_state(6);
  std::vector<Eigen::VectorXd> new_path;
  Vehicle                      future_v = v;

  //----------------------------------------------------------------------------
  // Copy some of the points to the new path
  //----------------------------------------------------------------------------
  for(int i = 0; i < kNumSharedPoints && i < previous_path_.size(); ++i)
    new_path.push_back(previous_path_[i]);

  if(new_path.size() >= 2) {
    auto &prev0  = new_path.back();
    auto &prev1  = new_path[new_path.size()-2];
    future_v.xy  = prev0;
    future_v.yaw = std::atan2(prev0[1]-prev1[1], prev0[0]-prev1[0]);
    future_v.v   = std::sqrt((prev0-prev1).squaredNorm())/kDT;
  }

  auto   trajectory = map_.ComputeWaypointPoly(current_lane_, future_v);
  double cte        = PolyEval(trajectory, 0);
  double epsi       = -std::atan(trajectory[1]);

  initial_state << 0, 0, 0, future_v.v, cte, epsi;

  //----------------------------------------------------------------------------
  // Check if we have a leader
  //----------------------------------------------------------------------------
  double speed  = speed_limit_;
  auto   leader = predictor_.FindLeader(v, current_lane_);
  if(leader) {
    double dist_leader = std::sqrt((leader->xy-v.xy).squaredNorm());
    if(dist_leader <= 25)
      speed = leader->speed - 0.25*(25 - dist_leader);
  }

  //----------------------------------------------------------------------------
  // Compute a path to reach the desired trajectory
  //----------------------------------------------------------------------------
  auto mpc = MPC(speed, kNumPoints+1, kDT, kLF);
  auto result = mpc.Solve(initial_state, trajectory);

  for(int i = 1; i < result.states.size(); ++i) {
    auto &p  = result.states[i];
    auto  xy = Coordinates::ToGlobal(p.segment(0, 2), future_v.xy,
                                     future_v.yaw);
    new_path.push_back(xy);
  }

  return new_path;

}
