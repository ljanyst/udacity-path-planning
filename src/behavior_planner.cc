//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   10.08.2017
//------------------------------------------------------------------------------

#include "behavior_planner.hh"

#include <stdexcept>
#include <limits>
#include <cmath>
#include <iostream>
#include <iomanip>

//------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------
BehaviorPlanner::BehaviorPlanner(const Map     &map,
                                 PathPredictor &predictor,
                                 double         speed):
    predictor_(predictor), map_(map), speed_(speed) {
  sm_[Plan::kStartUp]    [Plan::kStartUp]     = [this]() { return SU2SU(); };
  sm_[Plan::kStartUp]    [Plan::kKeepLane]    = [this]() { return SU2KL(); };
  sm_[Plan::kKeepLane]   [Plan::kKeepLane]    = [this]() { return KL2KL(); };
  sm_[Plan::kKeepLane]   [Plan::kChangeLeft]  = [this]() { return KL2CL(-1); };
  sm_[Plan::kKeepLane]   [Plan::kChangeRight] = [this]() { return KL2CL(1); };
  sm_[Plan::kChangeLeft] [Plan::kKeepLane]    = [this]() { return CL2KL(); };
  sm_[Plan::kChangeRight][Plan::kKeepLane]    = [this]() { return CL2KL(); };
}

//------------------------------------------------------------------------------
// Plan the behavior
//------------------------------------------------------------------------------
Plan BehaviorPlanner::ComputeBehavior(const Vehicle &v) {
  //----------------------------------------------------------------------------
  // Compute costs of changing the plan
  //----------------------------------------------------------------------------
  v_                = &v;
  auto &transitions = sm_[current_plan_];
  double min_cost   = std::numeric_limits<double>::max();
  Plan   plan       = current_plan_;

  std::cout << "======================================" << std::endl;
  std::cout << "Current plan:      " << PlanToString(current_plan_) << std::endl;
  for(auto &tr: transitions) {
    double cost = tr.second();
    std::cout << std::left << std::setw(19);
    std::cout << PlanToString(tr.first) + " costs:" << cost << std::endl;
    if(cost < min_cost) {
      plan     = tr.first;
      min_cost = cost;
    }
  }

  current_plan_ = plan;
  return current_plan_;
}

//------------------------------------------------------------------------------
// Convert plan to string
//------------------------------------------------------------------------------
std::string BehaviorPlanner::PlanToString(Plan plan) {
  switch(plan) {
    case Plan::kStartUp:     return "StartUp";
    case Plan::kKeepLane:    return "KeepLane";
    case Plan::kChangeLeft:  return "ChangeLeft";
    case Plan::kChangeRight: return "ChangeRight";
  }
  throw std::runtime_error("Impossible happened - Unknown Plan!");
  return "";
}
//------------------------------------------------------------------------------
// Start Up to Start Up
//------------------------------------------------------------------------------
double BehaviorPlanner::SU2SU() {
  ++start_up1_;
  if(start_up1_ < 4)
    return 0;
  return 100;
}

//------------------------------------------------------------------------------
// Start Up to Keep Lane
//------------------------------------------------------------------------------
double BehaviorPlanner::SU2KL() {
  ++start_up2_;
  if(start_up2_ < 4)
    return 100;
  return 0;
}

//------------------------------------------------------------------------------
// Keep Lane to Keep Lane
//------------------------------------------------------------------------------
double BehaviorPlanner::KL2KL() {
  auto   leader          = predictor_.FindLeader(*v_);
  double cost            = 0;
  double leader_distance = 250;

  if(leader) {
    if(leader->speed < speed_)
      cost += 100*(speed_ - leader->speed);

    leader_distance = leader->sd[0] - v_->sd[0];
  }
  if(leader_distance < 15)
    cost += std::exp(15-leader_distance);
  else
    cost -= 3*leader_distance;
  return cost;
}

//------------------------------------------------------------------------------
// Keep Lane to Change Lane
//------------------------------------------------------------------------------
double BehaviorPlanner::KL2CL(int direction) {
  //----------------------------------------------------------------------------
  // Sanity checks and stuff
  //----------------------------------------------------------------------------
  int target_lane = map_.Lane(*v_) + direction;
  if(target_lane < 0 || target_lane >= map_.NumLanes())
    return std::numeric_limits<double>::max();

  auto leader   = predictor_.FindLeader(*v_, target_lane);
  auto follower = predictor_.FindFollower(*v_, target_lane);

  double cost = 0;

  //----------------------------------------------------------------------------
  // Speed and distance from the leader
  //----------------------------------------------------------------------------
  double leader_distance = 250;
  if(leader) {
    if(leader->speed < speed_)
      cost += 100*(speed_ - leader->speed);

    leader_distance = leader->sd[0] - v_->sd[0];
  }
  if(leader_distance < 20)
    cost += std::exp(20-leader_distance);
  else
    cost -= 3*leader_distance;

  //----------------------------------------------------------------------------
  // Distance from the follower
  //----------------------------------------------------------------------------
  if(follower) {
    double follower_distance = v_->sd[0] - follower->sd[0];
    if(follower_distance < 20) {
      cost += std::exp(20-follower_distance);
      cost += std::exp(5*(follower->speed - v_->v));
    }
  }

  return cost;
}

//------------------------------------------------------------------------------
// Change Lane to Keep Lane
//------------------------------------------------------------------------------
double BehaviorPlanner::CL2KL() {
  return 0;
}
