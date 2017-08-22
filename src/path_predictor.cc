//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   11.08.2017
//------------------------------------------------------------------------------

#include "path_predictor.hh"

#include <Eigen/Core>
#include <cmath>
#include <limits>

//------------------------------------------------------------------------------
// Add sensor fussion objects
//------------------------------------------------------------------------------
void PathPredictor::AddObjects(const std::vector<SFObject> &objects,
                               const Vehicle               &v) {
  objects_.clear();
  for(auto &obj: objects) {
    Eigen::VectorXd diff = v.xy-obj.xy;
    if(std::sqrt(diff.squaredNorm()) < distance_)
      objects_.push_back(obj);
  }
}

//------------------------------------------------------------------------------
// Find the leader wrt our vehicle
//------------------------------------------------------------------------------
std::shared_ptr<SFObject> PathPredictor::FindLeader(
    const Vehicle &v, int lane) const {

  int    index = -1;
  double min_s = std::numeric_limits<int>::max();

  if(lane == -1)
    lane  = map_.Lane(v);

  for(int i = 0; i < objects_.size(); ++i) {
    auto &o = objects_[i];
    if((map_.Lane(o) != lane) || (v.sd[0] > o.sd[0]))
      continue;
    if(min_s > o.sd[0]) {
      index = i;
      min_s = o.sd[0];
    }
  }

  if(index == -1)
    return nullptr;

  return std::make_shared<SFObject>(objects_[index]);
}

//------------------------------------------------------------------------------
// Find the follower wrt our vehicle
//------------------------------------------------------------------------------
std::shared_ptr<SFObject> PathPredictor::FindFollower(
    const Vehicle &v, int lane) const {

  int    index = -1;
  double max_s = std::numeric_limits<int>::min();

  if(lane == -1)
    lane = map_.Lane(v);

  for(int i = 0; i < objects_.size(); ++i) {
    auto &o = objects_[i];
    if((map_.Lane(o) != lane) || (v.sd[0] <= o.sd[0]))
      continue;
    if(max_s < o.sd[0]) {
      index = i;
      max_s = o.sd[0];
    }
  }

  if(index == -1)
    return nullptr;

  return std::make_shared<SFObject>(objects_[index]);
}
