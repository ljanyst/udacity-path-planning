//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   05.08.2017
//------------------------------------------------------------------------------

#include "map.hh"
#include "utils.hh"
#include "poly.hh"
#include "spline.hh"

#include <fstream>
#include <stdexcept>
#include <limits>

//------------------------------------------------------------------------------
// Load map data
//------------------------------------------------------------------------------
void Map::LoadFromFile(const std::string &filename, bool interpolate) {
  std::ifstream file(filename.c_str());
  if(!file.is_open())
    throw std::runtime_error("Unable to open: " + filename);

  std::vector<WayPoint> wps;
  while(1) {
    Eigen::VectorXd xy(2), d(2);
    double s;
    file >> xy[0] >> xy[1] >> s >> d[0] >> d[1];
    if(!file.good())
      break;
    wps.emplace_back(xy, s, d);
  }

  for(int i = 0; i < 3; ++i) {
    if(interpolate)
      InterpolateLaneWaypoints(i, wps);
    else
      AddLaneWaypoints(i, wps);
  }

}

//------------------------------------------------------------------------------
// Interpolate lane waypoints
//------------------------------------------------------------------------------
void Map::AddLaneWaypoints(int lane, const std::vector<WayPoint> &waypoints) {
  double                       lane_offset = (0.5+lane)*lane_width_;
  std::vector<Eigen::VectorXd> lane_wps;

  // simulator hack
  if(lane >= 2)
    lane_offset -= 0.4;

  for(auto &wp: waypoints) {
    Eigen::VectorXd pt = wp.xy + lane_offset*wp.d;
    lane_wps.push_back(pt);
  }

  waypoints_.push_back(lane_wps);
}

//------------------------------------------------------------------------------
// Interpolate lane waypoints
//------------------------------------------------------------------------------
void Map::InterpolateLaneWaypoints(int lane,
                                   const std::vector<WayPoint> &waypoints) {

  //----------------------------------------------------------------------------
  // Compute the waypoints for the lane
  //----------------------------------------------------------------------------
  double                       lane_offset = (0.5+lane)*lane_width_;
  std::vector<Eigen::VectorXd> lane_wps;
  const int                    wps_size = waypoints.size();

  // simulator hack
  if(lane >= 2)
    lane_offset -= 0.4;

  //----------------------------------------------------------------------------
  // Treat every third waypoint as a center of the frame, get five previous and
  // five next points, fit a spline and get points from [i-3; u+3) every meter.
  //----------------------------------------------------------------------------
  for(int i = 3; i < wps_size+3; i+=6) {
    //--------------------------------------------------------------------------
    // Compute the position and the yaw of the frame center
    //--------------------------------------------------------------------------
    Eigen::VectorXd tangent(2);
    int k = (i+wps_size)%wps_size;
    tangent << waypoints[k].d[1], -waypoints[k].d[0];
    auto yaw = std::fmod(std::atan2(tangent[1], tangent[0])+M_PI, 2*M_PI);
    auto xy  = waypoints[k].xy + lane_offset*waypoints[k].d;

    //--------------------------------------------------------------------------
    // Get the referrence points converting them to the local frame
    //--------------------------------------------------------------------------
    std::vector<double> xs;
    std::vector<double> ys;
    for(int j = i - 5; j <= i+5; ++j) {
      int k = (j+wps_size)%wps_size;
      Eigen::VectorXd pt = waypoints[k].xy + lane_offset*waypoints[k].d;
      pt = Coordinates::ToLocal(pt, xy, yaw);
      xs.push_back(pt[0]);
      ys.push_back(pt[1]);
    }

    //--------------------------------------------------------------------------
    // Compute and evaluate the spline
    //--------------------------------------------------------------------------
    tk::spline s;
    s.set_points(xs, ys);

    for(int j = i-2, k = 3; j <= i+3 && j <= wps_size; ++j, ++k) {
      for(double x = xs[k-1]; x < xs[k]; x += 1) {
        Eigen::VectorXd pt(2);
        pt << x, s(x);
        lane_wps.push_back(Coordinates::ToGlobal(pt, xy, yaw));
      }
    }
  }

  //----------------------------------------------------------------------------
  // Remember the new waypoints
  //----------------------------------------------------------------------------
  waypoints_.push_back(lane_wps);
}

//------------------------------------------------------------------------------
// Compute waypoints
//------------------------------------------------------------------------------
Eigen::VectorXd Map::ComputeWaypointPoly(int lane, const Vehicle &v) const {
  //----------------------------------------------------------------------------
  // Find the index of the closest waypoint
  //----------------------------------------------------------------------------
  auto   &wps_lane = waypoints_[lane];
  double  min      = std::numeric_limits<double>::max();
  int     min_idx  = -1;
  for(int i = 0; i < wps_lane.size(); ++i) {
    auto &pt = wps_lane[i];
    double dist = (pt-v.xy).squaredNorm();
    if(dist < min) {
      min     = dist;
      min_idx = i;
    }
  }

  //----------------------------------------------------------------------------
  // Add some waypoints from the back and from the front
  //----------------------------------------------------------------------------
  std::vector<Eigen::VectorXd> pts;
  for(int i = min_idx-10; i <= min_idx+20; ++i) {
    int k = (i+wps_lane.size())%wps_lane.size();
    pts.push_back(Coordinates::ToLocal(wps_lane[k], v.xy, v.yaw));
  }

  return PolyFit(pts, 3);
}

//------------------------------------------------------------------------------
// Find the lane the car is in
//------------------------------------------------------------------------------
int Map::Lane(const Vehicle &v) const {
  for(int i = 0; i < NumLanes(); ++i)
    if(v.sd[1] >= i * lane_width_ && v.sd[1] < (i+1)*lane_width_)
      return i;
  return -1;
}
