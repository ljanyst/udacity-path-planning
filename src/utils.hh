//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   05.08.2017
//------------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>
#include <cmath>
#include <vector>
#include <chrono>

//------------------------------------------------------------------------------
//! A sensor fusion object
//------------------------------------------------------------------------------
struct SFObject {
    SFObject(int idp, double x, double y, double s, double d, double vx,
             double vy): id(idp), xy(2), sd(2), v(2) {
      xy << x, y; sd << s, d; v << vx, vy;
      speed = std::sqrt(v.squaredNorm());
      Eigen::VectorXd xy1 = xy + v;
      yaw = std::atan2(xy1[1]-xy[1], xy1[0]-xy[0])+2*M_PI;
      yaw = std::fmod(yaw, 2*M_PI);
    }
    int id;
    Eigen::VectorXd xy;
    Eigen::VectorXd sd;
    Eigen::VectorXd v;
    double          speed;
    double          yaw;
};

//------------------------------------------------------------------------------
//! Print an SFObject to ostream
//------------------------------------------------------------------------------
inline std::ostream &operator<<(std::ostream &stream, const SFObject &obj) {
  stream << "SFObject[id=" << obj.id << ", xy=(" << obj.xy[0] << ",";
  stream << obj.xy[1] << "), sd=(" << obj.sd[0] << "," << obj.sd[1];
  stream << "), v=(" << obj.v[0] << "," << obj.v[1] << ")]";
  return stream;
}

//------------------------------------------------------------------------------
//! State of the vehicle
//------------------------------------------------------------------------------
struct Vehicle {
  Vehicle(double x, double y, double s, double d, double yawp, double vp):
    xy(2), sd(2), yaw(yawp), v(vp) { xy << x, y; sd << s, d; }

  Vehicle(const Eigen::VectorXd &xyp, const Eigen::VectorXd &sdp,
          double yawp, double vp):
    xy(xyp), sd(sdp), yaw(yawp), v(vp) {}

  Vehicle(const Eigen::VectorXd &xyp, double yawp, double vp):
    xy(xyp), yaw(yawp), v(vp) {}

  Vehicle(const SFObject &obj):
    xy(obj.xy), sd(obj.sd), yaw(obj.yaw), v(obj.speed) {}

  Eigen::VectorXd xy;
  Eigen::VectorXd sd;
  double          yaw;
  double          v;
};

//------------------------------------------------------------------------------
//! Print a vehicle to ostream
//------------------------------------------------------------------------------
inline std::ostream &operator<<(std::ostream &stream, const Vehicle &v) {
  stream << "Vehicle[xy=(" << v.xy[0] << "," << v.xy[1] << "), sd=(";
  stream << v.sd[0] << "," << v.sd[1] << "), v=" << v.v << ", yaw=";
  stream << v.yaw << "]";
  return stream;
}

//------------------------------------------------------------------------------
//! Convert degrees to radians
//------------------------------------------------------------------------------
inline double Deg2Rad(double x) { return x * M_PI / 180; }

//------------------------------------------------------------------------------
//! Convert MPH to m/s
//------------------------------------------------------------------------------
inline double MPH2ms(double x) { return x * 0.44704; }

//------------------------------------------------------------------------------
// Convert waypoints between coordinate systems
//------------------------------------------------------------------------------
namespace Coordinates {
  Eigen::VectorXd ToLocal(const Eigen::VectorXd &global,
                          const Eigen::VectorXd &xy, double yaw);
  Eigen::VectorXd ToGlobal(const Eigen::VectorXd &local,
                           const Eigen::VectorXd &xy, double yaw);
}
