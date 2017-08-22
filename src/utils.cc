//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   05.08.2017
//------------------------------------------------------------------------------

#include "utils.hh"

//------------------------------------------------------------------------------
// Convert a point to the vehicle coordinate system
//------------------------------------------------------------------------------
Eigen::VectorXd Coordinates::ToLocal(const Eigen::VectorXd &global,
                                     const Eigen::VectorXd &xy, double yaw) {
  Eigen::MatrixXd transform(2, 2);
  transform << std::cos(-yaw), -std::sin(-yaw),
               std::sin(-yaw), std::cos(-yaw);
  return transform*(global-xy);
}

//------------------------------------------------------------------------------
// Convert a point to the global coordinate system
//------------------------------------------------------------------------------
Eigen::VectorXd Coordinates::ToGlobal(const Eigen::VectorXd &local,
                                      const Eigen::VectorXd &xy, double yaw) {
  Eigen::MatrixXd transform(2, 2);
  transform << std::cos(yaw), -std::sin(yaw),
               std::sin(yaw), std::cos(yaw);
  return (transform*local)+xy;
}
