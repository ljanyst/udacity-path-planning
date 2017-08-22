//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   05.08.2017
//------------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>
#include <vector>

//------------------------------------------------------------------------------
//! Evaluate a polynomial at a point
//------------------------------------------------------------------------------
double PolyEval(const Eigen::VectorXd &coeffs, double x);

//------------------------------------------------------------------------------
// Fit a polynomial to a set of points.
//------------------------------------------------------------------------------
Eigen::VectorXd PolyFit(const std::vector<Eigen::VectorXd> &pts,
                        int                                 order);
