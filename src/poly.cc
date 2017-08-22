//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   26.05.2017
//------------------------------------------------------------------------------

#include "poly.hh"
#include <Eigen/QR>

//------------------------------------------------------------------------------
// Evaluate a polynomial at a point
//------------------------------------------------------------------------------
double PolyEval(const Eigen::VectorXd &coeffs, double x) {
  double result = 0.0;
  for(int i = 0; i < coeffs.size(); i++)
    result += coeffs[i] * pow(x, i);
  return result;
}

//------------------------------------------------------------------------------
// Fit a polynomial to a set of points.
//
// Say you have a set of points {2, 4}, {3, 5}, {7, 9}, {12, 53} and you want
// to fit a polynomial to those points. It can be a polynomial of at most 3rd
// degree. It really is a set of linear equations in the form:
//
// 1 * d + x * c + x*x * b + x*x*x * a = y
//
// Therefore for our points, we get:
//
// 1 * d +  2 * c +   2*2 * b +    2*2*2 * a = 4
// 1 * d +  3 * c +   3*3 * b +    3*3*3 * a = 5
// 1 * d +  7 * c +   7*7 * b +    7*7*7 * a = 9
// 1 * d + 12 * c + 12*12 * b + 12*12*12 * a = 53
//
// This can be written in the matrix form as:
//
// [1  2    2*2    2*2*2]   [d]   [ 4]
// |1  3    3*3    3*3*3|   |c|   | 5|
// |1  7    7*7    7*7*7| * |b| = | 9|
// [1  12 12*12 12*12*12]   [a]   [53]
//
// This function tries to find the best fitting values for a, b, c, and d using
// the Hauseholder QR decomposition and the least square fits. See the links
// below for details.
//
// https://eigen.tuxfamily.org/dox/classEigen_1_1HouseholderQR.html#a3905420f396c56ae6f0f65038c48ecd7
// https://eigen.tuxfamily.org/dox/group__LeastSquares.html
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
//------------------------------------------------------------------------------
Eigen::VectorXd PolyFit(const std::vector<Eigen::VectorXd> &pts,
                        int                                 order) {
  assert(pts.size() >= order+1);

  Eigen::MatrixXd A(pts.size(), order+1);
  Eigen::VectorXd yvals(pts.size());

  for(int i = 0; i < pts.size(); ++i)
    yvals(i) = pts[i][1];

  // cumulative product is more precise than the power function; we assign 1.0
  // to the first column and then multiply each column by the appropriate x
  // value
  for(int i = 0; i < pts.size(); i++)
    A(i, 0) = 1.0;

  for(int i = 0; i < pts.size(); i++)
    for (int j = 0; j < order; j++)
      A(i, j+1) = A(i, j) * pts[i][0];

  // solve for least squares fit
  return A.householderQr().solve(yvals);
}
