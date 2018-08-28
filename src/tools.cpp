#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

#define APPROX_ZERO     0.0001

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  //Calculate the RMSE here.
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  //check inputs
  if (estimations.size() != ground_truth.size() ||
        estimations.size() == 0) {
    cout << "invalid estimate or ground truth value" << endl;
    return rmse;
  }

  //Calculate rmse
  for (unsigned i = 0; i < estimations.size(); i++) {
    VectorXd residual = estimations[i] - ground_truth[i];
    //coefficient wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  //calculate mean
  rmse /= estimations.size();
  //calculate square root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  //Calculate a Jacobian here.

  MatrixXd Hj(3, 4);

  //get the state parameters
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  //calculate frequently used values
  double px2 = px * px;
  double py2 = py * py;

  //set px2/py2 to small value, if they are 0
  if (fabs(px2) < APPROX_ZERO) {
    px2 = APPROX_ZERO;
  }
  if (fabs(py2) < APPROX_ZERO) {
    py2 = APPROX_ZERO;
  }

  double ss    = px2 + py2;
  double ssqrt = sqrt(ss);
  double sprod = ss * ssqrt;

  //Create the jacobian

  //row 0
  Hj(0, 0) = px / ssqrt;
  Hj(0, 1) = py / ssqrt;
  Hj(0, 2) = 0;
  Hj(0, 3) = 0;

  //row 1
  Hj(1, 0) = - py / ss;
  Hj(1, 1) =   px / ss;
  Hj(1, 2) = 0;
  Hj(1, 3) = 0;

  //row 2
  Hj(2, 0) = (py * (vx * py - vy * px)) / sprod;
  Hj(2, 1) = (px * (vy * px - py * vx)) / sprod;
  Hj(2, 2) = px / ssqrt;
  Hj(2, 3) = py / ssqrt;

  return Hj;
}

//Convert from polar to cartesian coordinates
void Tools::Polar2Cartesian(const MeasurementPackage& measurement_pack, double& px, double& py) {
  double rho = measurement_pack.raw_measurements_[0];
  double phi = measurement_pack.raw_measurements_[1];

  px = rho * cos(phi);
  py = rho * sin(phi);
}

//Convert from cartesian to polar coordinates
VectorXd Tools::Cartesian2Polar(const VectorXd x) {
  VectorXd  z_pred(3);

  //Unpack the state vector
  double px = x(0);
  double py = x(1);
  double vx = x(2);
  double vy = x(3);

  if (fabs(px) < APPROX_ZERO) {
    px = APPROX_ZERO;
  }

  //calculate frequently used values
  double px2 = px * px;
  double py2 = py * py;
  double rho = sqrt(px2 + py2);

  if (fabs(rho) < APPROX_ZERO) {
    rho = APPROX_ZERO;
  }

  z_pred[0] = rho;
  z_pred[1] = atan2(py, px);
  z_pred[2] = (px * vx + py * vy) / rho;

  return z_pred;
}
