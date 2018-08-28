#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define LARGE_VARIANCE    999.0

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
    x_ = VectorXd(4);

    F_ = MatrixXd(4, 4);
    F_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;

    Q_ = MatrixXd(4, 4);

    P_ = MatrixXd(4, 4);
    P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, LARGE_VARIANCE, 0,
          0, 0, 0, LARGE_VARIANCE;
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

//Setters

void KalmanFilter::setF_(float dt) {
  F_(0, 2) = dt;
  F_(1, 3) = dt;
}

void KalmanFilter::setQ_(float dt, float noise_ax, float noise_ay) {
  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt3 * dt;

  Q_ << noise_ax * dt4 / 4, 0, noise_ax * dt3 / 2, 0,
        0, noise_ay * dt4 / 4, 0, noise_ay * dt3 / 2,
        noise_ax * dt3 / 2, 0, noise_ax * dt2,     0,
        0, noise_ay * dt3 / 2, 0, noise_ay * dt2;
}

void KalmanFilter::Predict() {
  /**
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z, const Eigen::MatrixXd &R,
    const Eigen::MatrixXd &H) {
  /**
    * update the state by using Kalman Filter equations(lidar)
  */
  R_ = R;
  H_ = H;

  VectorXd z_pred = H * x_;
  Estimate(z, z_pred, false);
}

void KalmanFilter::UpdateEKF(const VectorXd &z, const Eigen::MatrixXd &R,
    const Eigen::MatrixXd &Hj) {
  /**
    * update the state by using Extended Kalman Filter equations (radar)
  */
  R_ = R;
  H_ = Hj;

  //Convert current state from cartesian to polar
  VectorXd z_pred = tools_.Cartesian2Polar(x_);

  Estimate(z, z_pred, true);
}

void KalmanFilter::Estimate(const VectorXd& z, const VectorXd& z_pred, bool radar) {

  VectorXd  y   = z - z_pred;
  if (radar) {
    y[1] = remainder(y[1], M_PI);
  }
  MatrixXd  Ht  = H_.transpose();
  MatrixXd  S   = H_ * P_ * Ht + R_;
  MatrixXd  Si  = S.inverse();
  MatrixXd  PHt = P_ * Ht;
  MatrixXd  K   = PHt * Si;

  //New estimates for x and P
  x_ = x_ + (K * y);
  MatrixXd  I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}
