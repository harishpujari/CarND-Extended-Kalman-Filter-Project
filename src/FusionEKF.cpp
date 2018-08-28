#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define MiSEC2SEC(x)         (x) / 1000000.0 //microsec to seconds

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_     = false; //will be inited on first measurement
  previous_timestamp_ = 0; //will be inited on first measurement

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  //measurement covariance matrix - laser
  R_laser_ <<
    0.0225, 0,
    0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ <<
    0.09, 0,      0,
    0,    0.0009, 0,
    0,    0,   0.09;

  /**
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  H_laser_ <<
    1, 0, 0, 0,
    0, 1, 0, 0;

  //acceleration noise
  noise_ax_ = 9;
  noise_ay_ = 9;

  //H_jacobian will be inited on each measurement separately
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    //Initialize the kalman filter position vector with first sensor measurement

    double px;
    double py;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      //for radar : convert form polar to cartesian coordinates
      tools.Polar2Cartesian(measurement_pack, px, py);
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      //no conversion required for lidar measurements
      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];
    }

    //Set initial state x to the first measurement's position and zero velocity
    VectorXd firstX = VectorXd(4);
    firstX << px, py, 0, 0;
    ekf_.setx_(firstX);

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
   */

  //compute elapsed time between last measurement and this
  float dt = MiSEC2SEC(measurement_pack.timestamp_ - previous_timestamp_);
  previous_timestamp_ = measurement_pack.timestamp_;

  //update F and Q
  ekf_.setF_(dt);
  ekf_.setQ_(dt, noise_ax_, noise_ay_);

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    //Radar updates. Use Jacobian
    auto H_jacobian = tools.CalculateJacobian(ekf_.getx_());
    ekf_.UpdateEKF(measurement_pack.raw_measurements_, R_radar_, H_jacobian);
  } else {
    //Laser updates
    ekf_.Update(measurement_pack.raw_measurements_, R_laser_, H_laser_);
  }

  // print the output
  cout << "x_ = " << ekf_.getx_() << endl;
  cout << "P_ = " << ekf_.getP_() << endl;
}
