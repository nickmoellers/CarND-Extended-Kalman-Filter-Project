#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  int i = 1;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
  DONE:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  Hj_ << 1,1,0,0,
      1,1,0,0,
      1,1,1,1;

  float dt = 1;
  ekf_.F_ = MatrixXd(4,4);

  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0,  dt,
             0, 0, 1,  0,
             0, 0, 0,  1;

  ekf_.x_ = VectorXd(4);

  ekf_.x_ << 0, 0, 0, 0;

  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  //2. Set the process covariance matrix Q
    float noise_ax = 9.0;
    float noise_ay = 9.0;

    float dt2 = dt*dt;
    float dt3 = dt*dt*dt;
    float dt4 = dt*dt*dt*dt;

    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << dt4*noise_ax/4, 0, dt3*noise_ax/2, 0,
              0, dt4*noise_ay/4, 0, dt3*noise_ay/2,
              dt3*noise_ax/2, 0, dt2*noise_ax,   0,
              0, dt3*noise_ay/2, 0,   dt2*noise_ay;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_ ) { //|| previous_timestamp_ > measurement_pack.timestamp_) {
    /**
    DONE:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
   //cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    float px = 0;
    float py = 0;
    //Q&A says to play with vx and vy values
    float vx = 0;
    float vy = 0;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      //cout << "Initializing x_ from radar data." << endl;

      float ro = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];
      theta = atan2(sin(theta), cos(theta));
      float ro_dot = measurement_pack.raw_measurements_[2];

      px = ro * cos( theta );
      py = ro * sin( theta );
      //vx = ro_dot * cos( theta );
      //vy = ro_dot * sin( theta );

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //cout << "Initializing x_ from laser data." << endl;
      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];

    } else {
      std::cout << "WARNING: Failed to initalize ekf_.x_ from data" << std::endl;
    }

    if( fabs( px ) < 0.0001 ) px = 0.0001;
    if( fabs( py ) < 0.0001 ) py = 0.0001;

    ekf_.x_ << px, py, vx, vy;

    // print the output

    //cout << "x_ = " << ekf_.x_ << endl;

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   DONE:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0; //add one more zero?
  previous_timestamp_ = measurement_pack.timestamp_;

  //1. Modify the F matrix so that the time is integrated
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0,  dt,
             0, 0, 1,  0,
             0, 0, 0,  1;

  // print the output
  //cout << "F_ = " << ekf_.F_ << endl;

  //2. Set the process covariance matrix Q
  float noise_ax = 9.0;
  float noise_ay = 9.0;

  float dt2 = dt*dt;
  float dt3 = dt*dt*dt;
  float dt4 = dt*dt*dt*dt;

  ekf_.Q_ = MatrixXd(4, 4);

  ekf_.Q_ << dt4*noise_ax/4, 0, dt3*noise_ax/2, 0,
            0, dt4*noise_ay/4, 0, dt3*noise_ay/2,
            dt3*noise_ax/2, 0, dt2*noise_ax,   0,
            0, dt3*noise_ay/2, 0,   dt2*noise_ay;

  // print the output
  //cout << "Q_ = " << ekf_.Q_ << endl;

  //cout << "Predicting State... " << endl;

  ekf_.Predict();

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   DONE:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
     *   Note: Isn't this done in Update() and UpdateEKF()?
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
   //cout << "Radar updates: " << endl;
    //cout << "Calculating Jacobian... " << endl;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    //cout << "Hj_ = " << Hj_ << endl;
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    //cout << "Updating State... " << endl;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
   //cout << "Laser updates: " << endl;
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    //cout << "Updating State (EKF)... " << endl;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //cout << "at time = " << measurement_pack.timestamp_ << endl;
  //cout << "measurements = " << measurement_pack.raw_measurements_ << endl;
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
