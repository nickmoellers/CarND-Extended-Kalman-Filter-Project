#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

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

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */


  x_ = F_*x_; //Should I add random normal noise?
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft+Q_;



}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */



  MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());

  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht+R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_*Ht*Si;

  x_ = x_ + K*y;
  P_ = (I-K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */



  MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt( px*px+py*py );
  float phi = atan2( py, px );
  float rhodot = (px*vx+py*vy)/sqrt(px*px+py*py);

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, phi, rhodot;

  VectorXd y = z-z_pred;

  VectorXd Ht = H_.transpose();
  VectorXd S = H_*P_*Ht+R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_*Ht*Si;

  x_ = x_ + K*y;
  P_ = (I-K*H_)*P_;
}
