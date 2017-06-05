#include "kalman_filter.h"
#include <iostream>

using namespace std;

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

  int i = 0;

  MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());


 //cout << "x_ = " << x_ << endl;

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt( px*px+py*py );

  float theta = atan2( py, px );
  if( fabs(theta) > M_PI ) {
   //cout << "PREDICTED THETA = "<< theta << endl;
    theta = atan2(sin(theta), cos(theta));
   //cout << "NORMALIZED THETA = "<< theta << endl;
  }
  theta = atan2(sin(theta), cos(theta));
  float rhodot_denom = sqrt(px*px+py*py);
  float rhodot = (px*vx+py*vy)/sqrt(px*px+py*py);

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, rhodot;
  //Divide by zero? Zero out.
  if( rhodot_denom < 0.0001) {
   //cout << "DIVIDE BY ZERO" << endl;
    z_pred << 0.0, 0.0, 0.0;
  }

 //cout << "z = "<< z << endl;
 //cout << "z_pred = "<< z_pred << endl;
  VectorXd y = z-z_pred;
  theta = y[1];
  if( fabs(theta) > M_PI ) {
     //cout << "DELTA THETA = "<< theta << endl;
      theta = atan2(sin(theta), cos(theta));
     //cout << "NORMALIZED THETA = "<< theta << endl;
    }
  y[1] = theta;
 //cout << "y = "<< y << endl;
  MatrixXd Ht = H_.transpose();
  //cout << "H = "<< H_ << endl;
  //cout << "Ht = "<< Ht << endl;
  //cout << "P = "<< P_ << endl;
  //cout << "R = "<< R_ << endl;
  MatrixXd S = H_*P_*Ht+R_;
  MatrixXd Si = S.inverse();
  //cout << "S = "<< S << endl;
  //cout << "Si = "<< Si << endl;
  MatrixXd K = P_*Ht*Si;
  //cout << "K = " << K << endl;
  MatrixXd Ky = K*y;
 //cout << "Ky = " << Ky << endl;

  x_ = x_ + K*y;

 //cout << "x_ = " << x_ << endl;

  P_ = (I-K*H_)*P_;

}
