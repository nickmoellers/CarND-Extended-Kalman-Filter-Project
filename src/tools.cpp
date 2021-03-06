#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  //cout << "Calculating RMSE." << endl;

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if( estimations.size() == 0 ) return rmse;
  //  * the estimation vector size should equal ground truth vector size
  if( estimations.size() != ground_truth.size() ) return rmse;

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    VectorXd x_e=estimations[i];
    VectorXd x_gt=ground_truth[i];
    VectorXd residual = x_e-x_gt;
    VectorXd squared_residual = residual.array()*residual.array();
    rmse+=squared_residual;
  }

  //cout << "RTE = " << rmse << endl;

  //calculate the mean
  rmse = rmse/estimations.size();

  // << "RME = " << rmse << endl;

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //cout << "RMSE = " << rmse << endl;

  //cout << "Calculated RMSE." << endl;


  //return the result
  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  //cout << "Calculating Jacobian." << endl;


  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float px2pluspy2 = px*px+py*py;
  float sqrtpx2pluspy2 = sqrt(px2pluspy2);
  float px2pluspy2onehalf = px2pluspy2 * sqrtpx2pluspy2;

  if( px2pluspy2 < 0.0001 ) return Hj;

  Hj << px/sqrtpx2pluspy2, py/sqrtpx2pluspy2, 0, 0,
        -1*py/px2pluspy2,  px/px2pluspy2,     0, 0,
        py*(vx*py - vy*px)/px2pluspy2onehalf, px*(vy*px - vx*py)/px2pluspy2onehalf,
        px/sqrtpx2pluspy2, py/sqrtpx2pluspy2;

  //cout << "Calculated Jacobian." << endl;

  return Hj;
}
