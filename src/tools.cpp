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

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if( estimations.size() == 0 ) return rmse;
  //  * the estimation vector size should equal ground truth vector size
  if( estimations.size() != ground_truth.size() ) return rmse;

  //float sum_squared_residual=0
  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    float x_e=estimations[i];
    float x_gt=ground_truth[i];
    float squared_residual = pow(x_e-x_gt,2);
    rmse+=squared_residual;
  }

  //calculate the mean
  rmse = rmse/estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hf(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float px2pluspy2 = px*px+py*py;

  if( px2pluspy2 < 0.0001 ) return Hj;

  Hj << px/sqrt(px2pluspy2), py/pow(px2pluspy2,0.5), 0, 0,
        -1*py/px2pluspy2,    px/px2pluspy2,          0, 0,
        py*(vx*py - vy*px)/pow(px2pluspy2,1.5), px*(vy*px - vx*py)/pow(px2pluspy2,1.5),
        px/pow(px2pluspy2, 0.5),                py/pow(px2pluspy2,0.5);

  return Hj;
}
