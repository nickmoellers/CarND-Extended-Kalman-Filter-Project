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
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hf(3,4)
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float px2pluspy2 = px*px+py*py

  if( px2pluspy2 < 0.0001 ) return Hj;

  Hj << px/sqrt(px2pluspy2), py/pow(px2pluspy2,0.5), 0, 0,
        -1*py/px2pluspy2,    px/px2pluspy2,          0, 0,
        py*(vx*py - vy*px)/pow(px2pluspy2,1.5), px*(vy*px - vx*py)/pow(px2pluspy2,1.5),
        px/pow(px2pluspy2, 0.5),                py/pow(px2pluspy2,0.5);

  return Hj;
}
