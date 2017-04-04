#include <iostream>
#include "tools.h"
#include <math.h>

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
    VectorXd RMSE(4);
    RMSE << 0,0,0,0;
    
    // check validity of the inputs;
    if(estimations.size() != ground_truth.size() || estimations.size() < 1){
      std::cout << "invalid ground truth, please check" << std::endl;
        return RMSE;
    }
    
    // accumulating squared residuals
    for(int i=0; i < estimations.size(); ++i){
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        RMSE += residual;
    }
    
    // mean and squared root
    RMSE = RMSE / estimations.size();
    RMSE = RMSE.array().sqrt();
    
    return RMSE;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3,4);
  Hj << 0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0;

    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //calculate Jacobian
    float x_y_2 = px*px + py*py;
    float x_y_0_5 = sqrt(x_y_2);
  float x_y_1_5 = x_y_2 * x_y_0_5;
  
  std::cout << "x_y_2" << x_y_2 << std::endl;
  
  
  // check validaity
    if(x_y_2 < 0.00000001){
      std::cout << "CalculateJacobian() Error: Divided by zero" << std::endl;
        return Hj;
    }
  
    Hj << px / x_y_0_5, py / x_y_0_5, 0, 0,
          -py / x_y_2, px / x_y_2, 0, 0,
    py * (vx*py - vy*px) / x_y_1_5, px * (vy*px - vx*py) / x_y_1_5,
    px / x_y_0_5, py/x_y_0_5;
  
  std::cout << Hj<<std::endl;
    return Hj;
}
