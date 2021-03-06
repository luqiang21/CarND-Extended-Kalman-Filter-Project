#include "kalman_filter.h"
#include "tools.h"
#include <iostream>
#include <math.h>

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
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;
    MatrixXd S = H_ * PHt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = PHt * Si;
    
    //new estimate
    x_ = x_ + K * y;
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // note here, z is size of 3.
  
  
  VectorXd z_pred = VectorXd(3); // h(x')
  
  float x_y_2 = x_[0]*x_[0] + x_[1]*x_[1];
  // avoid the situation that atan2(0, 0) (error)
  if(x_y_2 < 0.00000001){
    z_pred << 0.0, 0.0, 0.0;
  }else{
    z_pred(0) = sqrt(x_y_2);
    z_pred(1) = atan2(x_[1], x_[0]); // what if x_[1] == 0 && x_[0] == 0?

  }
  
  // make phi between -pi and pi.
  float pi = M_PI;
  while(z_pred(1) > pi && z_pred(1) < -pi){
    
    if(z_pred(1) > pi){
      z_pred(1) = z_pred(1) - 2*pi;
    }else if(z_pred(2) < -pi){
      z_pred(1) = z_pred(1) + 2*pi;
    }

    
  }
  
  if(z_pred(0) < 0.00001){
    z_pred(2) = 0.0;
  }else{
    z_pred(2) = (x_[2]*x_[0] + x_[3]*x_[1]) / sqrt(x_y_2);
  }
  
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;
  

  //new estimate
  x_ = x_ + K * y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  

}
