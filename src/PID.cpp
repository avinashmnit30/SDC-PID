/*
*  Author: Avinash Sharma
*/

#include <iostream>
#include "PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}


/*
* PID Controller Initialization.
* @param kp : Proportional Gain
* @param ki : Integral Gain
* @param kd : Differential Gain
*/
void PID::Init(double kp, double ki, double kd) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
  p_error = 0;
  i_error = 0;
  d_error = 0;
}


/*
* PID error vaiables update
* @param cte : Current Cross Track Error
*/
void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}


/*
* Compute steering angle
*/
double PID::TotalError_steer() {
  return -Kp*p_error - Ki*i_error - Kd*d_error;
}


/*
* Compute throttle
* @param throttle_lim : Maximum Throttle Limit
*/
double PID::TotalError_throttle(double throttle_lim){
  return throttle_lim -Kp*p_error - Ki*i_error - Kd*d_error;
}