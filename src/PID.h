/*
*  Author: Avinash Sharma
*/


#ifndef PID_H
#define PID_H

#include <uWS/uWS.h>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor  
  */
  PID();

  /**
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double kp, double ki, double kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total steering PID error.
  */
  double TotalError_steer();

  /*
  * Calculate the total throttle PID error.
  */
  double TotalError_throttle(double throttle_lim);
};

#endif /* PID_H */
