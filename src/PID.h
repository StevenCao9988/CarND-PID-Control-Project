#ifndef PID_H
#define PID_H
#include <vector>
#include <numeric>
#include <math.h>
#include <iostream>
#include <string>
using std::string;

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

 // private:  调试阶段
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  // 参数自动调节需要的变量
  int twiddle_flg;     // 是否调节
  std::vector<double> dp;    // 每次变动的步进值
  std::vector<double> p;    // 每次变动的步进值

};

#endif  // PID_H