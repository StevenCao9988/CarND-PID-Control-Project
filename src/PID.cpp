#include "PID.h"


/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */

	PID::Kp = Kp_;
	PID::Ki = Ki_;
	PID::Kd = Kd_;

	p_error = 0;
	i_error = 0;
	d_error = 0;

	p = {Kp_, Ki_, Kd_};
	dp = { 0.002 * PID::Kp, 0.0001 * PID::Ki, 0.002 * PID::Kd };
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
	PID::i_error = PID::i_error + cte;
	PID::d_error = cte - PID::p_error;
	PID::p_error = cte;

	int cycle = 2000;  // 多长为一个周期 5470
	int number = 200;  // 调节多少次
	static double best_err = 65535;
	static double err = 0;
	static int cycle_cnt = 0;
	static int number_cnt = 0;
	static int i = 2;
	static int dir_flg = 0;   // 调节参数的方向 
	double sum = 0;
	static int first_j = 0;

	if (twiddle_flg == 1)
	{
		int len_p = PID::p.size();
		cycle_cnt = cycle_cnt + 1;
		// 一下记录一整个周期产生的 平方差
		sum = PID::dp[0] + PID::dp[1] + PID::dp[2];
		if (sum > 0.00001)
		{
			if (cycle_cnt > cycle)
			{
				// debug
				std::cout << "PID parameter: " << std::endl;
				std::cout << " kP: " << PID::Kp;
				std::cout << " kI: " << PID::Ki;
				std::cout << " kD: " << PID::Kd;
				std::cout << " dir_flg: " << dir_flg;
				std::cout << " i: " << i;
				std::cout << " best_err: " << best_err;
				std::cout << " err: " << err << std::endl;


				cycle_cnt = 0;
				number_cnt += 1;
				first_j += 1;
				if (first_j >= 2)
				{
					first_j = 2;
					if (dir_flg == 0)
					{
						best_err = err;
						dir_flg = 1;
					}
					else if (dir_flg == 1)
					{
						if (err < best_err)
						{
							best_err = err;
							PID::dp[i] = PID::dp[i] * 1.1;
							dir_flg = 1;
						}
						else
						{
							PID::p[i] -= 2.0 * PID::dp[i];
							dir_flg = 2;
						}
					}
					else if (dir_flg == 2)
					{
						if (err < best_err)
						{
							best_err = err;
							PID::dp[i] *= 1.1;
						}
						else
						{
							PID::p[i] += PID::dp[i];
							PID::dp[i] *= 0.9;
						}
						dir_flg = 1;
					}

					if (dir_flg == 1)   // 本次结束 调试下一个参数
					{
						i = i + 1;
						if (PID::p[i] == 0)
						{
							i = i + 1;
						}
						if (i >= 3)   // 依次改变 PID三个参数	
						{
							i = 0;
						}
						PID::p[i] += PID::dp[i];
					}
					PID::Kp = PID::p[0];
					PID::Ki = PID::p[1];
					PID::Kd = PID::p[2];
					PID::i_error = 0;
					err = 0;   // 准备下一次
				}

			}
			else
			{
				err = err + pow(cte, 2);     // 平方和
			}
		}
	}
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
	double result = 0;
	double I = 0;
	result = -PID::Kp * PID::p_error
		- PID::Kd * PID::d_error;

	I = -PID::Ki * PID::i_error;
	if (I > 0.1)
	{
		I = 0.1;
	}
	else if (I < -0.1)
	{
		I = -0.1;
	}
	result = result + I;
  return result;  // TODO: Add your total error calc here!
}