#ifndef PID_H
#define PID_H

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
	double p[3];
	
	/* 
	* Error values
	*/
	double prev_cte;
	double error;
	double steer;
	
	/* 
	* Twiddle values
	*/
	double best_error;
	double tol;
	int iter;
	int twiddle_update;
	int param;
	int stage;
	double dp[3];
	double sum_dp;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init();

  /*
  * Update the PID error variables given cross track error.
  */
  double SteerCommand(double cte);
	
  /*
  * Twiddle the PID values
  */
  void Twiddle();

  /*
  * Calculate the total PID error.
  */
  double TotalError(double cte);
};

#endif /* PID_H */
