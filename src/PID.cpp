#include <vector>
#include <algorithm>

#include "PID.h"

using namespace std;

/*
* PID CONTROLLER FOR SIMULATOR USING REAL-TIME TWIDDLE UPDATES
*/

PID::PID() {}

PID::~PID() {}

void PID::Init() {
  // Initial controller values
  Kp = 0.1;
  Kd = 10.0;
  Ki = 0.00;
	p[0] = Kp;
	p[1] = Kd;	
  p[2] = Ki;
	sum_dp = 0;
 
  // Error values
  prev_cte = 0;
  error = 0;
  d_error = 0;
  i_error = 0;
  
  // Twiddle values
  tol = 0.0001;
	twiddle_update = 1000;
  best_error = 0;
  iter = 0;
	param = 0;
	stage = 1;
	dp[0] = 0.1;
	dp[1] = 1.0;
	dp[2] = 0.00001;
}

double PID::SteerCommand(double cte) {	
  // Error Updates
  iter += 1;
  d_error = cte - prev_cte;
  i_error += cte;
  steer = - p[0] * cte - p[1] * d_error - p[2] * i_error;
	
  // Store the cte in prev_cte for derivative term
  prev_cte = cte;	
  return steer;
}

void PID::Twiddle() {
	sum_dp = 0;
	
	// Sum the P,I,D controller parameter changes 
	for (int i = 0; i < 3; i++) {
		sum_dp += dp[i];
	} 
	
	// Stage 1 Update	
  if (sum_dp > tol and stage == 1) {
		p[param] += dp[param];
		error = 0;
	}
  
	// Stage 2 Update
	if (sum_dp > tol and stage == 2) { 
		if (error < best_error) {
        best_error = error;
        dp[param] *= 1.1;
				error = 0;
    } else {
        p[param] -= 2 * dp[param];
				error = 0;
		}
	}
	
	// Stage 3 Update		
  if (sum_dp > tol and stage == 3) { 
    if (error < best_error) {
      best_error = error;
      dp[param] *= 1.1;
			error = 0;
    } else {
      p[param] += dp[param];
      dp[param] *= 0.9;
			error = 0;
    }
  }
}

double PID::TotalError(double cte) {
  iter += 1;
  error += cte * cte / iter;

  // Activate twiddle
  if (iter == twiddle_update) {
  	best_error = error;
    Twiddle();
		stage += 1;
  }
	
	// Only update PID parameters every 500 timesteps
  if (iter > twiddle_update and iter % twiddle_update == 0) {
	  Twiddle();
		error = 0;
		iter = 1000;
		stage += 1;
		if (stage > 3) {
			stage = 1;
		  param += 1;
			if (param > 2) {param = 0;}
		}
	}
  return error;
}

