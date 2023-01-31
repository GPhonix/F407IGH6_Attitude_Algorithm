/*
 * PID_primary.h
 *
 *  Created on: 2022年7月27日
 *      Author: Phonix
 */

#ifndef PID_PRIMARY_PID_PRIMARY_H_
#define PID_PRIMARY_PID_PRIMARY_H_
#include "main.h"
#include "math.h"
#include "Algorithms_Lib.h"
typedef  struct{
	struct{
		float set;
		float get;
		float err;
		float last_err;
		float previous_err;
	}input;
	struct{
		float Kp;
		float Ki;
		float Kd;
	}param;
	struct{
		float p_out;
		float i_out;
		float d_out;
		float total_out;
	}output;

	struct{
		float max_err_input;
		float max_i_out;
		float max_total_out;
	}limit;
}PID_Typedef;

float Pid_Calculate(PID_Typedef *pid, float get, float set);
float Pid_Calculate_Sep(PID_Typedef *pid, float get, float set);

float Pid_Increase(PID_Typedef *pid, float get, float set);


#endif /* PID_PRIMARY_PID_PRIMARY_H_ */
