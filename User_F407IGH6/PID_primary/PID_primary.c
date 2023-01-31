#include "PID_primary.h"


/**
  * @brief     calculate delta PID and position PID
  * @param[in] pid: control pid struct
  * @param[in] get: measure feedback value
  * @param[in] set: target value
  * @retval    pid calculate output
  */
float Pid_Calculate(PID_Typedef *pid, float get, float set)
{
//	uint8_t flag = 0;
	//get input
	pid->input.get = get;
	pid->input.set = set;
	pid->input.last_err = pid->input.err;
	pid->input.err = set - get;
	//死区控制
//	if(fabs(pid->input.err) < 0.5)
//		{
//		pid->output.i_out = 0;
//		return 0;
//	//input limit
//		}
	if ((pid->limit.max_err_input != 0) && (fabs(pid->input.err) > pid->limit.max_err_input))
		return 0;

	//output calculate
	pid->output.p_out  = pid->param.Kp * pid->input.err;
	pid->output.i_out += pid->param.Ki * pid->input.err;
	pid->output.d_out  = pid->param.Kd * (pid->input.err - pid->input.last_err) * 1000;
	//output limit
	pid->output.i_out	  = limit_control(pid->output.i_out, pid->limit.max_i_out);
	pid->output.total_out = pid->output.p_out + pid->output.i_out + pid->output.d_out;
	pid->output.total_out = limit_control(pid->output.total_out, pid->limit.max_total_out);

	return pid->output.total_out;
}
/**
  * @brief     calculate delta PID and position PID
  * @param[in] pid: control pid struct
  * @param[in] get: measure feedback value
  * @param[in] set: target value
  * @retval    pid calculate output
  */
float Pid_Calculate_Sep(PID_Typedef *pid, float get, float set)
{
	//get input
	pid->input.get = get;
	pid->input.set = set;
	pid->input.last_err = pid->input.err;
	pid->input.err = set - get;
	//input limit
	if((pid->limit.max_err_input != 0) && (fabs(pid->input.err) > pid->limit.max_err_input))
		return 0;
	//output calculate
	pid->output.p_out  = pid->param.Kp * pid->input.err;
	pid->output.i_out += pid->param.Ki * pid->input.err;
	//积分分离
	if(pid->input.err > 90)
	{
		pid->output.i_out = 0;
	}
	pid->output.d_out  = pid->param.Kd * (pid->input.err - pid->input.last_err) * 1000;
	//output limit
	pid->output.i_out	  = limit_control(pid->output.i_out, pid->limit.max_i_out);
	pid->output.total_out = pid->output.p_out + pid->output.i_out + pid->output.d_out;
	pid->output.total_out = limit_control(pid->output.total_out, pid->limit.max_total_out);

	return pid->output.total_out;
}

float Pid_Increase(PID_Typedef *pid, float get, float set)
{
	static float accumulate = 0;
	//get input
	pid->input.get = get;
	pid->input.set = set;
	pid->input.previous_err = pid->input.last_err;
	pid->input.last_err = pid->input.err;
	pid->input.err = set - get;
	//死区控制
//	if(fabs(pid->input.err) < 0.5)
//		{
//		return accumulate;
//		}
	//input limit
	if((pid->limit.max_err_input != 0) && (fabs(pid->input.err) > pid->limit.max_err_input))
		return 0;
	//output calculate
	pid->output.p_out  = pid->param.Kp * (pid->input.err - pid->input.last_err);
	pid->output.i_out  = pid->param.Ki * pid->input.err;
	pid->output.d_out  = pid->param.Kd * (pid->input.err - 2 * pid->input.last_err - pid->input.previous_err);
	//output limit
	pid->output.total_out = pid->output.p_out + pid->output.i_out + pid->output.d_out;
	pid->output.total_out = limit_control(pid->output.total_out, pid->limit.max_total_out);

	accumulate += pid->output.total_out;
	return accumulate;
}


