/*
 * PID.c
 *
 *  Created on: Jun 21, 2025
 *      Author: bitstream
 */

#include "pid.h"
#include <stdbool.h>
#include "Config.h"
#include <math.h>

PID PIDroll;
PID PIDpitch;
PID PIDyaw;
PID PIDPmw;
PID PIDAcc;
PID PIDAccAlt;
PID PIDGps;

// 🚨 新增：安全限幅宏和 NaN/Inf 检查宏
#define LIMIT(x, min, max) (((x) > (max)) ? (max) : (((x) < (min)) ? (min) : (x)))
#define IS_INVALID(x) (isnan(x) || isinf(x))

// 基准 = max(设定点 * kp, ki * limit_i)
// D_max = 基准 * 0.5f
#define CALC_D_TERM_MAX(ref, kp, ki, limit_i) \
    ( (fabsf(ref * kp) > fabsf(ki * limit_i) ? fabsf(ref * kp) : fabsf(ki * limit_i)) * 0.5f )

void InitPid(PID * pid, float scale, float limit)
{
	memset((void*)pid, 0, sizeof(pid));
	pid->kscale = scale;
	pid->klimit_i = limit;
}

float control(PID * pid, float actual, float desired, float dt) {
	if (dt == 0)
	{
		//防止除0得到NaN
		dt = 0.001;
	}
	float err = desired - actual;
	// I 项累加前先检查 err 是否合法
	if (IS_INVALID(err))
	{
	  return pid->pid;
	}
	pid->err_i += err * dt;
	//Saturate integrator to prevent unsafe buildup

	if(pid->err_i < -(pid->klimit_i)) pid->err_i = -(pid->klimit_i);
	if(pid->err_i > pid->klimit_i) pid->err_i = pid->klimit_i;

	float i = pid->ki * pid->err_i;

	float err_d = (err - pid->err_prev) / dt;

	float p = pid->kp * err;
	float current_d_max = CALC_D_TERM_MAX(desired, pid->kp, pid->ki, pid->klimit_i);

	float d = pid->kd * err_d;
	d = LIMIT(d, -current_d_max, current_d_max);

	pid->pid = pid->kscale * (p + i + d);

	if (IS_INVALID(pid->pid))
	{
		pid->pid = 0;
	}

	pid->err_prev = err;
	return pid->pid;
}


#define DT 0.005f
#define OUTER_DERIV_FILT_ENABLE 0
#define INNER_DERIV_FILT_ENABLE 0

PIDDouble db_roll;
PIDDouble db_pitch;
PIDSingle yaw_heading;
PIDSingle yaw_rate;
PIDDouble db_thr;

void InitAllPid()
{
	InitPid(&PIDPmw, 1.0, cfg.param.pid_pmw_il);
	PIDPmw.kp = cfg.param.pid_pmw_p;
	PIDPmw.ki = cfg.param.pid_pmw_i;
	PIDPmw.kd = cfg.param.pid_pmw_d;
	InitPid(&PIDAcc, 1.0, cfg.param.pid_acc_il);
	PIDAcc.kp = cfg.param.pid_acc_p;
	PIDAcc.ki = cfg.param.pid_acc_i;
	PIDAcc.kd = cfg.param.pid_acc_d;
	InitPid(&PIDAccAlt, 1.0, cfg.param.pid_acc_alt_il);
	PIDAccAlt.kp = cfg.param.pid_acc_alt_p;
	PIDAccAlt.ki = cfg.param.pid_acc_alt_i;
	PIDAccAlt.kd = cfg.param.pid_acc_alt_d;
	InitPid(&PIDGps, 1.0, cfg.param.pid_gps_il);
	PIDGps.kp = cfg.param.pid_gps_p;
	PIDGps.ki = cfg.param.pid_gps_i;
	PIDGps.kd = cfg.param.pid_gps_d;

	memset((void*)&db_roll, 0, sizeof(db_roll));
	memset((void*)&db_pitch, 0, sizeof(db_pitch));
	memset((void*)&yaw_heading, 0, sizeof(yaw_heading));
	memset((void*)&yaw_rate, 0, sizeof(yaw_rate));
	memset((void*)&db_thr, 0, sizeof(db_thr));

	db_roll.in.kp = cfg.param.pid_in_roll_p;
	db_pitch.in.kp = cfg.param.pid_in_pitch_p;
	db_roll.in.ki = cfg.param.pid_in_roll_i;
	db_pitch.in.ki = cfg.param.pid_in_pitch_i;
	db_roll.in.kd = cfg.param.pid_in_roll_d;
	db_pitch.in.kd = cfg.param.pid_in_pitch_d;
	db_roll.out.kp = cfg.param.pid_ou_roll_p;
	db_pitch.out.kp = cfg.param.pid_ou_pitch_p;
	db_roll.out.ki = cfg.param.pid_ou_roll_i;
	db_pitch.out.ki = cfg.param.pid_ou_pitch_i;
	db_roll.out.kd = cfg.param.pid_ou_roll_d;
	db_pitch.out.kd = cfg.param.pid_ou_pitch_d;
	db_roll.in.limit_i = cfg.param.pid_in_roll_il;
	db_roll.out.limit_i = cfg.param.pid_ou_roll_il;
	db_pitch.in.limit_i = cfg.param.pid_in_pitch_il;
	db_pitch.out.limit_i = cfg.param.pid_ou_pitch_il;
	yaw_heading.kp = cfg.param.pid_yaw_p;
	yaw_heading.ki = cfg.param.pid_yaw_i;
	yaw_heading.limit_i = cfg.param.pid_yaw_il;
	yaw_heading.kd = cfg.param.pid_yaw_d;
	yaw_rate.kp = cfg.param.pid_yaw_rate_p;
	yaw_rate.ki = cfg.param.pid_yaw_rate_i;
	yaw_rate.limit_i = cfg.param.pid_yaw_rate_il;
	yaw_rate.kd = cfg.param.pid_yaw_rate_d;
	db_thr.in.kp = cfg.param.pid_in_thr_p;
	db_thr.in.ki = cfg.param.pid_in_thr_i;
	db_thr.in.kd = cfg.param.pid_in_thr_d;
	db_thr.in.limit_i = cfg.param.pid_in_thr_il;
	db_thr.out.kp = cfg.param.pid_ou_thr_p;
	db_thr.out.ki = cfg.param.pid_ou_thr_i;
	db_thr.out.kd = cfg.param.pid_ou_thr_d;
	db_thr.out.limit_i = cfg.param.pid_ou_thr_il;
}

void UpdateAllPid()
{
	PIDPmw.kscale = 1.0;
	PIDPmw.klimit_i = cfg.param.pid_pmw_il;
	PIDPmw.kp = cfg.param.pid_pmw_p;
	PIDPmw.ki = cfg.param.pid_pmw_i;
	PIDPmw.kd = cfg.param.pid_pmw_d;
	PIDAcc.kscale = 1.0;
	PIDAcc.klimit_i = cfg.param.pid_acc_il;
	PIDAcc.kp = cfg.param.pid_acc_p;
	PIDAcc.ki = cfg.param.pid_acc_i;
	PIDAcc.kd = cfg.param.pid_acc_d;
	PIDAccAlt.kscale = 1.0;
	PIDAccAlt.klimit_i = cfg.param.pid_acc_alt_il;
	PIDAccAlt.kp = cfg.param.pid_acc_alt_p;
	PIDAccAlt.ki = cfg.param.pid_acc_alt_i;
	PIDAccAlt.kd = cfg.param.pid_acc_alt_d;
	PIDGps.kscale = 1.0;
	PIDGps.klimit_i = cfg.param.pid_gps_il;
	PIDGps.kp = cfg.param.pid_gps_p;
	PIDGps.ki = cfg.param.pid_gps_i;
	PIDGps.kd = cfg.param.pid_gps_d;

	db_roll.in.kp = cfg.param.pid_in_roll_p;
	db_pitch.in.kp = cfg.param.pid_in_pitch_p;
	db_roll.in.ki = cfg.param.pid_in_roll_i;
	db_pitch.in.ki = cfg.param.pid_in_pitch_i;
	db_roll.in.kd = cfg.param.pid_in_roll_d;
	db_pitch.in.kd = cfg.param.pid_in_pitch_d;
	db_roll.out.kp = cfg.param.pid_ou_roll_p;
	db_pitch.out.kp = cfg.param.pid_ou_pitch_p;
	db_roll.out.ki = cfg.param.pid_ou_roll_i;
	db_pitch.out.ki = cfg.param.pid_ou_pitch_i;
	db_roll.out.kd = cfg.param.pid_ou_roll_d;
	db_pitch.out.kd = cfg.param.pid_ou_pitch_d;
	db_roll.in.limit_i = cfg.param.pid_in_roll_il;
	db_roll.out.limit_i = cfg.param.pid_ou_roll_il;
	db_pitch.in.limit_i = cfg.param.pid_in_pitch_il;
	db_pitch.out.limit_i = cfg.param.pid_ou_pitch_il;
	yaw_heading.kp = cfg.param.pid_yaw_p;
	yaw_heading.ki = cfg.param.pid_yaw_i;
	yaw_heading.limit_i = cfg.param.pid_yaw_il;
	yaw_heading.kd = cfg.param.pid_yaw_d;
	yaw_rate.kp = cfg.param.pid_yaw_rate_p;
	yaw_rate.ki = cfg.param.pid_yaw_rate_i;
	yaw_rate.limit_i = cfg.param.pid_yaw_rate_il;
	yaw_rate.kd = cfg.param.pid_yaw_rate_d;
	db_thr.in.kp = cfg.param.pid_in_thr_p;
	db_thr.in.ki = cfg.param.pid_in_thr_i;
	db_thr.in.kd = cfg.param.pid_in_thr_d;
	db_thr.in.limit_i = cfg.param.pid_in_thr_il;
	db_thr.out.kp = cfg.param.pid_ou_thr_p;
	db_thr.out.ki = cfg.param.pid_ou_thr_i;
	db_thr.out.kd = cfg.param.pid_ou_thr_d;
	db_thr.out.limit_i = cfg.param.pid_ou_thr_il;
}

void Reset_PID_Integrator(PIDSingle* axis)
{
	axis->error_sum = 0;
	axis->error_deriv_filt = 0;
	axis->meas_value_prev = 0;
}

void Reset_All_PID_Integrator(void)
{
	Reset_PID_Integrator(&db_roll.in);
	Reset_PID_Integrator(&db_roll.out);
	Reset_PID_Integrator(&db_pitch.in);
	Reset_PID_Integrator(&db_pitch.out);
	Reset_PID_Integrator(&yaw_heading);
	Reset_PID_Integrator(&yaw_rate);

	PIDPmw.err_i = 0;
	PIDPmw.err_prev = 0;
	PIDPmw.pid = 0;
	PIDAccAlt.err_i = 0;
	PIDAccAlt.err_prev = 0;
	PIDAccAlt.pid = 0;
	PIDAccAlt.err_i = 0;
	PIDAccAlt.err_prev = 0;
	PIDAccAlt.pid = 0;
	PIDGps.err_i = 0;
	PIDGps.err_prev = 0;
	PIDGps.pid = 0;
}

void Double_PID_Calculation(PIDDouble* axis, float set_point_angle, float angle/*BNO080 Rotation Angle*/, float rate/*ICM-20602 Angular Rate*/, float dt)
{
	if (dt <= 0.0f || IS_INVALID(dt))
	{
		dt = DT;
	}

	// 输入合法性检查
	if (IS_INVALID(set_point_angle) || IS_INVALID(angle) || IS_INVALID(rate)) return;

	/*********** Double PID Outer Begin (Roll and Pitch Angular Position Control) *************/
	axis->out.reference = set_point_angle;	//Set point of outer PID control
	axis->out.meas_value = angle;			//BNO080 rotation angle

	axis->out.error = axis->out.reference - axis->out.meas_value;	//Define error of outer loop
	axis->out.p_result = axis->out.error * axis->out.kp;			//Calculate P result of outer loop

	axis->out.error_sum = axis->out.error_sum + axis->out.error * dt;	//Define summation of outer loop
#if 0
#define OUT_ERR_SUM_MAX 25
#define OUT_I_ERR_MIN -OUT_ERR_SUM_MAX
	if(axis->out.error_sum > OUT_ERR_SUM_MAX) axis->out.error_sum = OUT_ERR_SUM_MAX;
	else if(axis->out.error_sum < OUT_I_ERR_MIN) axis->out.error_sum = OUT_I_ERR_MIN;
#endif

	if(axis->out.error_sum > axis->out.limit_i) axis->out.error_sum = axis->out.limit_i;
	else if(axis->out.error_sum < -axis->out.limit_i) axis->out.error_sum = -axis->out.limit_i;
	axis->out.i_result = axis->out.error_sum * axis->out.ki;			//Calculate I result of outer loop

	axis->out.error_deriv = -rate;										//Define derivative of outer loop (rate = ICM-20602 Angular Rate)

#if !OUTER_DERIV_FILT_ENABLE
	axis->out.d_result = axis->out.error_deriv * axis->out.kd;			//Calculate D result of outer loop
#else
	axis->out.error_deriv_filt = axis->out.error_deriv_filt * 0.4f + axis->out.error_deriv * 0.6f;	//filter for derivative
	axis->out.d_result = axis->out.error_deriv_filt * axis->out.kd;									//Calculate D result of inner loop
#endif

	// 🚨 动态限幅：根据内环的 kp 动态计算 D 项的极限
	float out_d_max = CALC_D_TERM_MAX(axis->out.reference, axis->out.kp, axis->out.ki, axis->out.limit_i);
	axis->out.d_result = LIMIT(axis->out.d_result, -out_d_max, out_d_max);

	axis->out.pid_result = axis->out.p_result + axis->out.i_result + axis->out.d_result;  //Calculate PID result of outer loop
	/****************************************************************************************/

	/************ Double PID Inner Begin (Roll and Pitch Angular Rate Control) **************/
	axis->in.reference = axis->out.pid_result;	//Set point of inner PID control is the PID result of outer loop (for double PID control)
	axis->in.meas_value = rate;					//ICM-20602 angular rate

	axis->in.error = axis->in.reference - axis->in.meas_value;	//Define error of inner loop
	axis->in.p_result = axis->in.error * axis->in.kp;			//Calculate P result of inner loop

	axis->in.error_sum = axis->in.error_sum + axis->in.error * dt;	//Define summation of inner loop
#if 0
#define IN_ERR_SUM_MAX 25
#define IN_I_ERR_MIN -IN_ERR_SUM_MAX
	if(axis->in.error_sum > IN_ERR_SUM_MAX) axis->in.error_sum = IN_ERR_SUM_MAX;
	else if(axis->in.error_sum < IN_I_ERR_MIN) axis->in.error_sum = IN_I_ERR_MIN;
#endif
	if(axis->in.error_sum > axis->in.limit_i) axis->in.error_sum = axis->in.limit_i;
	else if(axis->in.error_sum < -axis->in.limit_i) axis->in.error_sum = -axis->in.limit_i;
	axis->in.i_result = axis->in.error_sum * axis->in.ki;							//Calculate I result of inner loop

	//axis->in.error_deriv = -(axis->in.meas_value - axis->in.meas_value_prev) / dt;	//Define derivative of inner loop
	axis->in.error_deriv = -(axis->in.meas_value - axis->in.meas_value_prev);
	axis->in.meas_value_prev = axis->in.meas_value;									//Refresh value_prev to the latest value

#if !INNER_DERIV_FILT_ENABLE
	axis->in.d_result = axis->in.error_deriv * axis->in.kd;				//Calculate D result of inner loop
#else
	axis->in.error_deriv_filt = axis->in.error_deriv_filt * 0.5f + axis->in.error_deriv * 0.5f;	//filter for derivative
	axis->in.d_result = axis->in.error_deriv_filt * axis->in.kd;								//Calculate D result of inner loop
#endif

	// 🚨 动态限幅：根据内环的 kp 动态计算 D 项的极限
	float in_d_max = CALC_D_TERM_MAX(axis->in.reference, axis->in.kp, axis->in.ki, axis->in.limit_i);
	axis->in.d_result = LIMIT(axis->in.d_result, -in_d_max, in_d_max);

	axis->in.pid_result = axis->in.p_result + axis->in.i_result + axis->in.d_result; //Calculate PID result of inner loop
	/****************************************************************************************/
	// 🚨 修复5：使用 isfinite 替代 _isnan，同时防 NaN 和 Inf
	if (IS_INVALID(axis->in.pid_result))
	{
		axis->in.pid_result = 0;
	}
}

void Single_Yaw_Heading_PID_Calculation(PIDSingle* axis, float set_point_angle, float angle/*BNO080 Rotation Angle*/, float rate/*ICM-20602 Angular Rate*/, float dt)
{
	if (dt <= 0.0f || IS_INVALID(dt))
	{
		dt = DT;
	}
	if (IS_INVALID(set_point_angle) || IS_INVALID(angle) || IS_INVALID(rate)) return;
	/*********** Single PID Begin (Yaw Angular Position) *************/
	axis->reference = set_point_angle;	//Set point of yaw heading @ yaw stick is center.
	axis->meas_value = angle;			//Current BNO080_Yaw angle @ yaw stick is center.

	axis->error = axis->reference - axis->meas_value;	//Define error of yaw angle control

	if(axis->error > 180.f) axis->error -= 360.f;
	else if(axis->error < -180.f) axis->error += 360.f;

	axis->p_result = axis->error * axis->kp;			//Calculate P result of yaw angle control

	axis->error_sum = axis->error_sum + axis->error * dt;	//Define summation of yaw angle control
#if 0
#define IN_ERR_SUM_MAX 10
#define IN_I_ERR_MIN -IN_ERR_SUM_MAX
	if(axis->error_sum > IN_ERR_SUM_MAX) axis->error_sum = IN_ERR_SUM_MAX;
	else if(axis->error_sum < IN_I_ERR_MIN) axis->error_sum = IN_I_ERR_MIN;
#endif
	if(axis->error_sum > axis->limit_i) axis->error_sum = axis->limit_i;
	else if(axis->error_sum < -axis->limit_i) axis->error_sum = -axis->limit_i;
	axis->i_result = axis->error_sum * axis->ki;			//Calculate I result of yaw angle control

	axis->error_deriv = -rate;						//Define differentiation of yaw angle control
	axis->d_result = axis->error_deriv * axis->kd;	//Calculate D result of yaw angle control

	float yaw_head_d_max = CALC_D_TERM_MAX(axis->reference, axis->kp, axis->ki, axis->limit_i);
	axis->d_result = LIMIT(axis->d_result, -yaw_head_d_max, yaw_head_d_max);

	axis->pid_result = axis->p_result + axis->i_result + axis->d_result; //Calculate PID result of yaw angle control
	/***************************************************************/
	if (IS_INVALID(axis->pid_result))
	{
		axis->pid_result = 0;
	}
}

void Single_Yaw_Rate_PID_Calculation(PIDSingle* axis, float set_point_rate, float rate/*ICM-20602 Angular Rate*/, float dt)
{
    if (dt <= 0.0f || IS_INVALID(dt))
    {
        dt = DT;
    }
    if (IS_INVALID(set_point_rate) || IS_INVALID(rate)) return;

	/*********** Single PID Begin (Yaw Angular Rate Control) *************/
	axis->reference = set_point_rate;	//Set point of yaw heading @ yaw stick is not center.
	axis->meas_value = rate;			//Current ICM20602.gyro_z @ yaw stick is not center.

	axis->error = axis->reference - axis->meas_value;	//Define error of yaw rate control
	axis->p_result = axis->error * axis->kp;			//Calculate P result of yaw rate control

	axis->error_sum = axis->error_sum + axis->error * dt;	//Define summation of yaw rate control
#if 0
#define IN_ERR_SUM_MAX 10
#define IN_I_ERR_MIN -IN_ERR_SUM_MAX
	if(axis->error_sum > IN_ERR_SUM_MAX) axis->error_sum = IN_ERR_SUM_MAX;
	else if(axis->error_sum < IN_I_ERR_MIN) axis->error_sum = IN_I_ERR_MIN;
#endif
	if(axis->error_sum > axis->limit_i) axis->error_sum = axis->limit_i;
	else if(axis->error_sum < -axis->limit_i) axis->error_sum = -axis->limit_i;
	axis->i_result = axis->error_sum * axis->ki;			//Calculate I result of yaw rate control

	//axis->error_deriv = -(axis->meas_value - axis->meas_value_prev) / dt;	//Define differentiation of yaw rate control
	axis->error_deriv = -(axis->meas_value - axis->meas_value_prev);
	axis->meas_value_prev = axis->meas_value;								//Refresh value_prev to the latest value
	axis->d_result = axis->error_deriv * axis->kd;							//Calculate D result of yaw rate control
	// 🚨 修复7：Yaw 角速度环 D 项限幅
	float yaw_rate_d_max = CALC_D_TERM_MAX(axis->reference, axis->kp, axis->ki, axis->limit_i);
	axis->d_result = LIMIT(axis->d_result, -yaw_rate_d_max, yaw_rate_d_max);

	axis->pid_result = axis->p_result + axis->i_result + axis->d_result; //Calculate PID result of yaw control
	/*******************************************************************/
    if (IS_INVALID(axis->pid_result))
    {
        axis->pid_result = 0;
    }
}

