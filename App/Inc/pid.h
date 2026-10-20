/*
 * pid.h
 *
 *  Created on: Jun 21, 2025
 *      Author: bitstream
 */

#ifndef PID_H_
#define PID_H_

#include <string.h>
#include <stdint.h>

typedef struct {

	float pid; //PID output value

	float err_i; //integral error
	float err_prev; //previous error

	float kp; //Kp
	float ki; //Ki
	float kd; //Kd

	float kscale; //scale factor
	float klimit_i; //err_i is limited to -limit_i to +limit_i


} PID;

void InitPid(PID * pid, float scale, float limit);

float control(PID * pid, float actual, float desired, float dt);

extern PID PIDroll;
extern PID PIDpitch;
extern PID PIDyaw;
extern PID PIDPmw;
extern PID PIDAcc;
extern PID PIDAccAlt;
extern PID PIDGps;

typedef struct _PIDSingle
{
	float kp;
	float ki;
	float kd;

	float reference;
	float meas_value;
	float meas_value_prev;
	float error;
	float error_sum;
	float error_deriv;
	float error_deriv_filt;

	float limit_i;

	float p_result;
	float i_result;
	float d_result;

	float pid_result;
}PIDSingle;

typedef struct _PIDDouble
{
	PIDSingle in;
	PIDSingle out;
}PIDDouble;

extern PIDDouble db_roll;
extern PIDDouble db_pitch;
extern PIDSingle yaw_heading;
extern PIDSingle yaw_rate;
extern PIDDouble db_thr;

void InitAllPid();
void UpdateAllPid();
void Double_PID_Calculation(PIDDouble* axis, float set_point_angle, float angle, float rate, float dt);
void Single_Yaw_Rate_PID_Calculation(PIDSingle* axis, float set_point, float value, float dt);
void Single_Yaw_Heading_PID_Calculation(PIDSingle* axis, float set_point, float angle, float rate, float dt);
void Reset_PID_Integrator(PIDSingle* axis);
void Reset_All_PID_Integrator(void);

#endif /* PID_H_ */
