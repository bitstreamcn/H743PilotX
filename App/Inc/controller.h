/*
 * controller.h
 *
 *  Created on: May 7, 2026
 *      Author: bitstream
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "pid.h"
#include "common.h"

class Controller
{
public:
	Controller();
	~Controller();

	void Init();
	void PID_Update();
	void PID_Calculation();
	void climb();

	float motor_speeds[4];
	float yaw_heading_reference = 0;

private:
	//高度自动校准
	float cal_alt = 0;

	FirstOrderLPF thr_lpf = {0.1f, 0.0f};
};
#endif /* INC_CONTROLLER_H_ */
