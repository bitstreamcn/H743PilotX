/*
 * Servo.cpp
 *
 *  Created on: May 4, 2026
 *      Author: bitstream
 */
#include "Servo.h"
#ifdef __cplusplus
extern "C" {
#endif

#include "../Device/Motor/dshot.h"

#ifdef __cplusplus
}
#endif

#include "tim.h"


static void Init();
static void Write(float *motor_speeds);

Servo_s Servo = {
		Init,
		Write
};


static void Init()
{
	//HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	dshot_init(DSHOT300);
}

static void Write(float *motor_speeds)
{
    //发送电机控制
    dshot_write(motor_speeds);
}





