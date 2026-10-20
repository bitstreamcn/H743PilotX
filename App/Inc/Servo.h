/*
 * Servo.h
 *
 *  Created on: May 4, 2026
 *      Author: bitstream
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include <stdint.h>

typedef struct{
	void ( *Init )();
	void ( *Write )(float *motor_speeds);
}Servo_s;

extern Servo_s Servo;



#endif /* INC_SERVO_H_ */
