/*
 * bottom.h
 *
 *  Created on: 2026年7月18日
 *      Author: bitstream
 */

#ifndef INC_BOTTOM_H_
#define INC_BOTTOM_H_


#include <stdint.h>
#include <stdbool.h>
#include "common.h"

#ifdef __cplusplus

class Bottom{
public:
	bool Init();
	bool Update();
	void WhoAmI();
	bool SetAhr(float roll, float pitch, float yaw, float gx, float gy);
	bool Reset(float yaw);

	float alt;
	float x, y;
	float x1, y1; //实时位移，单位mm
	uint16_t quality;
private:

	float offset_x = 0;
	float offset_y = 0;
};

extern Bottom bottom;

#endif

#endif /* INC_BOTTOM_H_ */
