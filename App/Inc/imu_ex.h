/*
 * imu_ex.h
 *
 *  Created on: May 11, 2026
 *      Author: bitstream
 */

#ifndef INC_IMU_EX_H_
#define INC_IMU_EX_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus

class ImuEx{
public:
	bool Init();
	bool Update();
	void WhoAmI();

	float ax, ay, az, gx, gy, gz;
	float roll, pitch, yaw;
	float alt, vz;
	float vx, vy;
private:

	/*
	 * 一阶低通滤波
	 */
	typedef struct {
		float alpha;           // 滤波系数
		float prev_output;     // 上一次输出值
	} FirstOrderLPF;
	float lpf_update(FirstOrderLPF *filter, float input) {
		// 计算本次输出: Y(n) = α * X(n) + (1-α) * Y(n-1)
		filter->prev_output = filter->alpha * input + (1.0f - filter->alpha) * filter->prev_output;
		return filter->prev_output;
	}

	FirstOrderLPF gx_lpf = {0.5f, 0.0f};
	FirstOrderLPF gy_lpf = {0.5f, 0.0f};
	FirstOrderLPF gz_lpf = {0.5f, 0.0f};

	FirstOrderLPF ax_lpf = {0.5f, 0.0f};
	FirstOrderLPF ay_lpf = {0.5f, 0.0f};
	FirstOrderLPF az_lpf = {0.5f, 0.0f};
};

#else
	float GetAlt();
#endif

#endif /* INC_IMU_EX_H_ */
