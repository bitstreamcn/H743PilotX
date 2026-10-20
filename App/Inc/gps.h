/*
 * gps.h
 *
 *  Created on: 2026年8月14日
 *      Author: bitstream
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include <stdint.h>

typedef struct __attribute__((packed)){
	volatile unsigned long iTOW;
	volatile uint8_t fixType; //0=无定位，2=2D定位，3=3D定位，4=GNSS+DR组合，5=RTK浮点，6=RTK固定。飞控必须判断此值 ≥3 才能使用 GPS 数据
	// 🚨 新增：参与定位的卫星数量
	volatile uint8_t numSV;
	volatile float fLon;
	volatile float fLat;
	volatile signed long hMSL;   //海拔高度。相对于平均海平面的高度（单位：毫米）。飞控通常用这个值与气压计或 IMU 融合来定高。
	volatile unsigned long hAcc; //水平定位精度因子（单位：毫米）。例如 hAcc = 500 表示水平误差在 0.5 米左右。飞控可用此值判断 GPS 信号好不好，太差就降低 GPS 权重。
	volatile signed long velN;  //北向速度（单位：毫米/秒）。向北为正，向南为负。
	volatile signed long velE;  //东向速度（单位：毫米/秒）。向东为正，向西为负。
	int16_t mx;
	int16_t my;
	int16_t mz;
	volatile uint8_t checksum;
} st_gps_data;

extern st_gps_data gps_data;
extern volatile bool uart2_data_recv;
extern volatile uint16_t uart2_data_size;
extern uint32_t gps_update_tick;

extern float hdistance; //家的距离
extern float distance;  //目标位置的距离

bool poll_read_gps();
void gps_hold();
void get_target_distance(float *x, float *y);
void target_to_home();
float heading_to_target();
float heading_to_home();

#endif /* INC_GPS_H_ */
