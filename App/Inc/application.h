/*
 * application.h
 *
 *  Created on: Apr 2, 2026
 *      Author: bitstream
 */

#ifndef INC_APPLICATION_H_
#define INC_APPLICATION_H_

#include <stdbool.h>
#include <stdint.h>
#include "configlink.h"
#include "controller.h"
#include "flow_kalman.h"
#include "car.h"

//使用扩展IMU
//#define USE_IMUEX 1

#ifdef __cplusplus
extern "C" {
#endif

#include "../Device/Radio/crsf2.h"

#ifdef __cplusplus
}
#endif

class Application
{
public:
	Application();
	~Application();

	void Init();
	void Loop();
	bool CheckLoopTimeout();
	void Tim200hz();

	bool IsArmed(){return armed;}
	bool IsImuFail(){return imufail;}
	enum eFlightMode Mode(){return mode;}

	void ImuEvent();

	void Exec200hz();

	void RadioUpdateEvent();

	//检查传感器线程是否死掉
	bool CheckUpdateTimeout();
	//检查气压计是否死掉
	bool CheckBmpTimeout();
	//检查气压计是否正常
	bool IsBmpDataValid(float bmpalt);

	void Exec500hz();
	void _500Hz_Event();
	void BmpIrqEvent();
	void BottomEvent();

	// 获取当前位置和速度
	void Flow_GetState(float *pos_x, float *pos_y, float *vel_x, float *vel_y);

	void gps();
	void UpdateGPS(float gps_px, float gps_py, float gps_vx, float gps_vy, float hAcc);

	float GetFrontDistance();

private:
	volatile bool mIsInit = false;
	volatile uint8_t _200hz;
	volatile uint32_t mLoopTick = 0;

	float mBmpLandAlt = 0;
	float mBmpAlt = 0;
	volatile int bmp_irq = 0;
	FirstOrderLPF bmp_lpf = {0.2f, 0.0f};

	float mTofAlt = 0;
	bool  mIsTofUpdate = false;

	//tof气压计校准
	float mBmpCal = 0;

	void BmpUpdate();
	void Control();
	void ImuUpdate();
	void RadioUpdate();

	void LogUpdate();
	void BottomUpdate();

	void qmc5883l();

	void mag_cal();

	ConfigLink link;
	Controller controller;

	bool armed = false;
	bool imufail = false;
	enum eFlightMode mode;

	uint32_t imu_update_tick = 0;
	uint32_t radio_update_tick = 0;
	uint32_t bmp_update_tick = 0;
	uint32_t tofupdate_tick = 0;

	volatile uint8_t imu_irq = 0;
	volatile int _500hz_irq = 0;
	volatile int bottom_irq = 0;

	FlowKalman flow_kf;
	float last_flow_tick = 0;

	float qmc_yaw = 0;

	//超声波避障
	void FrontCheckUpdate();
	float front_distance = 0.0f;

	Car car;

};


extern Application app;



#endif /* INC_APPLICATION_H_ */
