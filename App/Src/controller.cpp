/*
 * controller.cpp
 *
 *  Created on: May 7, 2026
 *      Author: bitstream
 */
#include "controller.h"
#include "CRSFRadio.h"
#include "Config.h"
#include <math.h>
#include "main.h"
#include "application.h"
#include "imu_ex.h"
#include "log.h"
#include "common.h"
#include "Common/kalman.h"
#include "bottom.h"
#include "gps.h"

//extern ImuEx imuex;
extern volatile bool IsConnect;
extern Kalman location_kalman;

extern float total_delta_x; //x累积位移
extern float total_delta_y; //y累积位移
extern float delta_x;
extern float delta_y;

#ifdef __cplusplus
extern "C" {
#endif

#include "../Device/Imu/imu.h"
#include "../Device/Alt/alt.h"
#include "../Device/Motor/dshot.h"

#ifdef __cplusplus
}
#endif

//单环控制，用于检查硬件是否正常有效
#define ANGLE 0

//IMPORTANT: This is a safety feature which keeps props spinning when armed, and hopefully reminds the pilot to disarm!!!
const float armed_min_throttle = 0.01; //Minimum throttle when armed, set to a value between ~0.10 and ~0.25 which keeps the props spinning at minimum speed.

//Controller parameters (take note of defaults before modifying!):
const float i_limit        = 25.0;      //Integrator saturation level, mostly for safety (default 25.0)
const float maxRoll        = 40.0;      //Max roll angle in degrees for angle mode (maximum ~70 degrees)
const float maxPitch       = 40.0;      //Max pitch angle in degrees for angle mode (maximum ~70 degrees)
const float maxRollRate    = 50.0;      //Max roll rate in deg/sec for rate mode
const float maxPitchRate   = 50.0;      //Max pitch rate in deg/sec for rate mode
const float maxYawRate     = 30.0;     //Max yaw rate in deg/sec for angle and rate mode


//returns angle in range -180 to 180
float degreeModulus(float v) {
  if(v >= 180) {
    return fmod(v + 180, 360) - 180;
  }else if(v < -180.0) {
    return fmod(v - 180, 360) + 180;
  }
  return v;
}

// 限制 val 在 [min_val, max_val] 范围内
static float constrain(float val, float min_val, float max_val)
{
    if (val < min_val) {
        return min_val;  // 如果 val 小于最小值，返回最小值
    } else if (val > max_val) {
        return max_val;  // 如果 val 大于最大值，返回最大值
    } else {
        return val;  // 如果 val 在范围内，返回 val
    }
}

float ChannelNormalize(int rev, int val, int min, int center, int max, int deadband) {
  //int rev = 1; //1=normal, -1=reverse channel
  //needs: min < center < max
  if(val<min) return rev * -1.0;
  if(val<center-deadband) return (float)(rev * (val-(center-deadband))) / ((center-deadband)-min); //returns -1 to 0
  if(val<center+deadband) return 0;
  if(val<max) return (float)(rev * (val-(center+deadband))) / (max-(center+deadband));
  return rev * 1.0;
}


Controller::Controller()
{

}

Controller::~Controller()
{

}


void Controller::Init()
{
	InitAllPid();
}

void Controller::PID_Update()
{
	UpdateAllPid();
}


void out_Mixer(int _CrsfThrottle, float thr, float *motor_speeds_value) {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes PIDroll.PID, PIDpitch.PID, and PIDyaw.PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +PIDroll.PID while the right two motors
   * should have -PIDroll.PID. Front two should have +PIDpitch.PID and the back two should have -PIDpitch.PID etc... every motor has
   * normalized (0 to 1) rcl.throttle command for throttle control. Can also apply direct unstabilized commands from the transmitter with
   * rcl.xxx variables are to be sent to the motor ESCs and servos.
   *
   *Relevant variables:
   *rcl.throtle - direct thottle control
   *PIDroll.PID, PIDpitch.PID, PIDyaw.PID - stabilized axis variables
   *rcl.roll, rcl.pitch, rcl.yaw - direct unstabilized command passthrough
   *rcl.flight_mode - can be used to toggle things with an 'if' statement
   */
/*
Motor order diagram (Betaflight order)

      front
 CW -->   <-- CCW
     4     2
      \ ^ /
       |X|
      / - \
     3     1
CCW -->   <-- CW

                                        M1234
Pitch up (stick back)   (front+ back-)   -+-+
Roll right              (left+ right-)   --++
Yaw right               (CCW+ CW-)       -++-
*/
	float throttle = 0.0;
	if (cfg.param.sys_fly_mode == 0)
	{
	#define RCL_THROTTLE_DEADBAND 10 //pwm deadband for zero throttle in microseconds
		//因为油门回中，先减500，下面再*2，如果不减500，就会出现还没起飞就50%油门
		_CrsfThrottle -= 500;
		if (_CrsfThrottle < 1000)
		{
			_CrsfThrottle = 1000;
		}

		//throttle: 0.0 in range from stick full back to rcl_cfg_thro_low, 1.0 on full throttle
		//float tlow = 1100 + RCL_THROTTLE_DEADBAND;
		//float throttle = constrain( ((float)(CrsfThrottle - tlow)) / (1900 - tlow), 0.0, 1.0);
		float tlow = cfg.param.rcl_thr_pull + cfg.param.rcl_thr_dead;
		throttle = constrain( ((float)(_CrsfThrottle - tlow)) / (cfg.param.rcl_thr_push - tlow) * 2, 0.0, 1.0);

	  // IMPORTANT: This is a safety feature to remind the pilot to disarm.
	  // Set motor outputs to at least armed_min_throttle, to keep at least one prop spinning when armed. The [out] module will disable motors when out.armed == false
	  /*float */thr = armed_min_throttle + (1 - armed_min_throttle) * throttle; //shift motor throttle range from [0.0 .. 1.0] to [armed_min_throttle .. 1.0]
	}
	else
	{
	  //float throttle = flight_thr;
	  throttle = 0.02;
	}
  if(throttle == 0) {
	//if throttle idle, then run props at low speed without applying PID. This allows for stick commands for arm/disarm.
	//out.set(0, thr);
	//out.set(1, thr);
	//out.set(2, thr);
	//out.set(3, thr);
	motor_speeds_value[MOTOR_FL] = thr;
	motor_speeds_value[MOTOR_FR] = thr;
	motor_speeds_value[MOTOR_BL] = thr;
	motor_speeds_value[MOTOR_BR] = thr;
  }else{
    // Quad mixing
    //out.set(0, thr - PIDpitch.PID - PIDroll.PID - PIDyaw.PID); //M1 Back Right CW
    //out.set(1, thr + PIDpitch.PID - PIDroll.PID + PIDyaw.PID); //M2 Front Right CCW
    //out.set(2, thr - PIDpitch.PID + PIDroll.PID + PIDyaw.PID); //M3 Back Left CCW
    //out.set(3, thr + PIDpitch.PID + PIDroll.PID - PIDyaw.PID); //M4 Front Left CW
	motor_speeds_value[MOTOR_FL] = thr + PIDpitch.pid + PIDroll.pid - PIDyaw.pid; //M4 Front Left CW
	motor_speeds_value[MOTOR_FR] = thr + PIDpitch.pid - PIDroll.pid + PIDyaw.pid; //M2 Front Right CCW
	motor_speeds_value[MOTOR_BL] = thr - PIDpitch.pid + PIDroll.pid + PIDyaw.pid; //M3 Back Left CCW
	motor_speeds_value[MOTOR_BR] = thr - PIDpitch.pid - PIDroll.pid - PIDyaw.pid; //M1 Back Right CW
  }

  for (int i = 0; i < 4; i++) {
      // 转速限幅
      if (motor_speeds_value[i] < 0.02)
      {
    	  motor_speeds_value[i] = 0.02;
      }
      /*
      if (motor_speeds_value[i] > 0.6)
      {
    	  motor_speeds_value[i] = 0.6;
      }
      */
  }
}



#if ANGLE
//PID Angle Mode
const float Kp_ro_pi_angle  = 0.2;      //Roll/Pitch P-gain
const float Ki_ro_pi_angle  = 0.1;      //Roll/Pitch I-gain
const float Kd_ro_pi_angle  = 0.05;     //Roll/Pitch D-gain
const float Kp_yaw_angle    = 0.6;      //Yaw P-gain
const float Kd_yaw_angle    = 0.1;      //Yaw D-gain

//PID Rate Mode
const float Kp_ro_pi_rate   = 0.15;     //Roll/Pitch rate P-gain
const float Ki_ro_pi_rate   = 0.2;      //Roll/Pitch rate I-gain
const float Kd_ro_pi_rate   = 0.0002;   //Roll/Pitch rate D-gain (be careful when increasing too high, motors will begin to overheat!)
const float Kp_yaw_rate     = 0.3;       //Yaw rate P-gain
const float Ki_yaw_rate     = 0.05;      //Yaw rate I-gain
const float Kd_yaw_rate     = 0.00015;   //Yaw rate D-gain (be careful when increasing too high, motors will begin to overheat!)

//Yaw to keep in ANGLE mode when yaw stick is centered
float yaw_desired = 0;


void control_Angle(bool zero_integrators, float roll_des, float pitch_des, float yawRate_des, float dt) {
  //DESCRIPTION: Computes control commands based on angle error
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des. Error
   * is simply the desired state minus the actual state (ex. roll_des - ahr.roll). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle... saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I
   * terms will always start from 0 on takeoff. This function updates the variables PIDroll.PID, PIDpitch.PID, and PIDyaw.PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in out_Mixer().
   */

  //inputs: roll_des, pitch_des, yawRate_des
  //outputs: PIDroll.PID, PIDpitch.PID, PIDyaw.PID

  //desired values
  //float roll_des = rcl.roll * maxRoll; //Between -maxRoll and +maxRoll
  //float pitch_des = rcl.pitch * maxPitch; //Between -maxPitch and +maxPitch
  //float yawRate_des = rcl.yaw * maxYawRate; //Between -maxYawRate roll_PIDand +maxYawRate

  //state vars
  static float integral_roll, integral_pitch, error_yawRate_prev, integral_yawRate;

  //Zero the integrators (used to don't let integrator build if throttle is too low, or to re-start the controller)
  if(zero_integrators) {
    integral_roll = 0;
    integral_pitch = 0;
    integral_yawRate = 0;
  }

  //Roll PID
  //float error_roll = roll_des - ahr.roll;
  float error_roll = roll_des - imuex.roll;
  //integral_roll += error_roll * imu.dt;
  integral_roll += error_roll * dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  //float derivative_roll = ahr.gx;
  float derivative_roll = imuex.gx;
  //PIDroll.pid = 0.01 * (Kp_ro_pi_angle*error_roll + Ki_ro_pi_angle*integral_roll - Kd_ro_pi_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range
  PIDroll.pid = 0.01 * (db_roll.out.kp*error_roll + db_roll.out.ki*integral_roll - db_roll.out.kd*derivative_roll);

  //Pitch PID
  //float error_pitch = pitch_des - ahr.pitch;
  float error_pitch = pitch_des - imuex.pitch;
  //integral_pitch += error_pitch * imu.dt;
  integral_pitch += error_pitch * dt;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  //float derivative_pitch = ahr.gy;
  float derivative_pitch = imuex.gy;
  //PIDpitch.pid = 0.01 * (Kp_ro_pi_angle*error_pitch + Ki_ro_pi_angle*integral_pitch - Kd_ro_pi_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range
  PIDpitch.pid = 0.01 * (db_pitch.out.kp*error_pitch + db_pitch.out.ki*integral_pitch - db_pitch.out.kd*derivative_pitch);

  //Yaw PID
  //if(-0.02 < rcl.yaw && rcl.yaw < 0.02)
  if(-5 < yawRate_des && yawRate_des < 5)
  {
    //on reset, set desired yaw to current yaw
    //if(zero_integrators) yaw_desired = ahr.yaw;
	  if(zero_integrators) yaw_desired = imuex.yaw;

    //Yaw stick centered: hold yaw_desired
    //float error_yaw = degreeModulus(yaw_desired - ahr.yaw);
    float error_yaw = degreeModulus(yaw_desired - imuex.yaw);
    float desired_yawRate = error_yaw / 0.5; //set desired yawRate such that it gets us to desired yaw in 0.5 second
    //float derivative_yaw = desired_yawRate - ahr.gz;
    float derivative_yaw = desired_yawRate - imuex.gz;
    //PIDyaw.pid = 0.01 * (Kp_yaw_angle*error_yaw + Kd_yaw_angle*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range
    PIDyaw.pid = 0.01 * (yaw_heading.kp*error_yaw + yaw_heading.kd*derivative_yaw);

    //update yaw rate controller
    error_yawRate_prev = 0;
  }else{
    //Yaw stick not centered: stablize on rate from GyroZ
    //float error_yawRate = yawRate_des - ahr.gz;
	  float error_yawRate = yawRate_des - imuex.gz;
    //integral_yawRate += error_yawRate * imu.dt;
	  integral_yawRate += error_yawRate * dt;
    integral_yawRate = constrain(integral_yawRate, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
    //float derivative_yawRate = (error_yawRate - error_yawRate_prev) / imu.dt;
    float derivative_yawRate = (error_yawRate - error_yawRate_prev) / dt;
    //PIDyaw.pid = 0.01 * (Kp_yaw_rate*error_yawRate + Ki_yaw_rate*integral_yawRate + Kd_yaw_rate*derivative_yawRate); //Scaled by .01 to bring within -1 to 1 range
    PIDyaw.pid = 0.01 * (yaw_rate.kp*error_yawRate + yaw_rate.ki*integral_yawRate + yaw_rate.kd*derivative_yawRate);

    //Update derivative variables
    error_yawRate_prev = error_yawRate;

    //update yaw controller:
    //yaw_desired = ahr.yaw; //set desired yaw to current yaw, the yaw angle controller will hold this value
    yaw_desired = imuex.yaw;
  }
}
#endif


/**
 * @brief 计算重力补偿系数 (补偿因机身倾斜导致的升力损失)
 * @param roll_deg  横滚角 (度)  <- 改为传入角度，更符合直觉
 * @param pitch_deg 俯仰角 (度)
 * @return      补偿系数 (1.0 / cosθ，限制最大补偿倍数为 1.414 即 45度)
 */
static float get_gravity_compensation(float roll_deg, float pitch_deg) {
#define DEG_TO_RAD 0.017453292519943295f
    // 1. 角度转弧度
    float roll_rad = roll_deg * DEG_TO_RAD;
    float pitch_rad = pitch_deg * DEG_TO_RAD;

    // 2. 计算总倾斜角的余弦值
    // 标准公式: cos(tilt) = cos(roll) * cos(pitch)
    float cos_val = cosf(roll_rad) * cosf(pitch_rad);

    // 3. 安全检查：防止 NaN
    if (isnan(cos_val)) {
        cos_val = 1.0f;
    }

    // 4. 限制最大补偿倍数为 1.414 (对应 45度倾斜)
    // 1 / 1.414 = 0.7071f
    if (cos_val < 0.7071f) {
        cos_val = 0.7071f;
    }
    if (cos_val > 1.0f) {
        cos_val = 1.0f;
    }

    return 1.0f / cos_val;
}


//inputs: roll_des, pitch_des, yawRate_des
//outputs: PIDroll.PID, PIDpitch.PID, PIDyaw.PID
void Controller::PID_Calculation()
{
	int _CrsfRoll = 1500;
	int _CrsfPitch = 1500;
	int _CrsfThrottle = 1500;
	int _CrsfYaw = 0;
	int _CrsfMode = 0;
	int _CrsfLight = 0;
	int _CrsfReturnHome = 0;
	int _CrsfArmed = 0;

	float rcl_roll_des;
	float rcl_pitch_des;
	//高度融合计算后的油门
	static float dshot_throttle = 0;
	static float dest_alt = 0;
	static bool good = false;

	CRSFRadio.GetData(&_CrsfRoll, &_CrsfPitch, &_CrsfThrottle, &_CrsfYaw, &_CrsfMode, &_CrsfLight, &_CrsfReturnHome, &_CrsfArmed);

	bool thrIdle = false;
	//检查油门是否居中
	if (_CrsfThrottle > (cfg.param.rcl_thr_mid - cfg.param.rcl_thr_dead) && _CrsfThrottle < (cfg.param.rcl_thr_mid + cfg.param.rcl_thr_dead))
	{
		thrIdle = true;
	}
	(void)thrIdle;
	bool rolIdle = false;
	//检查roll是否居中
	if (_CrsfRoll > (cfg.param.rcl_rol_mid - cfg.param.rcl_rol_dead) && _CrsfRoll < (cfg.param.rcl_rol_mid + cfg.param.rcl_rol_dead))
	{
		rolIdle = true;
	}
	bool pitIdle = false;
	if (_CrsfPitch > (cfg.param.rcl_pit_mid - (cfg.param.rcl_pit_dead + /*遥控器校准*/10)) && _CrsfPitch < (cfg.param.rcl_pit_mid + (cfg.param.rcl_pit_dead + /*遥控器校准*/10)))
	//检查pitch是否居中
	{
		pitIdle = true;
	}

	//检查飞机状态是否异常
	bool fail = false;
    if (app.Mode() != MODE_DRONE && app.Mode() != MODE_RETURN_HOME)
    {
    	fail = true;
    }
#ifndef USE_IMUEX
    if (roll < -80 || roll > 80 || pitch < -80 || pitch > 80)
#else
    if (imuex.roll < -130 || imuex.roll > 130 || imuex.pitch < -130 || imuex.pitch > 130)
#endif
    {
    	if (good)
    	{
    		PERIODIC_LOGE(1000, "roll pitch error: roll %.2f, pitch %.2f", roll, pitch);
    	}
    	fail = true;
    }
    if (!app.IsArmed())
    {
    	//锁定飞机后重置传油门和传感器校准
    	dshot_throttle = 0;
    	//气压计长时间会有漂移
    	//cal_alt = imuex.alt;
    	cal_alt = Alt_getH();
    	dest_alt = 0;
    	fail = true;
    }
    if (app.IsImuFail())
    {
    	if (good)
    	{
    		PERIODIC_LOGE(1000, "ImuFail");
    	}
    	fail = true;
    }
    //printf("IsConnect:%d\r\n", IsConnect);
    //检查遥控器
    static uint32_t connect_tim = 0;
    if (!IsConnect)
    {
    	if (good)
    	{
    		PERIODIC_LOGE(1000, "Radio Connect Fail");
    	}
    	if ((HAL_GetTick() - connect_tim) > 10000)
    	{
    		fail = true;
    	}
    }
    else
    {
    	connect_tim = HAL_GetTick();
    }
    //检查关键传感器线程是否死掉（imu, radio）
    if (app.CheckUpdateTimeout() || (app.CheckBmpTimeout() && cfg.param.sys_fly_mode != 0))
    {
    	if (good)
    	{
    		PERIODIC_LOGE(1000, "update time out");
    	}
    	fail = true;
    }

    if (fail)
    {
    	yaw_heading_reference = 0;
    	dshot_throttle = 0;

		motor_speeds[0] = 1000;
		motor_speeds[1] = 1000;
		motor_speeds[2] = 1000;
		motor_speeds[3] = 1000;

		Reset_All_PID_Integrator();
		good = false;
		return;
    }
    good = true;

	PID_Update();

	float yawRate_des = ChannelNormalize(1, _CrsfYaw, cfg.param.rcl_yaw_left, cfg.param.rcl_yaw_mid, cfg.param.rcl_yaw_right, cfg.param.rcl_yaw_dead) * maxYawRate;
	rcl_roll_des = ChannelNormalize(1, _CrsfRoll, cfg.param.rcl_rol_left, cfg.param.rcl_rol_mid, cfg.param.rcl_rol_right, cfg.param.rcl_rol_dead) * maxRoll; //Between -maxRoll and +maxRoll
	rcl_pitch_des = ChannelNormalize(-1, _CrsfPitch, cfg.param.rcl_pit_pull, cfg.param.rcl_pit_mid, cfg.param.rcl_pit_push, cfg.param.rcl_pit_dead) * maxPitch; //Between -maxPitch and +maxPitch
#if 0
	float tlow = cfg.param.rcl_thr_pull + cfg.param.rcl_thr_dead;
	float tlow1 = (cfg.param.rcl_thr_push - tlow);
	if (tlow1 == 0){
		tlow1 = 1.0;
	}
	float throttle = constrain( ((float)(_CrsfThrottle - tlow)) / tlow1, 0.0, 1.0);
#endif

	static uint32_t update_dt = 0;
	if (update_dt == 0)
	{
		update_dt = micros();
	}
	float remain_dt = (micros() - update_dt) / 1000000.0f;
	update_dt = micros();

	float yaw_des;
	float roll_des;
	float pitch_des;

	//平衡校准微调
	roll_des = rcl_roll_des + cfg.param.cal_roll;
	pitch_des = rcl_pitch_des + cfg.param.cal_pitch;
	yaw_des = yawRate_des/* + cfg.param.cal_yaw*/;


	float pos_x, pos_y, vel_x, vel_y;
	app.Flow_GetState(&pos_x, &pos_y, &vel_x, &vel_y);
    float dx_body, dy_body;
    get_target_distance(&dx_body, &dy_body);
    bool gpsctl = false;
	//如果roll居中，就通过传感器控制
	if (rolIdle)
	{
		//IMU加速度控制
		//float acc_roll = control(&PIDAcc, location_kalman.get_vx(), 0, remain_dt);
		//float acc_roll = control(&PIDAcc, imuex.ay, 0, remain_dt);
		//float acc_roll = control(&PIDAcc, imu.state.ay, 0, remain_dt);
		//(void)acc_roll;
		//roll_des += acc_roll;
		//PMW光流控制
		//float pmw_roll = control(&PIDPmw, bottom.x, 0, remain_dt);
		if( fabsf(dx_body) > 2 && gps_data.fixType >= 3 && gps_data.numSV >= 8 && gps_data.hAcc < 1000 && (HAL_GetTick() - gps_update_tick) < 1000)
		{
			//gps
			float gps_roll = control(&PIDGps, dx_body, 0, remain_dt);
			gps_roll = constrain(gps_roll, -10, 10);
			roll_des += gps_roll;
			gpsctl = true;
		}
		else
		{
			float pmw_roll = control(&PIDPmw, vel_x, 0, remain_dt);
			//控制限幅
			pmw_roll = constrain(pmw_roll, -20, 20);
			//Log::I("pmw_roll:%f - %f", bottom.x, pmw_roll);
			roll_des += pmw_roll;
		}
	}
	//如果pitch居中，就通过传感器控制
	if (pitIdle)
	{
		//IMU加速度控制
		//float acc_pitch = -1 * control(&PIDAcc, location_kalman.get_vy(), 0, remain_dt);
		//float acc_pitch = -1 * control(&PIDAcc, imuex.ax, 0, remain_dt);
		//float acc_pitch = -1 * control(&PIDAcc, imu.state.ax, 0, remain_dt);
		//(void)acc_pitch;
		//pitch_des += acc_pitch;
		//PMW光流控制
		//float pmw_pitch = -1 * control(&PIDPmw, bottom.y, 0, remain_dt);

		if (app.GetFrontDistance() < 2.0 && app.GetFrontDistance() > 0.2)
		{
			//前避障
			if (pitch_des < 5)
			{
				pitch_des = 5;
			}
		}
		else
		{
			if(  fabsf(dy_body) > 2 && gps_data.fixType >= 3 && gps_data.numSV >= 8 && gps_data.hAcc < 1000 && (HAL_GetTick() - gps_update_tick) < 1000)
			{
				float gps_pitch = -1 * control(&PIDGps, dy_body, 0, remain_dt);
				gps_pitch = constrain(gps_pitch, -10, 10);
				pitch_des += gps_pitch;
				gpsctl = true;
			}
			else
			{
				float pmw_pitch = -1 * control(&PIDPmw, vel_y, 0, remain_dt);
				//控制限幅
				pmw_pitch = constrain(pmw_pitch, -20, 20);
				//Log::I("pmw_pitch:%f - %f", bottom.y, pmw_pitch);
				pitch_des += pmw_pitch;
			}
		}
	}

	if (gpsctl)
	{
		bottom.Reset(yaw);
	}

	//限幅
	roll_des = constrain(roll_des, -maxRoll, maxRoll);
	pitch_des = constrain(pitch_des, -maxPitch, maxPitch);
	yaw_des = constrain(yaw_des, -maxYawRate, maxYawRate);

#if 0
	Log::I("[radio]roll:%.2f,pitch:%.2f,yaw:%.2f,throttle:%.2f", roll_des, pitch_des, yaw_des, throttle);
#endif

#if ANGLE
    control_Angle(_CrsfThrottle > (cfg.param.rcl_thr_mid - cfg.param.rcl_thr_dead) && _CrsfThrottle < (cfg.param.rcl_thr_mid + cfg.param.rcl_thr_dead),
    		roll_des, pitch_des, yaw_des, remain_dt);

    out_Mixer(_CrsfThrottle, 0, motor_speeds); //Mixes PID outputs and sends command pulses to the motors, if mot.arm == true

#else


#ifndef USE_IMUEX
	Double_PID_Calculation(&db_roll, roll_des, roll, imu.state.gx, remain_dt);
	Double_PID_Calculation(&db_pitch, pitch_des, pitch, imu.state.gy, remain_dt);
#else
	Double_PID_Calculation(&db_roll, roll_des, imuex.roll, imuex.gx, remain_dt);
	Double_PID_Calculation(&db_pitch, pitch_des, imuex.pitch, imuex.gy, remain_dt);

#if 0
	Log::I("[roll_out]err:%f,p_r:%f,sum:%f,i_r:%f,gx:%f,d_r:%f,o_r:%f",
			db_roll.out.error, db_roll.out.p_result, db_roll.out.error_sum, db_roll.out.i_result,
			db_roll.in.meas_value, db_roll.out.d_result, db_roll.out.pid_result);
	Log::I("[roll_in]err:%f,p_r:%f,sum:%f,i_r:%f,di:%f,d_r:%f,o_r:%f",
			db_roll.in.error, db_roll.in.p_result, db_roll.in.error_sum, db_roll.in.i_result,
			db_roll.in.error_deriv, db_roll.in.d_result, db_roll.in.pid_result);
#endif

#endif
	db_roll.in.pid_result = constrain(db_roll.in.pid_result, -maxRoll, maxRoll);
	db_pitch.in.pid_result = constrain(db_pitch.in.pid_result, -maxPitch, maxPitch);

	PIDroll.pid = 0.01 * db_roll.in.pid_result;
	PIDpitch.pid = 0.01 * db_pitch.in.pid_result;

	PIDroll.pid = constrain(PIDroll.pid, -0.8, 0.8);
	PIDpitch.pid = constrain(PIDpitch.pid, -0.8, 0.8);

	bool is_yaw_heading = false;
	if (yaw_des == 0)
	{
		is_yaw_heading = true;
	}
#ifndef USE_IMUEX
	float imu_yaw = yaw;
#else
	float imu_yaw = imuex.yaw;
#endif
	if (is_yaw_heading)
	{
		if (yaw_heading_reference == 0/* || mode == MODE_CAR*/)
		{
			yaw_heading_reference = imu_yaw;
		}
#ifndef USE_IMUEX
		Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference, imu_yaw, imu.state.gz, remain_dt);
#else
		Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference, imu_yaw, imuex.gz, remain_dt);
#endif
		PIDyaw.pid = 0.01 * yaw_heading.pid_result;
		PIDyaw.pid = constrain(PIDyaw.pid, -0.3, 0.3);
	}
	else
	{
		yaw_heading_reference = imu_yaw;
#ifndef USE_IMUEX
		//Single_Yaw_Rate_PID_Calculation(&yaw_rate, yaw_des, imu.state.gz, remain_dt);
		Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_des, 0, imu.state.gz, remain_dt);
#else
		Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_des, 0, imuex.gz, remain_dt);
#endif
		PIDyaw.pid = 0.01 * yaw_heading.pid_result;
		PIDyaw.pid = constrain(PIDyaw.pid, -0.3, 0.3);
	}

	//计算油门比例
	float throttle = ChannelNormalize(1, _CrsfThrottle, cfg.param.rcl_thr_pull, cfg.param.rcl_thr_mid, cfg.param.rcl_thr_push, cfg.param.rcl_thr_dead);
	//根据油门比例计算目的高度
	//float current_alt = imuex.alt - cal_alt;
	float current_alt = Alt_getH() - cal_alt;
	static bool ischange_alt = false;
	if (thrIdle)
	{
		if (ischange_alt)
		{
			//dest_alt = current_alt;
		}
		ischange_alt = false;
	}
	else
	{
		static bool up = false;
		if (throttle > 0)
		{
			ischange_alt = true;
			float new_alt = current_alt + (throttle * 3.0);
			if (new_alt > dest_alt)
			{
				dest_alt = new_alt;
			}
			up = true;
		}
		else if (throttle < 0)
		{
			static float down_offset = 0.0f;
			if (up)
			{
				down_offset = dest_alt - current_alt;
				dest_alt = dest_alt + (throttle * 3.0);
			}
			else
			{
				dest_alt = current_alt + down_offset + (throttle * 3.0);
			}
			up = false;
		}
	}
	//Double_PID_Calculation(&db_thr, dest_alt, current_alt, imuex.vz, remain_dt);
	Double_PID_Calculation(&db_thr, dest_alt, current_alt, Alt_getV(), remain_dt);

	//悬停油门
	float hover_thr = cfg.param.alt_hover_thr;

    // 3. 重力补偿系数 (因为倾斜后升力垂直分量减小)
    //float grav_comp = get_gravity_compensation(imuex.roll, imuex.pitch);
	float grav_comp = get_gravity_compensation(roll, pitch);
	//PERIODIC_LOGI(1000, "grav_comp:%f", grav_comp);
    //为了调试，暂时禁用
    //grav_comp = 1.0;

	//加速度计算（加速度在运动的时候不准，而且az会受到倾斜的影响）
	//float thr_acc = control(&PIDAccAlt, imuex.az, 1.0, remain_dt);
	//thr_acc = constrain(thr_acc, -0.2f, 0.2f);

	//起飞前不对油门进行调整
	if (!(thrIdle && dshot_throttle == 0))
	{
		//dshot_throttle = hover_thr + thr_pid/* + thr_acc*/;

		// 4. 最终油门指令
		//    基础悬停油门 * 重力补偿 + 加速度修正 (将m/s^2转换为油门变化量，系数0.1需要根据实际机型调校)
		const float ACCEL_TO_THROTTLE = 0.1f;  // 1 m/s^2 对应约 0.1 油门变化 (需实验确定)
		float thr_pid = db_thr.in.pid_result * ACCEL_TO_THROTTLE;

		//如果气压计失效，就降落
		if (app.CheckBmpTimeout())
		{
			PERIODIC_LOGI(500, "bmp halt");
			thr_pid = hover_thr - 0.2f;
		}

		//对收油进行限制，避免螺旋桨停止
		thr_pid = constrain(thr_pid, -0.2f, 0.2f);

		//油门一阶低通滤波
		thr_lpf.alpha = cfg.param.alt_thr_lp;
		float thr_offset = lpf_update(&thr_lpf, thr_pid);

		dshot_throttle = (hover_thr + thr_offset) * grav_comp;

		//对最小油门进行限制，避免直接空中停机
		dshot_throttle = constrain(dshot_throttle, hover_thr - 0.1f, 1.0f);
	}
	dshot_throttle = constrain(dshot_throttle, 0.0f, 1.0f);

	static uint32_t prev_tick = 0;
	uint32_t tick = HAL_GetTick();
	if ((tick - prev_tick) > 1000)
	{
		prev_tick = tick;
		Log::I("thr:%f,roll:%f,pitch:%f,yaw:%f", dshot_throttle, PIDroll.pid, PIDpitch.pid, PIDyaw.pid);
		Log::I("dest_alt:%f, current_alt:%f", dest_alt, current_alt);
	}

    //Actuator mixing
    out_Mixer(_CrsfThrottle, dshot_throttle, motor_speeds); //Mixes PID outputs and sends command pulses to the motors, if mot.arm == true
#endif
}


void Controller::climb()
{
	int _CrsfRoll = 1500;
	int _CrsfPitch = 1500;
	int _CrsfThrottle = 1500;
	int _CrsfYaw = 0;
	int _CrsfMode = 0;
	int _CrsfLight = 0;
	int _CrsfReturnHome = 0;
	int _CrsfArmed = 0;

	CRSFRadio.GetData(&_CrsfRoll, &_CrsfPitch, &_CrsfThrottle, &_CrsfYaw, &_CrsfMode, &_CrsfLight, &_CrsfReturnHome, &_CrsfArmed);

	bool thrIdle = false;
	//检查油门是否居中
	if (_CrsfThrottle > (cfg.param.rcl_thr_mid - cfg.param.rcl_thr_dead) && _CrsfThrottle < (cfg.param.rcl_thr_mid + cfg.param.rcl_thr_dead))
	{
		thrIdle = true;
	}
	(void)thrIdle;
	bool rolIdle = false;
	//检查roll是否居中
	if (_CrsfRoll > (cfg.param.rcl_rol_mid - cfg.param.rcl_rol_dead) && _CrsfRoll < (cfg.param.rcl_rol_mid + cfg.param.rcl_rol_dead))
	{
		rolIdle = true;
	}
	bool pitIdle = false;
	if (_CrsfPitch > (cfg.param.rcl_pit_mid - (cfg.param.rcl_pit_dead + /*遥控器校准*/10)) && _CrsfPitch < (cfg.param.rcl_pit_mid + (cfg.param.rcl_pit_dead + /*遥控器校准*/10)))
	//检查pitch是否居中
	{
		pitIdle = true;
	}

	static float max_thr = 0; //最大油门
	static float min_thr = 0; //最小油门
	static float thr = 0.0;

	//检查飞机状态是否异常
	bool fail = false;
    if (!app.IsArmed())
    {
    	fail = true;
    }

    if (fail)
    {
		motor_speeds[0] = 1000;
		motor_speeds[1] = 1000;
		motor_speeds[2] = 1000;
		motor_speeds[3] = 1000;

		max_thr = 0;
		min_thr = 0;
		thr = 0.0;

		return;
    }


	float throttle = ChannelNormalize(1, _CrsfThrottle, cfg.param.rcl_thr_pull, cfg.param.rcl_thr_mid, cfg.param.rcl_thr_push, cfg.param.rcl_thr_dead);

	if (!thrIdle)
	{
		if (throttle > 0 && throttle > thr)
		{
		    thr     = throttle;
		    max_thr = thr;      // 不变量：加油门后 max_thr 恒等于 thr
		    min_thr = 0;
		}
		else if (throttle < 0 && throttle < min_thr)
		{
		    min_thr = throttle;
		    thr     = max_thr + min_thr;
		}
	}

	motor_speeds[MOTOR_FL] = thr;
	motor_speeds[MOTOR_FR] = thr;
	motor_speeds[MOTOR_BL] = thr;
	motor_speeds[MOTOR_BR] = thr;

	for (int i = 0; i < 4; i++) {
		// 转速限幅
		if (motor_speeds[i] < 0.02)
		{
			motor_speeds[i] = 0.02;
		}
	}

}

