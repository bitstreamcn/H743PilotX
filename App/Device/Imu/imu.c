#include "imu.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vqf/vqf.h"
#include <math.h>
#include "main.h"
#include "common.h"
#include "log.h"
#include "Config.h"

#define M_SQRT2     1.41421356237309504880   // sqrt(2)s

static const float rad_to_deg = 57.2957795132f;
static const float deg_to_rad = 0.0174532925199f;

float q[4];

float roll;               // roll in degrees: -180 to 180, roll right is positive
float pitch;              // pitch in degrees: -90 to 90, pitch up is positive
float yaw;                // yaw in degrees: -180 to 180, yaw right is positive
float yaw_imu_raw = 0;
float yaw_offset = 0;

float imu_cal_ax = 0;
float imu_cal_ay = 0;
float imu_cal_az = 0;
float imu_cal_gx = 0;
float imu_cal_gy = 0;
float imu_cal_gz = 0;

float B_gyr;
float B_acc;
float B_mag;

const float imu_gyr_lp = 60;
const float imu_acc_lp = 70;
const float mag_lp = 1e10;

const float acc_resolution = 32768;

float acc_multiplier = 0;
float gyro_multiplier = 0;

//Available excecution methods (not all platforms support all methods)
#define IMU_EXEC_IRQ 1            //execute in IRQ context on first core (works on STM32. Does NOT work on ESP32, RP2040)
#define IMU_EXEC_FREERTOS 2       //execute as IRQ triggered high priority FreeRTOS task on same core as setup() (works on ESP32, RP2040)
#define IMU_EXEC_FREERTOS_OTHERCORE 3 //execute as IRQ triggered high priority FreeRTOS task on second core (works on RP2040)

//default settings
#ifndef IMU_GYRO_DPS
  #define IMU_GYRO_DPS 2000 //Full scale gyro range in deg/sec. Most IMUs support 250,500,1000,2000. Can use any value here, driver will pick next greater setting.
#endif
#ifndef IMU_ACCEL_G
  #define IMU_ACCEL_G 4 //Full scale accelerometer range in G's. Most IMUs support 2,4,8,16. Can use any value here, driver will pick next greater setting.
#endif

//#define GRAVITY (16384)       // ±2g量程下，1g对应的ADC值（16位，2g=32768→1g=16384）

#define IMU_EXEC IMU_EXEC_IRQ

//global module class instance
Imu imu = DEFAULT_Imu;

volatile uint8_t imu_data_irq = 0;


float B_gyr;            // gyr filter constant
float B_acc;            // acc filter constant
float B_mag;            // mag filter constant


void _imu_ll_interrupt_setup(int interrupt_pin); //prototype
volatile bool _imu_ll_interrupt_enabled = false;
volatile bool _imu_ll_interrupt_busy = false;
volatile uint32_t _imu_ll_interrupt_ts = 0;

bool imu_usesI2C() { return true; } //returns true if IMU uses I2C bus (not SPI bus)
bool imu_hasMag() { return false; }

uint32_t imu_getSampleRate() {return imu._sampleRate;}
bool imu_installed() {return true;/*(imu.gizmo != NULL); */}

void Ahr_computeAngles();


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

//lowpass frequency to filter beta constant
static float lowpass_to_beta(float f0, float fs) {
  return constrain(1 - exp(-2 * M_PI * f0 / fs), 0.0f, 1.0f);
}


void AhrGizmoVqf_Init() {
  initVqf(1.0/imu_getSampleRate(), 1.0/imu_getSampleRate(), 1.0/imu_getSampleRate());
}


static void quatMultiply(const vqf_real_t q1[4], const vqf_real_t q2[4], vqf_real_t out[4])
{
    vqf_real_t w = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    vqf_real_t x = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    vqf_real_t y = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    vqf_real_t z = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

bool AhrGizmoVqf_update() {
  //convert to ENU from NED: xE = yE, yN = xE, zU = -zD
  float gyr[3] = {imu.state.gy * deg_to_rad,  imu.state.gx * deg_to_rad, -imu.state.gz * deg_to_rad}; //gyr in rad/sec
  float acc[3] = {imu.state.ay * 9.81f,            imu.state.ax * 9.81f,           -imu.state.az * 9.81f};           //acc in m/s/s -  acc[] horizontal: [0, 0, -9.81], nose down:[0, 9.81, 0], right down: [9.81, 0, 0]
  float mag[3] = {imu.state.mx,                   -imu.state.my,                    imu.state.mz};                   //mag no unit - mag[] horizontal pointing north: [15, 0, 40], east: [0,15,40]

  //get fused q_enu in ENU frame
  float q_enu[4];
  if (imu.state.mx==0 && imu.state.my==0 && imu.state.mz==0) {
    updateGyr(gyr);
    updateAcc(acc);
    getQuat6D(q_enu);
  }else{
    updateGyr(gyr);
    updateAcc(acc);
    updateMag(mag);
    getQuat9D(q_enu);
  }

  //convert q_enu to NED reference frame
  float q_trans[4] = {0,M_SQRT2/2,M_SQRT2/2,0}; //ENU to NED
  quatMultiply(q_enu, q_trans, q);

  return true;
}



bool imu_init()
{
	/*
	  // IMU - Intertial Measurement Unit (gyro/acc/mag)
	  imu.config.sampleRate = cfg.imu_rate; //sample rate [Hz]
	  imu.config.pin_int = cfg.pin_imu_int; //IMU data ready interrupt pin
	  imu.config.gizmo = (Cfg::imu_gizmo_enum)cfg.imu_gizmo; //the gizmo to use
	  imu.config.spi_bus = hal_get_spi_bus(cfg.imu_spi_bus); //SPI bus
	  imu.config.spi_cs = cfg.pin_imu_cs; //SPI select pin
	  imu.config.i2c_bus = hal_get_i2c_bus(cfg.imu_i2c_bus); //I2C bus (only used if spi_bus==nullptr)
	  imu.config.i2c_adr = cfg.imu_i2c_adr; //i2c address. 0=default address
	  imu.config.uses_i2c = ((Cfg::imu_bus_type_enum)cfg.imu_bus_type == Cfg::imu_bus_type_enum::mf_I2C);
	  */
	  imu.config.sampleRate = 500; //sample rate [Hz]
	  imu.config.pin_int = 0; //IMU data ready interrupt pin
	  imu.config.gizmo = 2; //the gizmo to use
	  imu.config.spi_bus = NULL; //SPI bus
	  imu.config.spi_cs = -1; //SPI select pin
	  imu.config.i2c_bus = NULL; //I2C bus (only used if spi_bus==nullptr)
	  imu.config.i2c_adr = 0; //i2c address. 0=default address
	  imu.config.uses_i2c = false;

	  // Some sensors need a couple of tries...
	  int tries = 10;
	  bool imu_check = false;
	  while(true) {
	    int rv = imu_setup(); //request 1000 Hz sample rate, returns 0 on success, positive on error, negative on warning
	    if(rv<=0)
	    {
	    	imu_check = true;
	    	break;
	    }
	    tries--;
	    HAL_Delay(100);
	    //madflight_warn_or_die("IMU init failed rv= " + String(rv) + ".", (tries <= 0) );
	  }
	  if(!imu_installed()/* && (Cfg::imu_gizmo_enum)cfg.imu_gizmo != Cfg::imu_gizmo_enum::mf_NONE*/) {
	    //madflight_die("IMU install failed.");
	  }
	  // start IMU update handler
	  if(imu_installed()) {
		  //Ahr_setInitalOrientation(); //do this before IMU update handler is started

	    //if(!imu_loop) madflight_warn("'void imu_loop()' not defined.");
	    //imu.onUpdate = imu_loop;
	    //if(!imu.waitNewSample()) madflight_die("IMU interrupt not firing. Is pin 'pin_imu_int' connected?");

	    #ifndef MF_DEBUG
	      //Calibrate for zero gyro readings, assuming vehicle not moving when powered up. Comment out to only use cfg values. (Use CLI to calibrate acc.)
	      //cli.calibrate_gyro();
	    #endif
	  }

	imu_cal_ax = cfg.param.imu_cal_ax;
	imu_cal_ay = cfg.param.imu_cal_ay;
	imu_cal_az = cfg.param.imu_cal_az;
	imu_cal_gx = cfg.param.imu_cal_gx;
	imu_cal_gy = cfg.param.imu_cal_gy;
	imu_cal_gz = cfg.param.imu_cal_gz;

	  if (imu_check)
	  {
		  if (cfg.param.imu_docal > 0 || isnan(imu_cal_ax) || isnan(imu_cal_ay) || isnan(imu_cal_az)
				  || isnan(imu_cal_gx) || isnan(imu_cal_gy) || isnan(imu_cal_gz))
		  {
			  //校准陀螺仪
				float ax = 0, ay = 0, az = 0;
				float gx = 0, gy = 0, gz = 0;
				unsigned long t0;
				t0 = millis();
				unsigned long count = 0;

				/* Stop after 3 seconds */
				//把陀螺仪（飞控）平放，读取约3秒，最后计算平均值
				while(millis() - t0 < 3000)
				{
					float _ax=0, _ay=0, _az=1, _gx=0, _gy=0, _gz=0;

					//get sensor data and update timestamps, count
#if 0
					if(false) {
					  MPUXXXX_getMotion9NED(&_ax, &_ay, &_az, &_gx, &_gy, &_gz, &_mx, &_my, &_mz);
					}else{
					  MPUXXXX_getMotion6NED(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);
					}
#endif
					signed short acc_data_buf[3];
					signed short gyr_data_buf[3];
					if (SL_ICM_42688P_RawData_Read(acc_data_buf, gyr_data_buf) != 0)
					{
						continue;
					}
					_ax = -acc_data_buf[1] * acc_multiplier;
					_ay = -acc_data_buf[0] * acc_multiplier;
					_az = acc_data_buf[2] * acc_multiplier;
					_gx = gyr_data_buf[1] * gyro_multiplier;
					_gy = gyr_data_buf[0] * gyro_multiplier;
					_gz = -gyr_data_buf[2] * gyro_multiplier;

					ax += _ax;
					ay += _ay;
					az += _az;
					gx += _gx;
					gy += _gy;
					gz += _gz;
					count++;

					HAL_Delay(1000 / imu.config.sampleRate);
				}
				ax /= count;
				ay /= count;
				az /= count;
				gx /= count;
				gy /= count;
				gz /= count;

				imu_cal_ax = ax;
				imu_cal_ay = ay;
				imu_cal_az = az - 1.0f;//az - GRAVITY;
				imu_cal_gx = gx;
				imu_cal_gy = gy;
				imu_cal_gz = gz;

				cfg.param.imu_cal_ax = imu_cal_ax;
				cfg.param.imu_cal_ay = imu_cal_ay;
				cfg.param.imu_cal_az = imu_cal_az;
				cfg.param.imu_cal_gx = imu_cal_gx;
				cfg.param.imu_cal_gy = imu_cal_gy;
				cfg.param.imu_cal_gz = imu_cal_gz;

				cfg.param.imu_docal = 0;
				cfg_write_to_eeprom();
		  }
	  }

	  if (isnan(imu_cal_ax))
	  {
		  imu_cal_ax = 0;
	  }
	  if (isnan(imu_cal_ay))
	  {
		  imu_cal_ay = 0;
	  }
	  if (isnan(imu_cal_az))
	  {
		  imu_cal_az = 0;
	  }
	  if (isnan(imu_cal_gx))
	  {
		  imu_cal_gx = 0;
	  }
	  if (isnan(imu_cal_gy))
	  {
		  imu_cal_gy = 0;
	  }
	  if (isnan(imu_cal_gz))
	  {
		  imu_cal_gz = 0;
	  }


	B_gyr = lowpass_to_beta(imu_gyr_lp, imu_getSampleRate());//gyro low pass filter freq [Hz]
	B_acc = lowpass_to_beta(imu_acc_lp, imu_getSampleRate());//accelerometer low pass filter freq [Hz]
	B_mag = lowpass_to_beta(mag_lp, imu_getSampleRate());//magnetometer low pass filter freq [Hz]


    AhrGizmoVqf_Init();

	return imu_check;
}

//returns 0 on success, positive on error, negative on warning
int imu_setup() {

  //disable interrupt handler
  _imu_ll_interrupt_enabled = false;

  //exit if gizmo == NONE
  //if(imu.config.gizmo == 0) return 0;

  //==============================================================
  //create gizmo
  //==============================================================
  //if(!imu.config.uses_i2c && imu.config.spi_bus && imu.config.spi_cs >= 0)
  {
    //-----------
    //SPI Sensors
    //-----------
	 /*
    switch(config.gizmo) {
      case Cfg::imu_gizmo_enum::mf_NONE : {
        //do nothing
        break;
      }
      case Cfg::imu_gizmo_enum::mf_MPU9250 : {
        auto mpu_iface = new MPU_InterfaceSPI(config.spi_bus, config.spi_cs);
        gizmo = new MPUXXXX(MPUXXXX::MPU9250, mpu_iface);
        gizmo->uses_i2c = false;
        gizmo->has_mag = true;
        break;
      }
      case Cfg::imu_gizmo_enum::mf_MPU6500 : {
      */
        //auto mpu_iface = new MPU_InterfaceSPI(config.spi_bus, config.spi_cs);
        //gizmo = new MPUXXXX(MPUXXXX::MPU6500, mpu_iface);
        //gizmo->uses_i2c = false;
        //gizmo->has_mag = false;
	  //6500
#if 0
	  if (imu.config.gizmo == 2)
	  {
		  imu.mpuxxxx.gizmo.has_mag = false;
	  }
	  //9250
	  if (imu.config.gizmo == 1)
	  {
		  imu.mpuxxxx.gizmo.has_mag = true;
	  }
#endif
       /*
        break;
      }
      case Cfg::imu_gizmo_enum::mf_MPU6000 : {
        auto mpu_iface = new MPU_InterfaceSPI(config.spi_bus, config.spi_cs);
        gizmo = new MPUXXXX(MPUXXXX::MPU6000, mpu_iface);
        gizmo->uses_i2c = false;
        gizmo->has_mag = false;
        break;
      }
      case Cfg::imu_gizmo_enum::mf_BMI270 : {
        gizmo = new ImuGizmoBMI270(config.spi_bus, config.spi_cs);
        gizmo->uses_i2c = false;
        gizmo->has_mag = false;
        break;
      }
      case Cfg::imu_gizmo_enum::mf_ICM45686 : {
        auto icm_iface = new Invensensev3_InterfaceSPI(config.spi_bus, config.spi_cs);
        gizmo = new ImuGizmoICM45686( (uint8_t)config.pin_int, icm_iface );
        gizmo->uses_i2c = false;
        gizmo->has_mag = false;
        break;
      }
      case Cfg::imu_gizmo_enum::mf_ICM42688 : {
        gizmo = ImuGizmoICM426XX::create(&config, (ImuState*)this);
        break;
      }
      default: {
        Serial.println("\n" MF_MOD ": ERROR - Sensor does not support imu_bus_type=SPI\n");
      }
    }*/
  }
  /*
  else if(config.uses_i2c && config.i2c_bus) {
    //-----------
    //I2C Sensors
    //-----------
    switch(config.gizmo) {
      case Cfg::imu_gizmo_enum::mf_NONE : {
        //do nothing
        break;
      }
      case Cfg::imu_gizmo_enum::mf_MPU9250 : {
        auto mpu_iface = new MPU_InterfaceI2C(config.i2c_bus, config.i2c_adr);
        gizmo = new MPUXXXX(MPUXXXX::MPU9250, mpu_iface);
        gizmo->uses_i2c = true;
        gizmo->has_mag = true;
        break;
      }
      case Cfg::imu_gizmo_enum::mf_MPU9150 : {
        auto mpu_iface = new MPU_InterfaceI2C(config.i2c_bus, config.i2c_adr);
        gizmo = new MPUXXXX(MPUXXXX::MPU9150, mpu_iface);
        gizmo->uses_i2c = true;
        gizmo->has_mag = true;
        break;
      }
      case Cfg::imu_gizmo_enum::mf_MPU6500 : {
        auto mpu_iface = new MPU_InterfaceI2C(config.i2c_bus, config.i2c_adr);
        gizmo = new MPUXXXX(MPUXXXX::MPU6500, mpu_iface);
        gizmo->uses_i2c = true;
        gizmo->has_mag = false;
        break;
      }
      case Cfg::imu_gizmo_enum::mf_MPU6050 : {
        auto mpu_iface = new MPU_InterfaceI2C(config.i2c_bus, config.i2c_adr);
        gizmo = new MPUXXXX(MPUXXXX::MPU6050, mpu_iface);
        gizmo->uses_i2c = true;
        gizmo->has_mag = false;
        break;
      }
      case Cfg::imu_gizmo_enum::mf_MPU6000 : {
        auto mpu_iface = new MPU_InterfaceI2C(config.i2c_bus, config.i2c_adr);
        gizmo = new MPUXXXX(MPUXXXX::MPU6000, mpu_iface);
        gizmo->uses_i2c = true;
        gizmo->has_mag = false;
        break;
      }
      case Cfg::imu_gizmo_enum::mf_ICM42688 : {
        gizmo = ImuGizmoICM426XX::create(&config, (ImuState*)this);
        break;
      }
      default: {
        Serial.println("\n" MF_MOD ": ERROR - Sensor does not support imu_bus_type=I2C\n");
      }
    }
  }
	*/
  //check gizmo
  /*
  if(!gizmo && config.gizmo != Cfg::imu_gizmo_enum::mf_NONE) {
    Serial.println("\n" MF_MOD ": ERROR - Check pin/bus/bus_type config\n");
    return -1001;
  }

  //exit if no gizmo installed
  if(!gizmo) return 0;
  */
  //exit if no interrupt pin specified
  //if(config.pin_int < 0) return 0;

  //==============================================================
  //setup low-level interrupt stuff
  //==============================================================
  //_imu_ll_interrupt_enabled = false; //already done
  //int rv = MPUXXXX_begin(IMU_GYRO_DPS, IMU_ACCEL_G, imu.config.sampleRate);

  int rv = SL_ICM_42688P_Config() ? 0 : 1;
  //+- 4G
  if(IMU_ACCEL_G <= 2) {
    acc_multiplier = 2.0 / acc_resolution;
  }else if(IMU_ACCEL_G <= 4) {
    acc_multiplier = 4.0 / acc_resolution;
  }else if(IMU_ACCEL_G <= 8) {
    acc_multiplier = 8.0 / acc_resolution;
  }else{
    acc_multiplier = 16.0 / acc_resolution;
  }
  if(IMU_GYRO_DPS <= 250) {
    gyro_multiplier = 250.0 / 32768.0;
  }else if(IMU_GYRO_DPS <= 500) {
    gyro_multiplier = 500.0 / 32768.0;
  }else if(IMU_GYRO_DPS <= 1000) {
    gyro_multiplier = 1000.0 / 32768.0;
  }else{
    gyro_multiplier = 2000.0 / 32768.0;
  }

  imu._sampleRate = imu.config.sampleRate;
  //Serial.printf(MF_MOD ": Actual sample rate:%d Hz\n", (int)_sampleRate);
  imu.onUpdate = NULL;
  _imu_ll_interrupt_busy = false;
  _imu_ll_interrupt_ts = 0;
  imu.overrun_cnt = 0;
  imu.state.dt = 0;
  imu.state.ts = micros();
  imu_statReset();
  _imu_ll_interrupt_setup(imu.config.pin_int);
  _imu_ll_interrupt_enabled = true;
  imu.interrupt_cnt = 0;
  imu.update_cnt = 0;
  return rv;
}

bool imu_update() {
#if 0
	//检查是否收到更新中断
	if (imu_data_irq == 0)
	{
		return false;
	}
	if (imu_data_irq > 1)
	{
		LogW("imu_update slow: %d", imu_data_irq);
	}
	imu_data_irq = 0;
#endif
  //if(!gizmo) return false;

  //start of update timestamp
  uint32_t update_ts = micros();

  float ax=0, ay=0, az=1, gx=0, gy=0, gz=0;

  //get sensor data and update timestamps, count
#if 0
  if(imu.mpuxxxx.gizmo.has_mag) {
	  MPUXXXX_getMotion9NED(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  }else{
	  MPUXXXX_getMotion6NED(&ax, &ay, &az, &gx, &gy, &gz);
  }
#endif
	signed short acc_data_buf[3];
	signed short gyr_data_buf[3];
	if (SL_ICM_42688P_RawData_Read(acc_data_buf, gyr_data_buf) != 0)
	{
		//HAL_Delay(2);
		//HAL_SPI_MspDeInit(&hspi4);
		//HAL_SPI_MspInit(&hspi4);
		//MX_SPI4_Init();
		imu_init();
		//HAL_Delay(1000);
		return false;
	}
	ax = -acc_data_buf[1] * acc_multiplier;
	ay = -acc_data_buf[0] * acc_multiplier;
	az = acc_data_buf[2] * acc_multiplier;
	gx = gyr_data_buf[1] * gyro_multiplier;
	gy = gyr_data_buf[0] * gyro_multiplier;
	gz = -gyr_data_buf[2] * gyro_multiplier;

  // get sensor data from imu and mag
   // correct the sensor data with the calibration values
   // use simple first-order low-pass filter to get rid of high frequency noise
   // store filtered data in ax ay az gx gy gz mx my mz
   // call the sensor fusion algorithm to update q
   // compute euler angles from q

   //Low-pass filtered, corrected accelerometer data
  imu.state.ax += B_acc * ((ax - imu_cal_ax) - imu.state.ax);
  imu.state.ay += B_acc * ((ay - imu_cal_ay) - imu.state.ay);
  imu.state.az += B_acc * ((az - imu_cal_az) - imu.state.az);

   //Low-pass filtered, corrected gyro data
  imu.state.gx += B_gyr * ((gx - imu_cal_gx) - imu.state.gx);
  imu.state.gy += B_gyr * ((gy - imu_cal_gy) - imu.state.gy);
  imu.state.gz += B_gyr * ((gz - imu_cal_gz) - imu.state.gz);


  //Magnetometer (External chip, or internal in IMU chip)
  //float _mx = mx, _my = my, _mz = mz;
  //If no external mag, then use internal mag
  /*
  if(!ahr.config.pmag || (ahr.config.pmag->x == 0 && ahr.config.pmag->y == 0 && ahr.config.pmag->z == 0))
  {
    _mx = ahr.config.pimu->state.mx;
    _my = ahr.config.pimu->state.my;
    _mz = ahr.config.pimu->state.mz;
  }
  else
  {
    _mx = ahr.config.pmag->x;
    _my = ahr.config.pmag->y;
    _mz = ahr.config.pmag->z;
  }
  //*/
#if 0
  //update the mag values
  if( ! (_mx == 0 && _my == 0 && _mz == 0) ) {
    //Correct the mag values with the calibration values
    _mx = (_mx - cfg.param.mag_cal_x) * cfg.param.mag_cal_sx;
    _my = (_my - cfg.param.mag_cal_y) * cfg.param.mag_cal_sy;
    _mz = (_mz - cfg.param.mag_cal_z) * cfg.param.mag_cal_sz;
    //Low-pass filtered magnetometer data
    imu.state.mx += B_mag * (_mx - imu.state.mx);
    imu.state.my += B_mag * (_my - imu.state.my);
    imu.state.mz += B_mag * (_mz - imu.state.mz);
  }else{
	  imu.state.mx = 0;
	  imu.state.my = 0;
	  imu.state.mz = 0;
  }
#endif
  //handle rotation for different mounting positions
  /*
  switch((Cfg::imu_align_enum)cfg.imu_align) {
    case Cfg::imu_align_enum::mf_CW0 :
      break;
    case Cfg::imu_align_enum::mf_CW90 :
      { float tmp; tmp=ax; ax=-ay; ay=tmp;   tmp=gx; gx=-gy; gy=tmp;   tmp=mx; mx=-my; my=tmp; }
      break;
    case Cfg::imu_align_enum::mf_CW180 :
      { ax=-ax; ay=-ay;   gx=-gx; gy=-gy;   mx=-mx; my=-my; }
      break;
    case Cfg::imu_align_enum::mf_CW270 :
      { float tmp; tmp=ax; ax=ay; ay=-tmp;   tmp=gx; gx=gy; gy=-tmp;   tmp=mx; mx=my; my=-tmp; }
      break;
    case Cfg::imu_align_enum::mf_CW0FLIP :
      { ay=-ay; az=-az;   gy=-gy; gz=-gz;   my=-my; mz=-mz; }
      break;
    case Cfg::imu_align_enum::mf_CW90FLIP :
      { float tmp; tmp=ax; ax=ay; ay=tmp; az=-az;   tmp=gx; gx=gy; gy=tmp; gz=-gz;   tmp=mx; mx=my; my=tmp; mz=-mz; }
      break;
    case Cfg::imu_align_enum::mf_CW180FLIP :
      { ax=-ax; az=-az;   gx=-gx; gz=-gz;   mx=-mx; mz=-mz; }
      break;
    case Cfg::imu_align_enum::mf_CW270FLIP :
      { float tmp; tmp=ax; ax=-ay; ay=-tmp; az=-az;   tmp=gx; gx=-gy; gy=-tmp; gz=-gz;   tmp=mx; mx=-my; my=-tmp; mz=-mz; }
      break;
  }
*/
  //this->dt = (update_ts - this->ts) / 1000000.0;
  //this->ts = update_ts;
  //this->update_cnt++;
  imu.state.dt = (update_ts - imu.state.ts) / 1000000.0;
  imu.state.ts = update_ts;
  imu.update_cnt++;

  //call gizmo to update q
  AhrGizmoVqf_update();

  //update euler angles
  Ahr_computeAngles();




  return true; //FIXME: should only return true if new sample was retrieved
}

//wait for new sample, returns false on fail
bool imu_waitNewSample() {
  uint32_t last_cnt = imu.update_cnt;
  uint32_t start = millis();
  while( last_cnt == imu.update_cnt && millis() - start <= (10*1000) / imu._sampleRate );
  return (last_cnt != imu.update_cnt);
}

void imu_statReset() {
	imu.stat_cnt = 0; //number of accumulated samples
	imu.stat_latency = 0; //summed interrupt latency from start of interrupt handler to start of interrupt task in us
	imu.stat_io_runtime = 0; //summed runtime of SPI/I2C io transfer in us
	imu.stat_runtime = 0; //summed runtime imu update including io transfer in us
	imu.stat_runtime_max = 0; //max runtime imu update including transfer in us, since last reset to 0
	imu.stat_reset_ts = micros(); //last time statReset() was called
}


//this function executes whenever the imu task is triggered
void imu__interrupt_handler() {
  //latency between start of low level interrupt handler and start of this method
	imu.stat_latency += micros() - _imu_ll_interrupt_ts;

  //start of task timestamp
  uint32_t task_ts = micros();

  //update sensor data
  imu_update();

  imu.stat_io_runtime += micros() - task_ts; //runtime of SPI/I2C transfer

  //call external update event handler
  if(imu.onUpdate) imu.onUpdate();

  uint32_t rt = micros() - task_ts; //runtime of full update including external event handler
  imu.stat_runtime += rt;
  if(imu.stat_runtime_max < rt) imu.stat_runtime_max = rt; //max runtime
  imu.stat_cnt++;
}

//========================================================================================================================//
// _IMU_LL_ IMU Low Level Interrrupt Handler
//========================================================================================================================//
// This runs the IMU updates triggered from pin HW_PIN_IMU_EXTI interrupt. When using FreeRTOS with IMU_EXEC_FREERTOS the
// IMU update is not executed directly in the interrupt handler, but a high priority task is used. This prevents FreeRTOS
// watchdog resets. The delay (latency) from rising edge INT pin to handler is approx. 10 us on ESP32 and 50 us on RP2040.

void _imu_ll_interrupt_handler();

//-------------------------------------------------------------------------------------------------------------------------
#if IMU_EXEC == IMU_EXEC_FREERTOS || IMU_EXEC == IMU_EXEC_FREERTOS_OTHERCORE

  TaskHandle_t _imu_ll_task_handle = NULL;

  void _imu_ll_task(void*) {
    for(;;) {
      ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
      imu._interrupt_handler();
    }
  }

  void _imu_ll_interrupt_setup(int interrupt_pin) {
    if(!_imu_ll_task_handle) {
      //
      #if IMU_EXEC == IMU_EXEC_FREERTOS_OTHERCORE
        int callcore = hal_get_core_num();
        int othercore = (callcore+1)%2;

        //TODO move this to hal
        #if defined ARDUINO_ARCH_ESP32
          //note: probably don't what to use this because of single FPU context switching issues...
          xTaskCreatePinnedToCore(_imu_ll_task, "IMU", MF_FREERTOS_DEFAULT_STACK_SIZE, NULL, IMU_FREERTOS_TASK_PRIORITY /*priority 0=lowest*/, &_imu_ll_task_handle, othercore); //[ESP32 only]
        #elif defined ARDUINO_ARCH_RP2040
          xTaskCreate(_imu_ll_task, "IMU", MF_FREERTOS_DEFAULT_STACK_SIZE, NULL, IMU_FREERTOS_TASK_PRIORITY /*priority 0=lowest*/, &_imu_ll_task_handle);
          vTaskCoreAffinitySet(_imu_ll_task_handle, (1<<othercore)); //[RP2040 only] Sets the core affinity mask for a task, i.e. the cores on which a task can run.
        #else
          #error "IMU_EXEC == IMU_EXEC_FREERTOS_OTHERCORE not supported on this processor"
        #endif

        Serial.printf(MF_MOD ": IMU_EXEC_FREERTOS_OTHERCORE call_core=%d imu_core=%d\n", callcore, othercore);
      #else
        xTaskCreate(_imu_ll_task, "IMU", MF_FREERTOS_DEFAULT_STACK_SIZE, NULL, IMU_FREERTOS_TASK_PRIORITY /*priority 0=lowest*/, &_imu_ll_task_handle);
        Serial.println(MF_MOD ": IMU_EXEC_FREERTOS");
      #endif
    }
    attachInterrupt(digitalPinToInterrupt(interrupt_pin), _imu_ll_interrupt_handler, RISING);
  }

  inline void _imu_ll_interrupt_handler2() {
    //let RTOS task _imu_ll_task handle the interrupt
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(_imu_ll_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
//-------------------------------------------------------------------------------------------------------------------------
#elif IMU_EXEC == IMU_EXEC_IRQ

  void _imu_ll_interrupt_setup(int interrupt_pin) {
    //Serial.println(MF_MOD ": IMU_EXEC_IRQ");
    //attachInterrupt(digitalPinToInterrupt(interrupt_pin), _imu_ll_interrupt_handler, RISING);
  }

  inline void _imu_ll_interrupt_handler2() {
      //call interrupt handler directly from interrupt context
	  imu__interrupt_handler();
  }
//-------------------------------------------------------------------------------------------------------------------------
#else
  #error #define IMU_EXEC is needed, see imu.h for available options
#endif

//main IRQ handler - timestamp + busy wrapper around _imu_ll_interrupt_handler2()
void _imu_ll_interrupt_handler() {
  _imu_ll_interrupt_ts = micros();
  if(_imu_ll_interrupt_enabled) {
    imu.interrupt_cnt = imu.interrupt_cnt + 1; //NOTE: "imu.interrupt_cnt++" gives warning: '++' expression of 'volatile'-qualified type is deprecated
    if (_imu_ll_interrupt_busy) { //note: time difference between check/update of _imu_ll_interrupt_busy can cause a race condition...
      imu.overrun_cnt++;
    } else {
      _imu_ll_interrupt_busy = true;
      _imu_ll_interrupt_handler2();
      _imu_ll_interrupt_busy = false;
    }
  }
}


//compute euler angles from q
void Ahr_computeAngles() {
	roll = atan2(q[0]*q[1] + q[2]*q[3], 0.5f - q[1]*q[1] - q[2]*q[2]) * rad_to_deg; //degrees - roll right is positive
	pitch = asin(constrain(-2.0f * (q[1]*q[3] - q[0]*q[2]), -1.0, 1.0)) * rad_to_deg; //degrees - pitch up is positive - use constrain() to prevent NaN due to rounding
	float _yaw = atan2(q[1]*q[2] + q[0]*q[3], 0.5f - q[2]*q[2] - q[3]*q[3]) * rad_to_deg + 180; //degrees - yaw right is positive  (0 - 359)
    // 保存 IMU 原始 yaw 供 set_mag 使用
    yaw_imu_raw = _yaw;
    // 叠加偏移量
    yaw = _yaw + yaw_offset;
    // 边界处理
    if (yaw < 0.0f)
    {
        yaw += 360.0f;
    }
    else if (yaw >= 360.0f)
    {
        yaw -= 360.0f;
    }
}

//世界坐标系下加速度北向
float Ahr_getAccelX()
{
	float w_ax = (1-2*q[2]*q[2]-2*q[3]*q[3])*imu.state.ax + 2*(q[1]*q[2]-q[0]*q[3])*imu.state.ay + 2*(q[1]*q[3]+q[0]*q[2])*imu.state.az;
	return 9.80665 * w_ax;
}
//世界坐标系统下加速度东向
float Ahr_getAccelY()
{
	float w_ay = 2*(q[1]*q[2]+q[0]*q[3])*imu.state.ax + (1-2*q[1]*q[1]-2*q[3]*q[3])*imu.state.ay + 2*(q[2]*q[3]-q[0]*q[1])*imu.state.az;
	return 9.80665 * w_ay;
}

// 获取机体坐标系下的前向纯运动加速度 (去除重力，单位 m/s^2)
float Ahr_getAccelBodyFront() {
    // g_proj_x 是重力 (1g) 在机体 X 轴 (前向) 上的投影分量 (单位为 g)
    // 该系数提取自四元数旋转矩阵的第三行第一列
    float g_proj_x = 2 * (q[1] * q[3] - q[0] * q[2]);

    // IMU 原始 X 轴数据 (前向，单位 g) 减去重力投影，再转为 m/s^2
    return (imu.state.ax - g_proj_x) * 9.80665f;
}

// 获取机体坐标系下的右向纯运动加速度 (去除重力，单位 m/s^2)
float Ahr_getAccelBodyRight() {
    // g_proj_y 是重力 (1g) 在机体 Y 轴 (右向) 上的投影分量 (单位为 g)
    // 该系数提取自四元数旋转矩阵的第三行第二列
    float g_proj_y = 2 * (q[2] * q[3] + q[0] * q[1]);

    // IMU 原始 Y 轴数据 (右向，单位 g) 减去重力投影，再转为 m/s^2
    return (imu.state.ay - g_proj_y) * 9.80665f;
}

//get acceleration in earth-frame up direction in [m/s^2]
float Ahr_getAccelUp() {
  return 9.80665 * ((2*q[1]*q[3] - 2*q[0]*q[2])*imu.state.ax + (2*q[2]*q[3] + 2*q[0]*q[1])*imu.state.ay + (q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])*imu.state.az - 1.0);
}

void set_mag(float mag)
{
    // 计算磁力计与 IMU 原始角度的差值
    float diff = mag - yaw_imu_raw;

    // 处理跨越 0/360 边界的情况
    // 例如: mag=10, yaw_imu_raw=350, diff=-340。应该等价于 +20。
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    // 低通滤波 (非常关键！磁力计噪声很大，直接用会导致角度慢慢偏移)
    // 0.05f 是融合系数，越小越平滑，但响应越慢。可根据需要调整 0.01~0.1
    yaw_offset = yaw_offset * 0.99f + diff * 0.01f;
}


