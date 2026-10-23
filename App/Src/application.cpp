/*
 * application.cpp
 *
 *  Created on: Apr 2, 2026
 *      Author: bitstream
 */
#include "main.h"
#include "application.h"
#include <cstdio>
#include "led.h"
#include "Device/Bmp/bmp5xx.h"
#include "log.h"
#include "CRSFRadio.h"
#include "common.h"
#include "Servo.h"
#include "tim.h"
#include "Config.h"
#include "uartx.h"
#include "i2c.h"
#include "imu_ex.h"
#include "Common/kalman.h"
#include "cppmain.h"
#include "bottom.h"
#include "file.h"
#include "../Device/Qmc5883l/qmc5883l.h"
#include "gps.h"
#include "mag_calibration.h"

#define SEALEVELPRESSURE_HPA (1013.25)

// For SPI mode, uncomment the next line and comment out the I2C begin() call in setup()
// #define BMP5XX_CS_PIN 10

Adafruit_BMP5xx bmp; // Create BMP5xx object
bmp5xx_powermode_t desiredMode = BMP5XX_POWERMODE_NORMAL; // Cache desired power mode


#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0f)

#ifdef __cplusplus
extern "C" {
#endif

#include "../Device/Imu/imu.h"
#include "../Device/Alt/alt.h"



#ifdef __cplusplus
}
#endif

extern volatile bool IsConnect;

Application app;

Kalman location_kalman;

typedef enum{
	LOOP_IDLE,
	LOOP_RADIO,
	LOOP_UART1,
	LOOP_BMP,
	LOOP_IMU,
	LOOP_LOG,
	LOOP_BOTTOM,
	LOOP_SDWRITE,
	LOOP_QMC5883L,
	LOOP_GPS,
	LOOP_FRONT,
	LOOP_CAR,
	LOOP_END
}LOOP_STATE;

Application::Application()
{

}

Application::~Application()
{

}

// 传入你的 I2C 句柄，例如 &hi2c1
void I2C_Scan(I2C_HandleTypeDef *hi2c)
{
    uint8_t found_count = 0;

    // I2C 7位地址范围是 0x01 到 0x7F (0x00是通用呼叫地址，通常跳过)
    for (uint8_t addr = 1; addr < 128; addr++)
    {
        // 记录当前Tick，用于超时判断
        uint32_t tick_start = HAL_GetTick();

        // 作用1：HAL_I2C_IsDeviceReady 会发送地址并等待ACK
        // 作用2：STM32 HAL库要求传入8位地址，所以要把7位地址左移1位 (addr << 1)
        HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 1, 10);

        if (status == HAL_OK)
        {
            // 收到应答，说明地址存在设备
            found_count++;
        }

        // 稍作延时，防止总线过于拥挤
        HAL_Delay(5);
    }
}


void Application::Init()
{
	Log::Init();

	//I2C_Scan(&hi2c3);

	location_kalman.init(0.000067, 0.02, 1.0);

	cfg_init();
	if (!cfg_load_from_eeprom())
	{
		cfg_clear();
		cfg_write_to_eeprom();
	}

	Log::I("Reset reason: 0x%lX\n", RCC->CSR);
	HAL_Delay(100);
	__HAL_RCC_CLEAR_RESET_FLAGS();

	Log::I("application init\r\n");

	//指南针校准
	if (cfg.param.sys_fly_mode == 2)
	{
		mag_cal();
	}

	Log::I("imu init");

	HAL_Delay(100);
#ifdef USE_IMUEX
	if (!imuex.Init() || !imu_init())
#else
	if (!imu_init())
#endif
	{
		Log::E("imu init fail");
		Log::E("\n");
		return;
	}


	//I2C_Scan(&hi2c4);
	QMC5883L_Init();

#ifndef USE_IMUEX
	Alt_Init();

	/*
	unsigned status;
	status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
	//status = bmp.begin();
	if (!status) {
		Log::E(F("Could not find a valid BMP280 sensor, check wiring or "
						  "try a different address!"));
		Log::E("SensorID was: 0x%x\r\n", bmp.sensorID());
		Log::E("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\r\n");
		Log::E("   ID of 0x56-0x58 represents a BMP 280,\r\n");
		Log::E("        ID of 0x60 represents a BME 280.\r\n");
		Log::E("        ID of 0x61 represents a BME 680.\r\n");
		while (1) delay(10);
	}
	//*/



	  // Try to initialize the sensor
	  // For I2C mode (default):
	  while (!bmp.begin(/*BMP5XX_ALTERNATIVE_ADDRESS, &Wire*/)) {
		// For SPI mode (uncomment the line below and comment out the I2C line above):
		// if (!bmp.begin(BMP5XX_CS_PIN, &SPI)) {
		 Log::I(F("Could not find a valid BMP5xx sensor, check wiring!"));
	     delay(1000);
	  }

	  Log::I(F("BMP5xx found!"));

	  delay(20);
	  // Demonstrate all setter functions with range documentation
	  Log::I(F("=== Setting Up Sensor Configuration ==="));

	  /* Temperature Oversampling Settings:
	   * BMP5XX_OVERSAMPLING_1X   - 1x oversampling (fastest, least accurate)
	   * BMP5XX_OVERSAMPLING_2X   - 2x oversampling
	   * BMP5XX_OVERSAMPLING_4X   - 4x oversampling
	   * BMP5XX_OVERSAMPLING_8X   - 8x oversampling
	   * BMP5XX_OVERSAMPLING_16X  - 16x oversampling
	   * BMP5XX_OVERSAMPLING_32X  - 32x oversampling
	   * BMP5XX_OVERSAMPLING_64X  - 64x oversampling
	   * BMP5XX_OVERSAMPLING_128X - 128x oversampling (slowest, most accurate)
	   */
	  delay(200);
	  Log::I(F("Setting temperature oversampling to 2X..."));
	  if (!bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X))
	  {
		  Log::E(F("Setting temperature oversampling error!"));
		  delay(100);
	  }

	  /* Pressure Oversampling Settings (same options as temperature):
	   * Higher oversampling = better accuracy but slower readings
	   * Recommended: 16X for good balance of speed/accuracy
	   */
	  delay(200);
	  Log::I(F("Setting pressure oversampling to 16X..."));
	  if (!bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X))
	  {
		  Log::E(F("Setting pressure oversampling error!"));
		  delay(100);
	  }

	  /* IIR Filter Coefficient Settings:
	   * BMP5XX_IIR_FILTER_BYPASS   - No filtering (fastest response)
	   * BMP5XX_IIR_FILTER_COEFF_1  - Light filtering
	   * BMP5XX_IIR_FILTER_COEFF_3  - Medium filtering
	   * BMP5XX_IIR_FILTER_COEFF_7  - More filtering
	   * BMP5XX_IIR_FILTER_COEFF_15 - Heavy filtering
	   * BMP5XX_IIR_FILTER_COEFF_31 - Very heavy filtering
	   * BMP5XX_IIR_FILTER_COEFF_63 - Maximum filtering
	   * BMP5XX_IIR_FILTER_COEFF_127- Maximum filtering (slowest response)
	   */
	  delay(200);
	  Log::I(F("Setting IIR filter to coefficient 3..."));
	  if (!bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3))
	  {
		  Log::E(F("Setting IIR filter error!"));
		  delay(100);
	  }

	  /* Output Data Rate Settings (Hz):
	   * BMP5XX_ODR_240_HZ, BMP5XX_ODR_218_5_HZ, BMP5XX_ODR_199_1_HZ
	   * BMP5XX_ODR_179_2_HZ, BMP5XX_ODR_160_HZ, BMP5XX_ODR_149_3_HZ
	   * BMP5XX_ODR_140_HZ, BMP5XX_ODR_129_8_HZ, BMP5XX_ODR_120_HZ
	   * BMP5XX_ODR_110_1_HZ, BMP5XX_ODR_100_2_HZ, BMP5XX_ODR_89_6_HZ
	   * BMP5XX_ODR_80_HZ, BMP5XX_ODR_70_HZ, BMP5XX_ODR_60_HZ, BMP5XX_ODR_50_HZ
	   * BMP5XX_ODR_45_HZ, BMP5XX_ODR_40_HZ, BMP5XX_ODR_35_HZ, BMP5XX_ODR_30_HZ
	   * BMP5XX_ODR_25_HZ, BMP5XX_ODR_20_HZ, BMP5XX_ODR_15_HZ, BMP5XX_ODR_10_HZ
	   * BMP5XX_ODR_05_HZ, BMP5XX_ODR_04_HZ, BMP5XX_ODR_03_HZ, BMP5XX_ODR_02_HZ
	   * BMP5XX_ODR_01_HZ, BMP5XX_ODR_0_5_HZ, BMP5XX_ODR_0_250_HZ, BMP5XX_ODR_0_125_HZ
	   */
	  delay(200);
	  Log::I(F("Setting output data rate to 50 Hz..."));
	  if (!bmp.setOutputDataRate(BMP5XX_ODR_50_HZ))
	  {
		  Log::E(F("Setting output data rate error!"));
		  delay(100);
	  }

	  /* Power Mode Settings:
	   * BMP5XX_POWERMODE_STANDBY     - Standby mode (no measurements)
	   * BMP5XX_POWERMODE_NORMAL      - Normal mode (periodic measurements)
	   * BMP5XX_POWERMODE_FORCED      - Forced mode (single measurement then standby)
	   * BMP5XX_POWERMODE_CONTINUOUS  - Continuous mode (fastest measurements)
	   * BMP5XX_POWERMODE_DEEP_STANDBY - Deep standby (lowest power)
	   */
	  delay(200);
	  Log::I(F("Setting power mode to normal..."));
	  desiredMode = BMP5XX_POWERMODE_NORMAL;
	  if (!bmp.setPowerMode(desiredMode))
	  {
		  Log::E(F("Setting power mode error!"));
		  delay(100);
	  }

	  /* Enable/Disable Pressure Measurement:
	   * true  - Enable pressure measurement (default)
	   * false - Disable pressure measurement (temperature only)
	   */
	  delay(200);
	  Log::I(F("Enabling pressure measurement..."));
	  if (!bmp.enablePressure(true))
	  {
		  Log::E(F("Enabling pressure measurement error!"));
		  delay(100);
	  }

	  /* Interrupt Configuration:
	   * BMP5XX_INTERRUPT_PULSED / BMP5XX_INTERRUPT_LATCHED - Interrupt mode
	   * BMP5XX_INTERRUPT_ACTIVE_LOW / BMP5XX_INTERRUPT_ACTIVE_HIGH - Interrupt polarity
	   * BMP5XX_INTERRUPT_PUSH_PULL / BMP5XX_INTERRUPT_OPEN_DRAIN - Interrupt drive
	   * BMP5XX_INTERRUPT_DATA_READY, BMP5XX_INTERRUPT_FIFO_FULL, etc. - Interrupt sources (can combine with |)
	   */
	  delay(200);
	  Log::I(F("Configuring interrupt pin with data ready source..."));
	  //BMP5XX_INTERRUPT_PULSED = 脉冲方式，50us
	  //BMP5XX_INTERRUPT_LATCHED = 锁存方式，需要通过调用bmp5_get_interrupt_status清除标志
	  if (!bmp.configureInterrupt(BMP5XX_INTERRUPT_PULSED, BMP5XX_INTERRUPT_ACTIVE_HIGH, BMP5XX_INTERRUPT_PUSH_PULL, BMP5XX_INTERRUPT_DATA_READY, true))
	  {
		  Log::E(F("Configuring interrupt pin error!"));
		  delay(100);
	  }

	  Log::I(F("=== Current Sensor Configuration ==="));

	  delay(200);
	  // Pretty print temperature oversampling inline
	  Log::I(F("Temperature Oversampling: "));
	  switch(bmp.getTemperatureOversampling()) {
	    case BMP5XX_OVERSAMPLING_1X:   Log::I(F("1X")); break;
	    case BMP5XX_OVERSAMPLING_2X:   Log::I(F("2X")); break;
	    case BMP5XX_OVERSAMPLING_4X:   Log::I(F("4X")); break;
	    case BMP5XX_OVERSAMPLING_8X:   Log::I(F("8X")); break;
	    case BMP5XX_OVERSAMPLING_16X:  Log::I(F("16X")); break;
	    case BMP5XX_OVERSAMPLING_32X:  Log::I(F("32X")); break;
	    case BMP5XX_OVERSAMPLING_64X:  Log::I(F("64X")); break;
	    case BMP5XX_OVERSAMPLING_128X: Log::I(F("128X")); break;
	    default: Log::I(F("Unknown")); break;
	  }

	  delay(200);
	  // Pretty print pressure oversampling inline
	  Log::I(F("Pressure Oversampling: "));
	  switch(bmp.getPressureOversampling()) {
	    case BMP5XX_OVERSAMPLING_1X:   Log::I(F("1X")); break;
	    case BMP5XX_OVERSAMPLING_2X:   Log::I(F("2X")); break;
	    case BMP5XX_OVERSAMPLING_4X:   Log::I(F("4X")); break;
	    case BMP5XX_OVERSAMPLING_8X:   Log::I(F("8X")); break;
	    case BMP5XX_OVERSAMPLING_16X:  Log::I(F("16X")); break;
	    case BMP5XX_OVERSAMPLING_32X:  Log::I(F("32X")); break;
	    case BMP5XX_OVERSAMPLING_64X:  Log::I(F("64X")); break;
	    case BMP5XX_OVERSAMPLING_128X: Log::I(F("128X")); break;
	    default: Log::I(F("Unknown")); break;
	  }

	  delay(200);
	  // Pretty print IIR filter coefficient inline
	  Log::I(F("IIR Filter Coefficient: "));
	  switch(bmp.getIIRFilterCoeff()) {
	    case BMP5XX_IIR_FILTER_BYPASS:   Log::I(F("Bypass (No filtering)")); break;
	    case BMP5XX_IIR_FILTER_COEFF_1:  Log::I(F("1 (Light filtering)")); break;
	    case BMP5XX_IIR_FILTER_COEFF_3:  Log::I(F("3 (Medium filtering)")); break;
	    case BMP5XX_IIR_FILTER_COEFF_7:  Log::I(F("7 (More filtering)")); break;
	    case BMP5XX_IIR_FILTER_COEFF_15: Log::I(F("15 (Heavy filtering)")); break;
	    case BMP5XX_IIR_FILTER_COEFF_31: Log::I(F("31 (Very heavy filtering)")); break;
	    case BMP5XX_IIR_FILTER_COEFF_63: Log::I(F("63 (Maximum filtering)")); break;
	    case BMP5XX_IIR_FILTER_COEFF_127:Log::I(F("127 (Maximum filtering)")); break;
	    default: Log::I(F("Unknown")); break;
	  }

	  delay(200);
	  // Pretty print output data rate inline
	  Log::I(F("Output Data Rate: "));
	  switch(bmp.getOutputDataRate()) {
	    case BMP5XX_ODR_240_HZ:   Log::I(F("240 Hz")); break;
	    case BMP5XX_ODR_218_5_HZ: Log::I(F("218.5 Hz")); break;
	    case BMP5XX_ODR_199_1_HZ: Log::I(F("199.1 Hz")); break;
	    case BMP5XX_ODR_179_2_HZ: Log::I(F("179.2 Hz")); break;
	    case BMP5XX_ODR_160_HZ:   Log::I(F("160 Hz")); break;
	    case BMP5XX_ODR_149_3_HZ: Log::I(F("149.3 Hz")); break;
	    case BMP5XX_ODR_140_HZ:   Log::I(F("140 Hz")); break;
	    case BMP5XX_ODR_129_8_HZ: Log::I(F("129.8 Hz")); break;
	    case BMP5XX_ODR_120_HZ:   Log::I(F("120 Hz")); break;
	    case BMP5XX_ODR_110_1_HZ: Log::I(F("110.1 Hz")); break;
	    case BMP5XX_ODR_100_2_HZ: Log::I(F("100.2 Hz")); break;
	    case BMP5XX_ODR_89_6_HZ:  Log::I(F("89.6 Hz")); break;
	    case BMP5XX_ODR_80_HZ:    Log::I(F("80 Hz")); break;
	    case BMP5XX_ODR_70_HZ:    Log::I(F("70 Hz")); break;
	    case BMP5XX_ODR_60_HZ:    Log::I(F("60 Hz")); break;
	    case BMP5XX_ODR_50_HZ:    Log::I(F("50 Hz")); break;
	    case BMP5XX_ODR_45_HZ:    Log::I(F("45 Hz")); break;
	    case BMP5XX_ODR_40_HZ:    Log::I(F("40 Hz")); break;
	    case BMP5XX_ODR_35_HZ:    Log::I(F("35 Hz")); break;
	    case BMP5XX_ODR_30_HZ:    Log::I(F("30 Hz")); break;
	    case BMP5XX_ODR_25_HZ:    Log::I(F("25 Hz")); break;
	    case BMP5XX_ODR_20_HZ:    Log::I(F("20 Hz")); break;
	    case BMP5XX_ODR_15_HZ:    Log::I(F("15 Hz")); break;
	    case BMP5XX_ODR_10_HZ:    Log::I(F("10 Hz")); break;
	    case BMP5XX_ODR_05_HZ:    Log::I(F("5 Hz")); break;
	    case BMP5XX_ODR_04_HZ:    Log::I(F("4 Hz")); break;
	    case BMP5XX_ODR_03_HZ:    Log::I(F("3 Hz")); break;
	    case BMP5XX_ODR_02_HZ:    Log::I(F("2 Hz")); break;
	    case BMP5XX_ODR_01_HZ:    Log::I(F("1 Hz")); break;
	    case BMP5XX_ODR_0_5_HZ:   Log::I(F("0.5 Hz")); break;
	    case BMP5XX_ODR_0_250_HZ: Log::I(F("0.25 Hz")); break;
	    case BMP5XX_ODR_0_125_HZ: Log::I(F("0.125 Hz")); break;
	    default: Log::I(F("Unknown")); break;
	  }

	  delay(200);
	  // Pretty print power mode inline
	  Log::I(F("Power Mode: "));
	  switch(bmp.getPowerMode()) {
	    case BMP5XX_POWERMODE_STANDBY:     Log::I(F("Standby")); break;
	    case BMP5XX_POWERMODE_NORMAL:      Log::I(F("Normal")); break;
	    case BMP5XX_POWERMODE_FORCED:      Log::I(F("Forced")); break;
	    case BMP5XX_POWERMODE_CONTINUOUS:  Log::I(F("Continuous")); break;
	    case BMP5XX_POWERMODE_DEEP_STANDBY:Log::I(F("Deep Standby")); break;
	    default: Log::I(F("Unknown")); break;
	  }

	  delay(20);

	/* Default settings from datasheet. */
	//bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
	//			  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
	//			  Adafruit_BMP280::SAMPLING_X4,    /* Pressure oversampling */
	//			  Adafruit_BMP280::FILTER_X4,      /* Filtering. */
	//			  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
#endif

	Log::I("CRSFRadio init\r\n");
	if (CRSFRadio.Init() != 0)
	{
		HAL_Delay(100);
		Log::E("CRSFRadio.Init fail");
		led.Error();
		Log::E("\n");
		while (1) delay(10);
	}


	Log::I("Servo init\r\n");
	Servo.Init();
    HAL_Delay(1);
    for (int n = 0; n < 3; n++)
    {
		for (int i=0; i<3; i++)
		{
			float motor_value[4] = {1000, 1000, 1000, 1000};
			Servo.Write(motor_value);
			HAL_Delay(1);
		}
		//等待电机同步
		HAL_Delay(3);
	}
    HAL_Delay(200);

    //启动200hz时钟
    HAL_TIM_Base_Start_IT(&htim1);

	//启动500hz时钟
	HAL_TIM_Base_Start_IT(&htim2);

    //启动调试工具接收
    UART1_DMA_Recv_Start();

    controller.Init();

    location_kalman.set_state(0, 0, 0, 0, micros() * 1E-6);

    //光流算法初始化
    FlowKalman_Init(&flow_kf);

    for (int i=0; i<3; i++)
    {
		if (!bottom.Init())
		{
			Log::E("bottom init fail");
			Log::E("\n");
		}
		else
		{
			break;
		}
    }


	Log::I("application init done\r\n");

	mIsInit = true;
}

void Application::_500Hz_Event()
{
	_500hz_irq++;
	//激活线程
	//tx_semaphore_put(&_500hz_semaphore);
}

void Application::Loop()
{
	LOOP_STATE state = LOOP_IDLE;
	while(1)
	{
		uint32_t tick = HAL_GetTick();
		(void)tick;
		if (_500hz_irq > 0)
		{
			Exec500hz();
		}
		else if (_200hz > 0)
		{
			Exec200hz();
		}
		else
		{
			switch(state)
			{
			case LOOP_IDLE:
				break;
			case LOOP_RADIO:
				RadioUpdate();
				break;
			case LOOP_UART1:
				link.RecvLoop();
				break;
#ifndef USE_IMUEX
			case LOOP_BMP:
				BmpUpdate();
				break;
#endif
#if 0
			case LOOP_IMU:
				ImuUpdate();
				break;

			case LOOP_TOF:
				TofUpdate();
				break;

			case LOOP_PMW:
				PMW_Task();
				break;
#endif
			case LOOP_LOG:
				LogUpdate();
				break;
			case LOOP_BOTTOM:
				if (mode != MODE_CAR)
				{
					BottomUpdate();
				}
				break;
			case LOOP_SDWRITE:
				file_log_write();
				break;
			case LOOP_QMC5883L:
				//qmc5883l();
				break;
			case LOOP_GPS:
				gps();
				break;
			case LOOP_FRONT:
				FrontCheckUpdate();
				break;
			case LOOP_CAR:
				if (mode == MODE_CAR)
				{
					car.Loop();
				}
				else
				{
					static uint32_t prev_tick = 0;
					if ((tick - prev_tick) > 3000)
					{
						prev_tick = tick;
						car.Loop();
					}
				}
				break;
			default:
				break;
			}

			//检查处理是否超过5毫秒，如果超过，会影响200hz控制频率和imu中断处理
			uint32_t remain = HAL_GetTick() - tick;
			if (remain >= 10)
			{
				Log::W("process timeout(%d): %d ms", (int)state, remain);
			}
			state = (LOOP_STATE)(((int)state) + 1);
			if (state >= LOOP_END)
			{
				state = LOOP_RADIO;
			}

			CALC_FUNC_FREQ("Application::Loop");


			mLoopTick = HAL_GetTick();
			//HAL_Delay(1);
		}
	}
}

bool Application::CheckLoopTimeout()
{
	uint32_t now = HAL_GetTick();
	uint32_t remain = now - mLoopTick;
	if (remain > 20)
	{
		return true;
	}
	return false;
}

void Application::Tim200hz()
{
	_200hz++;
	//激活线程
	//tx_semaphore_put(&_200hz_semaphore);
}

void Application::RadioUpdate()
{
	int CrsfRoll = 1500;
	int CrsfPitch = 1500;
	int CrsfThrottle = 1500;
	int CrsfYaw = 0;
	int CrsfMode = 0;
	int CrsfLight = 0;
	int CrsfReturnHome = 0;
	int CrsfArmed = 0;
	uint32_t now = millis();

	CRSFRadio.Loop();
	CRSFRadio.GetData(&CrsfRoll, &CrsfPitch, &CrsfThrottle, &CrsfYaw, &CrsfMode, &CrsfLight, &CrsfReturnHome, &CrsfArmed);

	bool thrIdle = false;
	//检查油门是否居中
	if (CrsfThrottle > (cfg.param.rcl_thr_mid - cfg.param.rcl_thr_dead) && CrsfThrottle < (cfg.param.rcl_thr_mid + cfg.param.rcl_thr_dead))
	{
		thrIdle = true;
	}
	(void)thrIdle;
	bool rolIdle = false;
	//检查roll是否居中
	if (CrsfRoll > (cfg.param.rcl_rol_mid - cfg.param.rcl_rol_dead) && CrsfRoll < (cfg.param.rcl_rol_mid + cfg.param.rcl_rol_dead))
	{
		rolIdle = true;
	}
	(void)rolIdle;
	bool pitIdle = false;
	if (CrsfPitch > (cfg.param.rcl_pit_mid - (cfg.param.rcl_pit_dead + /*遥控器校准*/10)) && CrsfPitch < (cfg.param.rcl_pit_mid + (cfg.param.rcl_pit_dead + /*遥控器校准*/10)))
	//检查pitch是否居中
	{
		pitIdle = true;
	}
	(void)pitIdle;

	bool yawIdle = false;
	if (CrsfYaw > (cfg.param.rcl_yaw_mid - cfg.param.rcl_yaw_dead) && CrsfYaw < (cfg.param.rcl_yaw_mid + cfg.param.rcl_yaw_dead))
	//检查yaw是否居中
	{
		yawIdle = true;
	}
	if (!rolIdle || !pitIdle || !yawIdle)
	{
		//flow_init(imuex.yaw);
		//flow_init(yaw);
		//total_delta_x = 0;
		//total_delta_y = 0;
		bottom.Reset(yaw);
		//FlowKalman_Reset(&flow_kf);
		gps_hold();

		if (mode == MODE_RETURN_HOME)
		{
			mode = MODE_DRONE;
		}
	}

    static uint32_t BtnModeDownTime = 0;
    static uint32_t BtnModeDownCount = 0;
    static uint32_t BtnModeUpTime = 0;
    static bool IsChangeMode = false;
    //模式切换
    if (CrsfMode > 1500)
    {
    	if (BtnModeDownTime == 0)
    	{
    		BtnModeDownTime = now;
    	}
    	if ((now - BtnModeDownTime) > 500 && BtnModeDownCount > 10) //500ms 连续10次
    	{
    		BtnModeUpTime = 0;
    		if (!IsChangeMode)
    		{
    			float alt = Alt_getH();//Alt_getH();
    			switch((int)mode)
    			{
    			case MODE_DRONE:
    				if (/*!Flighting*/alt < 0.1)
    				{
    					mode = MODE_CAR;
      					IsChangeMode = true;
    				}
    				break;
    			case MODE_CAR:
       				IsChangeMode = true;
       				mode = MODE_DRONE;
       				break;
    			case MODE_NONE:
    				mode = MODE_CAR;
    				IsChangeMode = true;
    				break;
    			}
    		}
    	}
    	BtnModeDownCount++;
    }
    else
    {
    	if (BtnModeUpTime == 0)
    	{
    		BtnModeUpTime = now;
    	}
    	if ((now - BtnModeUpTime) > 500) //500ms
    	{
    		BtnModeDownTime = 0;
    		IsChangeMode = false;
    	}
    	BtnModeDownCount = 0;
    }

    static uint32_t BtnReturnDownTime = 0;
    static uint32_t BtnReturnDownCount = 0;
    static uint32_t BtnReturnUpTime = 0;
    static bool IsChangeReturn = false;
    //返航切换
    if (CrsfReturnHome > 1500)
    {
    	if (BtnReturnDownTime == 0)
    	{
    		BtnReturnDownTime = now;
    	}
    	if ((now - BtnReturnDownTime) > 500 && BtnReturnDownCount > 10) //500ms 连续10次
    	{
    		BtnReturnUpTime = 0;
    		if (!IsChangeReturn)
    		{
    			if (mode == MODE_RETURN_HOME)
    			{
    				mode = MODE_DRONE;
    			}
    			else if (mode == MODE_DRONE)
    			{
    				mode = MODE_RETURN_HOME;
    			}
    		}
    		IsChangeReturn = true;
    	}
    	BtnReturnDownCount++;
    }
    else
    {
    	if (BtnReturnUpTime == 0)
    	{
    		BtnReturnUpTime = now;
    	}
    	if ((now - BtnReturnUpTime) > 500) //500ms
    	{
    		BtnReturnDownTime = 0;
    		IsChangeReturn = false;
    	}
    	BtnReturnDownCount = 0;
    }

    //解锁
    static uint32_t BtnArmedDownTime = 0;
    static uint32_t BtnArmedDownCount = 0;
    static uint32_t BtnArmedUpTime = 0;
    static bool IsChangeArmed = false;
    if (CrsfArmed > 1500)
    {
    	if (BtnArmedUpTime == 0)
		{
    		BtnArmedUpTime = now;
		}
		if ((now - BtnArmedUpTime) > 500) //500ms
		{
			BtnArmedDownTime = 0;
			IsChangeArmed = false;

	    	//检查传感器
	    	if (CheckUpdateTimeout())
	    	{
	    		PERIODIC_LOGI(1000, "CheckUpdateTimeout, can not armed.");
	    	}
	    	else if (CheckBmpTimeout())
	    	{
	    		PERIODIC_LOGI(1000, "CheckBmpTimeout, can not armed.");
	    	}
	    	else if (!IsBmpDataValid(mBmpAlt) && cfg.param.sys_fly_mode != 0)
	    	{
	    		PERIODIC_LOGI(2000, "bmp data error: %.2f, can not armed.", mBmpAlt);
	    	}
	    	else
	    	{
	    		if (!armed)
	    		{
	    			gps_hold();
	    		}
	    		armed = true;
	    	}
		}
		BtnArmedDownCount = 0;
    }
    else
    {
    	if (BtnArmedDownTime == 0)
		{
    		BtnArmedDownTime = now;
		}
		if ((now - BtnArmedDownTime) > 500 && BtnArmedDownCount > 5) //500ms 连续5次
		{
			BtnArmedUpTime = 0;
			if (!IsChangeArmed)
			{
				armed = false;
				mode = MODE_CAR;
			}
			IsChangeArmed = true;
		}
		BtnArmedDownCount++;
    }

    //车灯
    static uint32_t BtnLightDownTime = 0;
    static uint32_t BtnLightDownCount = 0;
    static uint32_t BtnLightUpTime = 0;
    static bool IsChangeLight = false;
    if (CrsfLight > 1500)
    {
    	if (BtnLightUpTime == 0)
		{
    		BtnLightUpTime = now;
		}
		if ((now - BtnLightUpTime) > 500) //500ms
		{
			BtnLightDownTime = 0;
			IsChangeLight = false;

	    	//HAL_GPIO_WritePin(LED_EN_GPIO_Port, LED_EN_Pin, GPIO_PIN_SET);
			car.setLedOn();
		}
		BtnLightDownCount = 0;
    }
    else
    {
    	if (BtnLightDownTime == 0)
		{
    		BtnLightDownTime = now;
		}
		if ((now - BtnLightDownTime) > 500 && BtnLightDownCount > 5) //500ms 连续5次
		{
			BtnLightUpTime = 0;
			if (!IsChangeLight)
			{
				//HAL_GPIO_WritePin(LED_EN_GPIO_Port, LED_EN_Pin, GPIO_PIN_RESET);
				car.setLedOff();
			}
			IsChangeLight = true;
		}
		BtnLightDownCount++;
    }

    if (mode == MODE_CAR || cfg.param.sys_fly_mode == 3)
    {
    	int value = -1 * (1500 - CrsfRoll) * 1.0f / 500 * 90 + 90;
    	int speed = (1500 - CrsfPitch) * 1.0f / 500 * 100;
    	if (speed < 0)
    	{
    		speed = -1 * (100 + speed);
    	}
    	else
    	{
    		speed = 100 - speed;
    	}
    	if (rolIdle)
    	{
    		car.setServo(90);
    	}
    	else
    	{
    		car.setServo(value);
    	}

    	//前避障
    	if (GetFrontDistance() < 2.0 && GetFrontDistance() > 0.2 && cfg.param.sys_fly_mode != 3)
    	{
    		if (speed < 0)
    		{
    			speed = 0;
    		}
    	}

    	if (pitIdle || !IsConnect)
    	{
    		car.setMotor(0);
    	}
    	else
    	{
    		car.setMotor(speed);
    	}
    }
    else
    {
    	car.setServo(90);
    	car.setMotor(0);
    }
}

void Application::BmpIrqEvent()
{
	bmp_irq++;
}

void Application::BmpUpdate()
{
	uint32_t tick = HAL_GetTick();
	(void)tick;

	//读取起飞海拔高度
	static float total = 0;
	static int count = 0;
	static int i = 0;

	if ((mBmpLandAlt == 0 || mBmpLandAlt == NAN) && i < 50)
	{
		if (bmp_irq > 0)
		{
			bmp_irq = 0;
			if (bmp.performReading()) {
				float _alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
				if (NAN != _alt)
				{
					count++;
					total += _alt;
					i++;
				}
			}
		}
	}
	if (i >= 50)
	{
		if (count > 0)
		{
			mBmpLandAlt = total / count;
		}
		total = 0;
		count = 0;
		i = 0;
	}
	if (mBmpLandAlt == 0 || NAN == mBmpLandAlt)
	{
		return;
	}

	bool bmp_data_ready = false;
	if (bmp_irq > 0)
	{
		if (bmp_irq > 1)
		{
			Log::W("bmp_irq slow: %d", bmp_irq);
		}
		bmp_irq = 0;
		bmp_data_ready = true;
	}
#if 0
	//锁存方式
	else
	{
		static uint32_t bmp_tick = 0;
		if ((tick - bmp_tick) > 100)
		{
			bmp_tick = tick;

			// Check if new data is ready before reading
			if (bmp.dataReady()) {
				bmp_data_ready = true;
			}
		}
	}
#endif
	static uint32_t refresh_tick = 0;
	uint32_t now = HAL_GetTick();
	//50hz读取
	if ((now - refresh_tick) >= 20)
	{
		refresh_tick = now;
		bmp_data_ready = true;
	}

	if (bmp_data_ready)
	{
		// Data is ready, perform reading
		if (bmp.performReading()) {
			float alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
			if (NAN != alt)
			{
				//根据IMU重置起飞高度(螺旋桨没有转动的时候， az的值变化在0.01以内)
				/*
				if (STABLE_FOR((imu.state.az - 1.0), 0.05, 1000) && fabsf(Alt_getH()) < 1.0)
				{
					mBmpLandAlt = alt;
				}
				*/
				float bmpalt = alt - mBmpLandAlt;

				//一阶低通滤波
				if (bmp_lpf.prev_output == 0.0f)
				{
					bmp_lpf.prev_output = bmpalt;
				}

				mBmpAlt = lpf_update(&bmp_lpf, bmpalt);
				//高度校准
				if ((HAL_GetTick() - tofupdate_tick) > 100)
				{
					Alt_updateBarAlt(mBmpAlt - mBmpCal, micros());
				}
				bmp_update_tick = now;
				if (!IsBmpDataValid(bmpalt))
				{
					led.Fail();
					PERIODIC_LOGE(2000, "bmp data error: %.2f", bmpalt);
				}
				else
				{
					//PERIODIC_LOGI(3000, "bmp alt: %.2f", bmpalt);
				}
			}
			//Log::I(F("Temperature = %f °C"), bmp.temperature);
			//Log::I(F("Pressure = %f hPa"), bmp.pressure);
			//Log::I(F("Approx. Altitude = %f m"), bmp.readAltitude(SEALEVELPRESSURE_HPA));
			//Log::I("\n");
			//Log::I(F("alt=%.2f, LAlt=%.2f, mAlt=%.2f"), alt, mBmpLandAlt, mBmpAlt);
			//Log::I("alt=%f v=%f accelup=%f", Alt_getH(), Alt_getV(), 0);
		}
	}

}

void Application::Control()
{
	static float prev_yaw = 0; //返航前的航向
	static bool has_prev_yaw = false;
	static bool near = false;
	static bool is_flying_home = false;
	//返航处理
	if (mode == MODE_RETURN_HOME)
	{
		// 记录返航开始时的航向（只记录一次）
		if (!has_prev_yaw)
		{
			prev_yaw = yaw;
			has_prev_yaw = true;
		}
		float dest_yaw = yaw;
		bool use_dest = false;
		PERIODIC_LOGI(2000, "distance: %f", hdistance);
		if (hdistance > 10 && !near)
		{
			//float heading = heading_to_target();
			float heading = heading_to_home();
			//PERIODIC_LOGI(2000, "heading:%f", heading);
			if (heading >= 0)
			{
				dest_yaw = heading;
				use_dest = true;
			}
		}
		else
		{
			near = true;
			dest_yaw = prev_yaw;
			use_dest = true;
		}

		if (near)
		{
			if (!is_flying_home)
			{
			    gps_hold();
			}
		}
		if (use_dest)
		{
		    // 1. 计算最短路径的航向误差，并标准化到 [-180, 180] 之间
		    float error = dest_yaw - controller.yaw_heading_reference;
		    if (error > 180.0f) {
		        error -= 360.0f;
		    } else if (error < -180.0f) {
		        error += 360.0f;
		    }

			static uint32_t prev_tick = 0;
			uint32_t now = HAL_GetTick();

			float step = 20.0f;
			// 每隔 30ms 步进一次
			if ((now - prev_tick) >= 50) {
				if (fabsf(error) <= step) {
					controller.yaw_heading_reference = dest_yaw;
				} else {
					controller.yaw_heading_reference += (error > 0.0f) ? step : -step;
				}

				// 将参考角限制在 0 ~ 360 度之间
				if (controller.yaw_heading_reference >= 360.0f) {
					controller.yaw_heading_reference -= 360.0f;
				} else if (controller.yaw_heading_reference < 0.0f) {
					controller.yaw_heading_reference += 360.0f;
				}
				PERIODIC_LOGI(2000, "dest_yaw:%f, yaw_heading:%f", dest_yaw, controller.yaw_heading_reference);
				prev_tick = now;
			}
			static uint32_t hold_tick = 0;
			if (fabsf(error) < step)
			{
				if (hold_tick == 0)
				{
					hold_tick = now;
				}
				if ((now - hold_tick) > 500)
				{
					is_flying_home = true;
					target_to_home();
					PERIODIC_LOGI(2000, "target to home");
				}
			}
			else
			{
				is_flying_home = false;
				bottom.Reset(yaw);
				hold_tick = 0;
			}
		}

		if (hdistance < 5)
		{
			gps_hold();
			mode = MODE_DRONE;
			prev_yaw = 0;
			has_prev_yaw = false;
			near = false;
			is_flying_home = false;
		}
	}
	else
	{
		prev_yaw = 0;
		has_prev_yaw = false;
		near = false;
		is_flying_home = false;
	}

	//计算pid
	if (cfg.param.sys_fly_mode == 3)
	{
		controller.climb();
	}
	else
	{
		controller.PID_Calculation();
	}
	Servo.Write(controller.motor_speeds);
}


void Application::ImuUpdate()
{

#ifndef USE_IMUEX
	bool new_data = false;
	if (imu_update())
	{
		new_data = true;
	}
	if (new_data)
	{
		float imu_acc[2] = {-Ahr_getAccelY(), Ahr_getAccelX()};
		location_kalman.update(micros() * 1E-6, NULL, imu_acc);
	}


	if (new_data)
	{
		//根据az判断飞机是否静止
		static uint32_t stillness_tick = 0;
		if ((1 - imu.state.az) < 0.05)
		{
			//静止超过1秒，重置气压计高度
			if (stillness_tick > 0 && (HAL_GetTick() - stillness_tick) > 1000)
			{
				//mBmpLandAlt += mBmpAlt;
				//Alt_updateBarAlt(mBmpAlt, micros());
			}
		}
		else
		{
			stillness_tick = HAL_GetTick();
		}

		//更新alt
		Alt_updateAccelUp(Ahr_getAccelUp(), imu.state.ts);

		static uint32_t prev_tick = 0;
		uint32_t tick = HAL_GetTick();
		if ((tick - prev_tick) > 100)
		{
			prev_tick = tick;

			Log::I("ax=%f ay=%f az=%f gx=%f gy=%f gz=%f", imu.state.ax, imu.state.ay, imu.state.az, imu.state.gx, imu.state.gy, imu.state.gz);
			Log::I("roll=%f pitch=%f yaw=%f", roll, pitch, yaw);
			Log::I("alt=%f v=%f accelup=%f", Alt_getH(), Alt_getV(), Ahr_getAccelUp());
		}
	}
#else
	//根据TOF校准高度
	if (mIsTofUpdate)
	{
		mIsTofUpdate = false;
		imuex.CalAlt(mTofAlt);
	}

	if (imu_irq == 0)
	{
		return;
	}
	static uint32_t fail_tick = 0;
	if (!imuex.Update())
	{
		if (fail_tick == 0)
		{
			fail_tick = HAL_GetTick();
		}
		uint32_t fail_remain = HAL_GetTick() - fail_tick;
		if (fail_remain > 100)
		{
			imufail = true;
		}
		return;
	}
	fail_tick = 0;
	imu_update_tick = HAL_GetTick();

	if (start_yaw_rad == 0)
	{
		flow_init(imuex.yaw);
	}

	imufail = false;
	imu_irq = 0;
	static uint32_t prev_tick = 0;
	uint32_t tick = HAL_GetTick();
	if ((tick - prev_tick) > 100)
	{
		prev_tick = tick;

		Log::I("ax=%f ay=%f az=%f gx=%f gy=%f gz=%f", imuex.ax, imuex.ay, imuex.az, imuex.gx, imuex.gy, imuex.gz);
		Log::I("roll=%f pitch=%f yaw=%f", imuex.roll, imuex.pitch, imuex.yaw);
		Log::I("alt=%f v=%f accelup=0", imuex.alt, imuex.vz);
	}
#endif

	static uint32_t fail_tick = 0;
	if (!new_data)
	{
		if (fail_tick == 0)
		{
			fail_tick = HAL_GetTick();
		}
		uint32_t fail_remain = HAL_GetTick() - fail_tick;
		if (fail_remain > 100)
		{
			imufail = true;
		}
		return;
	}
	fail_tick = 0;
	imu_update_tick = HAL_GetTick();


	imufail = false;
}


void Application::ImuEvent()
{
	imu_irq++;
}

void Application::LogUpdate()
{
	static uint32_t prev_tick = 0;
	uint32_t tick = HAL_GetTick();
	if ((tick - prev_tick) > 100)
	{
		prev_tick = tick;
		float pos_x, pos_y, vel_x, vel_y;
		Flow_GetState(&pos_x, &pos_y, &vel_x, &vel_y);
	    float dx_body, dy_body;
	    get_target_distance(&dx_body, &dy_body);
		/*
		Log::I("[LOC]x:%f,y:%f,vx:%f,vy:%f",
				location_kalman.get_x(), location_kalman.get_y(), location_kalman.get_vx(), location_kalman.get_vy());
				*/
		Log::I("[LOC]x:%f,y:%f,vx:%f,vy:%f", dx_body, dy_body, vel_x, vel_y);
	}
}

void Application::Exec500hz()
{
	if (_500hz_irq > 0)
	{
		if (_500hz_irq > 1)
		{
			Log::W("_500hz error: %d", _500hz_irq);
		}
		_500hz_irq = 0;

		ImuUpdate();

		// 假设你的 IMU 读取到的原始 Z 轴角速度是 dps (度/秒)
		float gyro_z_dps = imu.state.gz;

		// 2. 方向取反：将 "逆时针为正" 转换为 "顺时针(右转)为正"
		float yaw_rate_dps = -gyro_z_dps;

		// 3. 单位转换：度/秒 转换为 弧度/秒
		#define DEG_TO_RAD 0.017453292519943295f
		float yaw_rate_rad = yaw_rate_dps * DEG_TO_RAD;

		// ================= 加速度去重力 =================
		// 2. 将当前姿态角转为弧度
		float pitch_rad = pitch * DEG_TO_RAD;
		float roll_rad  = roll * DEG_TO_RAD;

		// 2. 利用四元数获取机体坐标系下的纯净加速度 (X:右正, Y:前正)
		float kalman_ax = Ahr_getAccelBodyRight(); // 右向加速度
		float kalman_ay = Ahr_getAccelBodyFront(); // 前向加速度

		// 预测步
		FlowKalman_Predict(&flow_kf, kalman_ax, kalman_ay, yaw_rate_rad, imu.state.dt);

		//CALC_FUNC_FREQ("500hz");
	}
}

void Application::Exec200hz()
{
	if (_200hz > 0)
	{
		if (_200hz > 1)
		{
			Log::W("_200hz error: %d", _200hz);
		}
		_200hz = 0;

		Control();

		CALC_FUNC_FREQ("200hz");
	}
}

void Application::RadioUpdateEvent()
{
	radio_update_tick = HAL_GetTick();
}

bool Application::CheckUpdateTimeout()
{
	if (!mIsInit)
	{
		return false;
	}
	uint32_t now = HAL_GetTick();
	if (radio_update_tick > 0 && (now - radio_update_tick) > 1000)
	{
		return true;
	}
	if (imu_update_tick > 0 && (now - imu_update_tick) > 1000)
	{
		return true;
	}
	return false;
}

bool Application::CheckBmpTimeout()
{
	if (!mIsInit)
	{
		return false;
	}
	uint32_t now = HAL_GetTick();
	if (bmp_update_tick > 0 && (now - bmp_update_tick) > 1000)
	{
		return true;
	}
	return false;
}

bool Application::IsBmpDataValid(float bmpalt)
{
	if (!armed)
	{
		//气压计误差在3米以上是不正常的
		if (fabsf(bmpalt) > 3.0f)
		{
			return false;
		}
	}
	return true;
}

void Application::BottomEvent()
{
	bottom_irq++;
}

void Application::BottomUpdate()
{
	bool need_update = false;
	static uint32_t prev_tick = 0;
	uint32_t tick = HAL_GetTick();
	uint32_t remain = tick - prev_tick;
	if (remain > (1000 / 50 )) //50hz发送姿态数据
	{
		need_update = true;
		prev_tick = tick;
		bottom.SetAhr(roll, pitch, yaw, imu.state.gx, imu.state.gy);
	}
	if (bottom_irq > 0 || remain > 15)
	{
		if (bottom_irq > 1)
		{
			//Log::W("bottom error: %d", bottom_irq);
		}
		bottom_irq = 0;
		if (bottom.Update())
		{
			static float prev_alt = 0;
			if (bottom.alt > 0 && bottom.alt < 4 && bottom.alt != prev_alt)
			{
				//自动校准气压计
				float tofalt = bottom.alt; //单位：米
				if (IsArmed()/* && tofalt > 0.1f*/) //小于0.1f可能是被遮盖了
				{
					mBmpCal = mBmpAlt - tofalt;
					Alt_updateBarAlt(tofalt, micros());
					tofupdate_tick = HAL_GetTick();
				}
				/*
				float alt = mBmpLandAlt + mBmpAlt;
				mBmpLandAlt = alt - tofalt;
				mBmpAlt = tofalt;
				*/
				//CALC_FUNC_FREQ("Bottom");
				prev_alt = tofalt;
			}
			//PERIODIC_LOGI(1000, "tof alt:%f", bottom.alt);
			need_update = true;
		}
	}

	//如果1秒内方向没有调整过来，就重置，说明可能因为振动导致出现很大的偏差
	static uint32_t prev_reset_tick = 0;
	static float zero_x_cmp = 0; //x是正数还是负数
	static float zero_y_cmp = 0; //x是正数还是负数
	if (prev_reset_tick == 0)
	{
		prev_reset_tick = tick;
	}
	if (zero_x_cmp == 0)
	{
		zero_x_cmp = bottom.x;
	}
	if (zero_y_cmp == 0)
	{
		zero_y_cmp = bottom.y;
	}
	if ((tick - prev_reset_tick) > 100 && ((zero_x_cmp < 0 && bottom.x < 0)
			|| (zero_y_cmp < 0 && bottom.y < 0)
			|| (zero_x_cmp > 0 && bottom.x > 0)
			|| (zero_y_cmp > 0 && bottom.y > 0)))
	{
		zero_x_cmp = 0;
		zero_y_cmp = 0;
		//bottom.Reset(yaw);
		prev_reset_tick = tick;
	}
	else
	{
		if ((zero_x_cmp < 0 && bottom.x > 0) || (zero_x_cmp > 0 && bottom.x < 0))
		{
			prev_reset_tick = tick;
			zero_x_cmp = bottom.x;
		}
		if ((zero_y_cmp < 0 && bottom.y > 0) || (zero_y_cmp > 0 && bottom.y < 0))
		{
			zero_y_cmp = bottom.y;
			prev_reset_tick = tick;
		}
	}


	uint32_t now_tick = micros();
	float dt_flow = (now_tick - last_flow_tick) / 1000000.0f;
	last_flow_tick = now_tick;
	if (need_update)
	{
		if (dt_flow > 0.0f && dt_flow < 1.0f)
		{
			static float prev_x = 0;
			static float prev_y = 0;
			if (prev_x == 0)
			{
				prev_x = bottom.x;
			}
			if (prev_y == 0)
			{
				prev_y = bottom.y;
			}
			// 4. 更新卡尔曼滤波器
			FlowKalman_UpdateFlow(&flow_kf, bottom.x - prev_x, bottom.y - prev_y, dt_flow, bottom.quality);
			prev_x = bottom.x;
			prev_y = bottom.y;
		}
	}

	PERIODIC_LOGI(200, "pmw3901=> tx:%f ty:%f x:%f y:%f", bottom.x, bottom.y, bottom.x1, bottom.y1);
	//PERIODIC_LOGI(200, "pmw3901=> tx:%f ty:%f x:%f y:%f", pos_x, pos_y, vel_x, vel_y);
}

void Application::Flow_GetState(float *pos_x, float *pos_y, float *vel_x, float *vel_y)
{
	FlowKalman_GetState(&flow_kf, pos_x, pos_y, vel_x, vel_y);
}

void Application::qmc5883l()
{
	static uint32_t prev_tick = 0;
	uint32_t tick = HAL_GetTick();
	uint32_t remain = tick - prev_tick;
	if (remain > (1000 / 100 )) //100hz
	{
		prev_tick = tick;
		if (QMC5883L_GetAngle(&qmc_yaw))
		{
			//起飞后磁力计被电机干扰
			if (!armed)
			{
				set_mag(qmc_yaw);
			}
			//Log::I("qmc_yaw:%f", qmc_yaw);
		}
	}
}

void Application::gps()
{
	poll_read_gps();
}

void Application::UpdateGPS(float gps_px, float gps_py, float gps_vx, float gps_vy, float hAcc)
{
	FlowKalman_UpdateGPS(&flow_kf, gps_px, gps_py, gps_vx, gps_vy, hAcc);
}

void Application::FrontCheckUpdate()
{
	//以下是超声波相关，超声波无法使用，电机一响，就返回异常数据
#if 0
	static bool isstart = false;
	static uint32_t start_tick = 0;

	if (!isstart)
	{
		//开始测距
		uint8_t reg_data[1] = {0x01};
		HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c3, (0x57<<1), reg_data, 1, 2);
		if (status != HAL_OK)
		{
			front_distance = 0;
			Log::E("FrontCheck HAL_I2C_Master_Transmit Error: %x", status);
			return;
		}
		start_tick = HAL_GetTick();
		isstart = true;
	}

	//200ms后才能读取结果
	if (isstart && (HAL_GetTick() - start_tick) > 200)
	{
		uint8_t reg_data[3];
		HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&hi2c3, (0x57<<1), reg_data, 3, 2);
		if (status == HAL_OK)
		{
			//距离mm
			front_distance = ((reg_data[0]<<16)+(reg_data[1]<<8)+reg_data[2])/1000; //((BYTE_H<<16)+(BYTE_M<<8)+ BYTE_L)/1000
		}
		else
		{
			front_distance = 0;
			Log::E("FrontCheck HAL_I2C_Master_Receive Error: %x", status);
		}
		isstart = false;
		if (front_distance > 200 && front_distance < 4000)
		{
			PERIODIC_LOGI(1000, "front_distance: %.2f", front_distance);
		}
	}
#endif
	static uint32_t update_tick = 0;
	if ((HAL_GetTick() - update_tick) > 1000)
	{
		front_distance = 0;
	}

	static uint32_t prev_tick = 0;
	if ((HAL_GetTick() - prev_tick) < 300)
	{
		return;
	}
	prev_tick = HAL_GetTick();

	typedef struct __attribute__((packed)){
		int16_t distance; //计算出的距离
		uint32_t tick;
		uint8_t validate;
	}st_front_data;
	st_front_data front_data;

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c3, 0x32 << 1, 1, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&front_data, sizeof(front_data), 2);
	if (status != HAL_OK)
	{
		if (status == HAL_ERROR)
		{
		    //HAL_I2C_DeInit(&hi2c1);
		    //HAL_I2C_Init(&hi2c1);
		}
		PERIODIC_LOGE(2000, "front Update Error: %x", status);
		return;
	}
	int16_t validate =
			front_data.distance + front_data.tick;
	if (front_data.validate != (validate & 0xff))
	{
		Log::E("front validate Error: %f != %f", front_data.validate, validate);
		return;
	}
	static int16_t prev_distance = 0;
	if (prev_distance == front_data.distance)
	{
		return;
	}
	prev_distance = front_data.distance;
	front_distance = front_data.distance / 100.0f;;
	if (front_distance > 0.5f && front_distance < 4.0f)
	{
		PERIODIC_LOGI(1000, "front_distance: %.2f", front_distance);
	}
	update_tick = HAL_GetTick();
}

float Application::GetFrontDistance()
{
	return front_distance;
}

void Application::mag_cal()
{
	//先等待10秒，让wifi透传连接完成
	HAL_Delay(10 * 1000);
	Log::I("mag calibration started, Please complete the operation within 2 minute.");
	uint32_t start_tick = HAL_GetTick();
	for (; (HAL_GetTick() - start_tick ) < 2 * 60 * 1000; )
	{
		if (poll_read_gps())
		{
			MagCalib_CollectData(gps_data.mx, gps_data.my, gps_data.mz);
		}
	}
	//保存配置文件
	float off_x, off_y, off_z;
	float sca_x, sca_y, sca_z;

	if (MagCalib_Compute(&off_x, &off_y, &off_z, &sca_x, &sca_y, &sca_z) == 0) {
		Log::I("calibration succ！\n");
		Log::I("Offset: X=%.2f, Y=%.2f, Z=%.2f\n", off_x, off_y, off_z);
		Log::I("Scale: SX=%.2f, SY=%.2f, SZ=%.2f\n", sca_x, sca_y, sca_z);
	    // ⚠️ 重要：将算出的 off 和 sca 保存到 Flash 或 EEPROM 中！
		cfg_set_param_mavlink("mag_cal_x", off_x);
		cfg_set_param_mavlink("mag_cal_y", off_y);
		cfg_set_param_mavlink("mag_cal_z", off_z);
		cfg_set_param_mavlink("mag_cal_sx", sca_x);
		cfg_set_param_mavlink("mag_cal_sy", sca_y);
		cfg_set_param_mavlink("mag_cal_sz", sca_z);
	} else {
		Log::I("calibration fail。\n");
	}
	Log::I("mag calibration done");
	cfg_set_param_mavlink("sys_fly_mode", 0);
	cfg_write_to_eeprom();
	while(1)
	{
		PERIODIC_LOGI(30000, "mag calibration done");
	}
}

