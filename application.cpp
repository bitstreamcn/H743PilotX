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

	Log::I("imu init");

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

	if (!bottom.Init())
	{
		Log::E("bottom init fail");
		Log::E("\n");
		while (1) delay(10);
	}


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
				BottomUpdate();
				break;
			case LOOP_SDWRITE:
				file_log_write();
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
	uint32_t now = millis();

	CRSFRadio.Loop();
	CRSFRadio.GetData(&CrsfRoll, &CrsfPitch, &CrsfThrottle, &CrsfYaw, &CrsfMode, &CrsfLight, &CrsfReturnHome);

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

	    	HAL_GPIO_WritePin(LED_EN_GPIO_Port, LED_EN_Pin, GPIO_PIN_SET);
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
	    		armed = true;
	    	}
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
				HAL_GPIO_WritePin(LED_EN_GPIO_Port, LED_EN_Pin, GPIO_PIN_RESET);
				armed = false;
				mode = MODE_CAR;
			}
			IsChangeLight = true;
		}
		BtnLightDownCount++;
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
				mBmpAlt = lpf_update(&bmp_lpf, bmpalt);
				//高度校准
				Alt_updateBarAlt(mBmpAlt - mBmpCal, micros());
				bmp_update_tick = now;
				if (!IsBmpDataValid(bmpalt))
				{
					led.Fail();
					PERIODIC_LOGE(2000, "bmp data error: %.2f", bmpalt);
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
	//计算pid
	controller.PID_Calculation();

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
		/*
		Log::I("[LOC]x:%f,y:%f,vx:%f,vy:%f",
				location_kalman.get_x(), location_kalman.get_y(), location_kalman.get_vx(), location_kalman.get_vy());
				*/
		//Log::I("[LOC]x:%f,y:%f,vx:%f,vy:%f", 0.0f, 0.0f, imuex.vx, imuex.vy);
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
		//气压计误差在1米以上是不正常的
		if (fabsf(bmpalt) > 1.0f)
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
	static uint32_t prev_tick = 0;
	uint32_t tick = HAL_GetTick();
	uint32_t remain = tick - prev_tick;
	if (remain > (1000 / 50 )) //50hz发送姿态数据
	{
		prev_tick = tick;
		bottom.SetAhr(roll, pitch, yaw);
	}
	if (bottom_irq > 0 || remain > 15)
	{
		if (bottom_irq > 1)
		{
			//Log::W("bottom error: %d", bottom_irq);
		}
		bottom_irq = 0;
		if (bottom.Update() && bottom.alt > 0 && bottom.alt < 4)
		{
			//自动校准气压计
			float tofalt = bottom.alt; //单位：米
			if (IsArmed())
			{
				mBmpCal = mBmpAlt - tofalt;
			}
			/*
			float alt = mBmpLandAlt + mBmpAlt;
			mBmpLandAlt = alt - tofalt;
			mBmpAlt = tofalt;
			*/
			//Alt_updateBarAlt(tofalt, micros());
		}
	}
	PERIODIC_LOGI(200, "pmw3901=> tx:%f ty:%f x:%f y:%f", bottom.x, bottom.y, 0, 0);
}

