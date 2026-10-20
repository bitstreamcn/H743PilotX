#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "./ICM-42688P/SL_ICM_42688P_Driver.h"

typedef struct  {
    uint32_t sampleRate; 	//sample rate [Hz]
    int pin_int; 			//IMU data ready interrupt pin
    int gizmo; 				//the gizmo to use
    void *spi_bus; 			//SPI bus
    int spi_cs; 			//SPI select pin
    void *i2c_bus; 			//i2c bus (only used if spi_bus == nullptr)
    uint8_t i2c_adr; 		//i2c address. 0=default address
    bool uses_i2c; 			//use I2C bus?
}ImuConfig;

#define DEFAULT_ImuConfig { \
	250, 		/*sample rate [Hz]*/ \
	-1, 		/*IMU data ready interrupt pin*/ \
	0, 			/*the gizmo to use*/ \
	NULL, 		/*SPI bus*/ \
	-1, 		/*SPI select pin*/ \
	NULL, 		/*i2c bus (only used if spi_bus == nullptr)*/ \
	0, 		/*i2c address. 0=default address*/ \
	false 		/*use I2C bus?*/ \
}

//imu sample data (raw, uncorrected and unfiltered)
typedef struct  {
    volatile uint32_t ts; 	//sample low level interrupt trigger timestamp [us]
    volatile float dt; 		//time since last sample [seconds]
    volatile float ax; 		//"North" acceleration [G]
    volatile float ay; 		//"East" acceleration [G]
    volatile float az; 		//"Down" acceleration [G]
    volatile float gx; 		//"North" rotation speed [deg/s]
    volatile float gy; 		//"East" rotation speed [deg/s]
    volatile float gz; 		//"Down" rotation speed [deg/s]
    volatile float mx; 		//"North" magnetic flux [uT]
    volatile float my; 		//"East" magnetic flux [uT]
    volatile float mz; 		//"Down" magnetic flux [uT]
    volatile float temp; //temperature [C]
}ImuState;

#define DEFAULT_ImuState { \
	0,  /*sample low level interrupt trigger timestamp [us]*/ \
	0,  /*time since last sample [seconds]*/ \
	0,  /*"North" acceleration [G]*/ \
	0,  /*"East" acceleration [G]*/ \
	0,  /*"Down" acceleration [G]*/ \
	0,  /*"North" rotation speed [deg/s]*/ \
	0,  /*"East" rotation speed [deg/s]*/ \
	0,  /*"Down" rotation speed [deg/s]*/ \
	0,  /*"North" magnetic flux [uT]*/ \
	0,  /*"East" magnetic flux [uT]*/ \
	0,  /*"Down" magnetic flux [uT]*/ \
	0   /*temperature [C]*/ \
}


typedef struct {
	ImuState  state;
    ImuConfig config;

    //interrupt statistics
    volatile uint32_t interrupt_cnt; //number of times interrupt was triggered since start
    uint32_t update_cnt; //number of times imu task was executed since start
    uint32_t overrun_cnt; //number of interrupt overruns (should stay 0)
    bool _imu_interrupt_busy; //is interrupt handler running?

    //statistics - sum of values since last statReset()
    uint32_t stat_cnt; //number of accumulated samples
    uint32_t stat_reset_ts; //last time statReset() was called
    uint32_t stat_latency; //summed  interrupt latency from start of interrupt handler to start of interrupt task in us
    uint32_t stat_io_runtime; //summed  runtime of SPI/I2C io transfer in us
    uint32_t stat_runtime; //summed  runtime imu update including io transfer in us
    uint32_t stat_runtime_max; //max runtime imu update including transfer in us, since last reset to 0

    //pointer to onUpdate event handler
    void (*onUpdate)(void);

    uint32_t _sampleRate; //sensor sample rate in Hz
} Imu;

#define DEFAULT_Imu { \
		DEFAULT_ImuState, \
		DEFAULT_ImuConfig, \
		0, 			/*number of times interrupt was triggered since start*/ \
		0, 			/*number of times imu task was executed since start*/ \
		0,  			/*number of interrupt overruns (should stay 0)*/ \
		false, 	/*is interrupt handler running?*/ \
		0, 			/*number of accumulated samples*/ \
		0, 			/*last time statReset() was called*/ \
		0, 			/*summed  interrupt latency from start of interrupt handler to start of interrupt task in us*/ \
		0, 			/*summed  runtime of SPI/I2C io transfer in us*/ \
		0, 			/*summed  runtime imu update including io transfer in us*/ \
		0, 			/*max runtime imu update including transfer in us, since last reset to 0*/ \
		NULL, 			/*onUpdate*/ \
		250 				/*sensor sample rate in Hz*/ \
}

extern Imu imu;
extern float q[4];

extern float roll;               // roll in degrees: -180 to 180, roll right is positive
extern float pitch;              // pitch in degrees: -90 to 90, pitch up is positive
extern float yaw;                // yaw in degrees: -180 to 180, yaw right is positive

extern volatile uint8_t imu_data_irq;

bool imu_init();

int imu_setup(); // Use config to setup gizmo, returns 0 on success, or error code
bool imu_update(); // Returns true if state was updated
bool  imu_installed();//returns true if a gizmo is installed

//methods
bool imu_waitNewSample(); //wait for new sample, returns false on fail
bool imu_hasMag(); //returns true if IMU has a magnetometer
bool imu_usesI2C(); //returns true if IMU uses I2C bus (not SPI bus)
uint32_t imu_getSampleRate();  //sensor sample rate in Hz
uint32_t inline imu_getSamplePeriod() {return (imu._sampleRate != 0 ? 1000000 / imu._sampleRate : 1000000);} //sensor sample period in us
void imu_statReset();

//low level interrupt handler (should be private, but is public, because called from interrupt)
void imu__interrupt_handler();

float Ahr_getAccelX();
float Ahr_getAccelY();
float Ahr_getAccelUp();
float Ahr_getAccelBodyFront();
float Ahr_getAccelBodyRight();

void set_mag(float mag);


