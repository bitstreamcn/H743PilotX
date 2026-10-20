/*
 * alt.c
 *
 *  Created on: Jul 30, 2025
 *      Author: bitstream
 */
#include "alt.h"
#include "KalmanFilter.h"
#include <stdint.h>
#include "common.h"

static uint32_t tsa = 0;


void Alt_Init()
{
    //default parameters
    float altCov = 0.16;  //meadured stdev BMP280 = 0.4 [m], BMP581 = 0.2 [m]
    float accCov = 0.000009; //measured stdev MPU6500 @ 16G = 0.003 [G]
    float biasCov = 1;

    //Serial.printf("ALT: KALMAN3  altCov=%f accCov=%f biasCov=%f\n", altCov, accCov, biasCov);

    KalmanFilter_setup(altCov, accCov, biasCov);

    tsa = 0;

    Alt_updateBarAlt(0, micros());
}



//a: accel up in [m/s^2], ts: timestamp in [us]
void Alt_updateAccelUp(float a, uint32_t ts) {
  if(tsa!=0) {
    float dt = 1e-6 * (ts - tsa);
    KalmanFilter_propagate(a, dt);
  }
  tsa = ts;
};

//altitude: barometric altitude in [m], ts: timestamp in [us]
void Alt_updateBarAlt(float alt, uint32_t ts) {
	KalmanFilter_update(alt);
}

float Alt_getH() {return KalmanFilter_getH();} //altitude estimate in [m]
float Alt_getV() {return KalmanFilter_getV();} //vertical up speed (climb rate) estimate in [m/s]


