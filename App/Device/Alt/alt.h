/*
 * alt.h
 *
 *  Created on: Jul 30, 2025
 *      Author: bitstream
 */

#ifndef ALT_ALT_H_
#define ALT_ALT_H_

#include <stdint.h>

void Alt_Init();
//a: accel up in [m/s^2], ts: timestamp in [us]
void Alt_updateAccelUp(float a, uint32_t ts);
//altitude: barometric altitude in [m], ts: timestamp in [us]
void Alt_updateBarAlt(float alt, uint32_t ts);
float Alt_getH();
float Alt_getV();


#endif /* ALT_ALT_H_ */
