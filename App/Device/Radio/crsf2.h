/*
 * crsf2.h
 *
 *  Created on: Jul 26, 2025
 *      Author: bitstream
 */

#ifndef RADIO_CRSF2_H_
#define RADIO_CRSF2_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "crsf_protocol2.h"

enum eFlightMode{
	MODE_NONE,
	MODE_CAR, //车模式
	MODE_DRONE, //无人机模式
	MODE_RETURN_HOME, //返航模式
};

enum eFailsafeAction { fsaNoPulses, fsaHold };

void crsf2_begin();
void crsf2_update();
void crsf2_write(uint8_t b);
void crsf2_write2(const uint8_t *buf, size_t len);
void crsf2_queuePacket(uint8_t addr, uint8_t type, const void *payload, uint8_t len);
void crsf2_writePacket(uint8_t addr, uint8_t type, const void *payload, uint8_t len);
void crsf2_handleSerialIn(const uint8_t *buf, size_t len);
void crsf2_sendChannels();
void crsf2_setChannel(unsigned int ch, uint16_t val);
long crsf_map(long x, long in_min, long in_max, long out_min, long out_max);

// Return current channel value (1-based) in us
int crsf2_getChannel(unsigned int ch);
const crsf_channels_t *crsf2_getChannelsPacked();
const crsfLinkStatistics_t *crsf2_getLinkStatistics();
const crsf_sensor_gps_t *crsf2_getGpsSensor();
const crsf_sensor_vario_t *crsf2_getVarioSensor();
const crsf_sensor_baro_altitude_t *crsf2_getBaroAltitudeSensor();
const crsf_sensor_attitude_t *crsf2_getAttitudeSensor();
bool crsf2_isLinkUp();

struct CrsfSerialEvent2_s
{
	// Event Handlers
	void (*onLinkUp)();
	void (*onLinkDown)();
	// OobData is any byte which is not CRSF, including passthrough
	void (*onOobData)(uint8_t b);
	// CRSF Packet Callbacks
	void (*onPacketChannels)();
	void (*onPacketLinkStatistics)(crsfLinkStatistics_t *ls);
	void (*onPacketGps)(crsf_sensor_gps_t *gpsSensor);
	void (*onPacketFlightMode)(enum eFlightMode mode);
	void (*onPacketIn)();
};


extern struct CrsfSerialEvent2_s CrsfSerialEvent2;

#endif /* RADIO_CRSF2_H_ */
