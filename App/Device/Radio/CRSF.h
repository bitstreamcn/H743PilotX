/*
 * CRSF.h
 *
 *  Created on: Jun 11, 2025
 *      Author: bitstream
 */

#ifndef RADIO_CRSF_H_
#define RADIO_CRSF_H_

#include <stdint.h>
#include "crc8.h"
#include "crsf_protocol.h"
#include "radio.h"


#ifdef __cplusplus
extern "C" {  // C++编译器进入C模式
#endif




enum eFailsafeAction { fsaNoPulses, fsaHold };


    void CrsfInit(uint32_t baud);
    void CrsfSerial_begin(uint32_t baud);
    void CrsfSerial_loop();
    void CrsfSerial_dataIn(int8_t *payload, uint16_t size);
    //void write(uint8_t b);
    void CrsfSerial_write(const uint8_t *buf, uint8_t len);
    bool CrsfSerial_queuePacket(uint8_t type, const void *payload, uint8_t len);
    void CrsfSerial_queuePacketChannels();

    uint32_t CrsfSerial_getBaud();
    // Return current channel value (1-based) in us
    int CrsfSerial_getChannel(unsigned int ch);
    void CrsfSerial_setChannel(unsigned int ch, unsigned int value_us);
    const crsfLinkStatistics_t *getLinkStatistics();
    const crsf_sensor_gps_t *getGpsSensor();
    bool CrsfSerial_isLinkUp();
    bool CrsfSerial_getPassthroughMode();
    void CrsfSerial_setPassthroughMode(bool val, uint32_t passthroughBaud);

    void mixer(unsigned int ch, int step);


    struct CrsfSerialEvent_s
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
		void (*onPacketFlightMode)(const char *mode);
		void (*onPacketIn)();
    };


extern struct CrsfSerialEvent_s CrsfSerialEvent;



#ifdef __cplusplus
}
#endif


#endif /* RADIO_CRSF_H_ */
