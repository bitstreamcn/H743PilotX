/*
 * crsf2.c
 *
 *  Created on: Jul 26, 2025
 *      Author: bitstream
 */
#include "crsf2.h"
#include "main.h"
#include <stdbool.h>
#include "crsf_protocol2.h"
#include "crc8.h"
#include "radio.h"
#include <string.h>
#include "common.h"
#include "log.h"

    // Packet timeout where buffer is flushed if no data is received in this time
    static const unsigned int CRSF_PACKET_TIMEOUT_MS = 100;
    static const unsigned int CRSF_FAILSAFE_STAGE1_MS = 300;

    static uint8_t _rxBuf[CRSF_MAX_PACKET_LEN+3];
    static uint8_t _rxBufPos;

    static crsf_channels_t _channelsPacked;
    static crsfLinkStatistics_t _linkStatistics;
    static crsf_sensor_gps_t _gpsSensor;
    static crsf_sensor_vario_t _varioSensor;
    static crsf_sensor_baro_altitude_t _baroAltitudeSensor;
    static crsf_sensor_attitude_t _attitudeSensor;
    //static uint32_t _baud;
    static uint32_t _lastReceive;
    static uint32_t _lastChannelsPacket;
    static bool _linkIsUp;
    static int _channels[CRSF_NUM_CHANNELS];
    static int _channels_send[CRSF_NUM_CHANNELS];

    struct CrsfSerialEvent2_s CrsfSerialEvent2;

    static void handleByteReceived();
    static void shiftRxBuffer(uint8_t cnt);
    static void processPacketIn(uint8_t len);
    static void checkPacketTimeout();
    static void checkLinkDown();

    // Packet RX Handlers
    static void packetChannelsPacked(const crsf_header_t *p);
    static void packetLinkStatistics(const crsf_header_t *p);
    static void packetGps(const crsf_header_t *p);
    static void packetVario(const crsf_header_t *p);
    static void packetBaroAltitude(const crsf_header_t *p);
    //static void packetAttitude(const crsf_header_t *p);
    static void packetFlightMode(const crsf_header_t *p);

    int crsf2_getChannel(unsigned int ch)  { return _channels[ch - 1]; }
    void crsf2_setChannel(unsigned int ch, uint16_t val){_channels_send[ch -1] = val;}
    const crsf_channels_t *crsf2_getChannelsPacked()  { return &_channelsPacked;}
    const crsfLinkStatistics_t *crsf2_getLinkStatistics()  { return &_linkStatistics; }
    const crsf_sensor_gps_t *crsf2_getGpsSensor()  { return &_gpsSensor; }
    const crsf_sensor_vario_t *crsf2_getVarioSensor()  { return &_varioSensor; }
    const crsf_sensor_baro_altitude_t *crsf2_getBaroAltitudeSensor()  { return &_baroAltitudeSensor; }
    const crsf_sensor_attitude_t *crsf2_getAttitudeSensor()  { return &_attitudeSensor; }
    bool crsf2_isLinkUp()  { return _linkIsUp; }

    long crsf_map(long x, long in_min, long in_max, long out_min, long out_max) {
    	if (x < in_min)
    	{
    		x = in_min;
    	}
    	if (x > in_max)
    	{
    		x = in_max;
    	}
        // 计算比例因子（避免整数除法截断）
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }


    void crsf2_begin()
    {
    	_lastReceive = 0;
    	_lastChannelsPacket = 0;
    	_linkIsUp = (false);
    	crc8_init(0xd5);
    }



    void crsf2_update(){
        checkPacketTimeout();
        checkLinkDown();
    }

    void crsf2_handleSerialIn(const uint8_t *buf, size_t len)
    {
        while (len > 0)
        {
            uint8_t b = *buf;
            buf++;
            _lastReceive = millis();

            _rxBuf[_rxBufPos++] = b;
            handleByteReceived();

            if (_rxBufPos == (sizeof(_rxBuf)/sizeof(_rxBuf[0])))
            {
                // Packet buffer filled and no valid packet found, dump the whole thing
                _rxBufPos = 0;
            }
            len--;
        }
    }

    static void handleByteReceived()
    {
        bool reprocess;
        do
        {
            reprocess = false;
            if (_rxBufPos > 1)
            {
                uint8_t len = _rxBuf[1];
                // Sanity check the declared length, can't be shorter than Type, X, CRC
                if (len < 3 || len > CRSF_MAX_PACKET_LEN)
                {
                    shiftRxBuffer(1);
                    reprocess = true;
                }

                else if (_rxBufPos >= (len + 2))
                {
                    uint8_t inCrc = _rxBuf[2 + len - 1];
                    uint8_t crc = crc8_calc(&_rxBuf[2], len - 1);
                    if (crc == inCrc)
                    {
                        processPacketIn(len);
                        shiftRxBuffer(len + 2);
                        reprocess = true;
                    }
                    else
                    {
                        shiftRxBuffer(1);
                        reprocess = true;
                    }
                }  // if complete packet
            } // if pos > 1
        } while (reprocess);
    }


    static void checkPacketTimeout()
    {
        // If we haven't received data in a long time, flush the buffer a byte at a time (to trigger shiftyByte)
        if (_rxBufPos > 0 && millis() - _lastReceive > CRSF_PACKET_TIMEOUT_MS)
            while (_rxBufPos)
                shiftRxBuffer(1);
    }

    static void checkLinkDown()
    {
        if (_linkIsUp && millis() - _lastChannelsPacket > CRSF_FAILSAFE_STAGE1_MS)
        {
            _linkIsUp = false;
        }
    }


    static void processPacketIn(uint8_t len)
    {
        const crsf_header_t *hdr = (crsf_header_t *)_rxBuf;
        if (hdr->device_addr == CRSF_ADDRESS_FLIGHT_CONTROLLER) //Rx to FC
        {
            switch (hdr->type)
            {
            case CRSF_FRAMETYPE_GPS:
                packetGps(hdr);
                break;
            case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                packetChannelsPacked(hdr);
                break;
            case CRSF_FRAMETYPE_LINK_STATISTICS:
                packetLinkStatistics(hdr);
                break;
            case CRSF_FRAMETYPE_BARO_ALTITUDE:
                packetBaroAltitude(hdr);
                break;
            case CRSF_FRAMETYPE_VARIO:
                packetVario(hdr);
                break;
            case CRSF_FRAMETYPE_FLIGHT_MODE:
            	packetFlightMode(hdr);
            	break;
            }
        }
        else if (hdr->device_addr == CRSF_ADDRESS_CRSF_TRANSMITTER) //Headset to TX
        {
            if (hdr->type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
            {
                packetChannelsPacked(hdr);
            }
            else if (hdr->type == CRSF_FRAMETYPE_FLIGHT_MODE)
            {
            	packetFlightMode(hdr);
            }
        }
        else
        {
        	if (hdr->type == CRSF_FRAMETYPE_FLIGHT_MODE)
			{
				packetFlightMode(hdr);
			}
        }
        if (CrsfSerialEvent2.onPacketIn)
        	CrsfSerialEvent2.onPacketIn();
    }

    // Shift the bytes in the RxBuf down by cnt bytes
    static void shiftRxBuffer(uint8_t cnt)
    {
        // If removing the whole thing, just set pos to 0
        if (cnt >= _rxBufPos)
        {
            _rxBufPos = 0;
            return;
        }

        // Otherwise do the slow shift down
        uint8_t *src = &_rxBuf[cnt];
        uint8_t *dst = &_rxBuf[0];
        _rxBufPos -= cnt;
        uint8_t left = _rxBufPos;
        while (left--)
            *dst++ = *src++;
    }

    static void packetChannelsPacked(const crsf_header_t *p)
    {
        crsf_channels_t *ch = (crsf_channels_t *)&p->data;

        static int _chs[CRSF_NUM_CHANNELS];
        _chs[0] = ch->ch0;
        _chs[1] = ch->ch1;
        _chs[2] = ch->ch2;
        _chs[3] = ch->ch3;
        _chs[4] = ch->ch4;
        _chs[5] = ch->ch5;
        _chs[6] = ch->ch6;
        _chs[7] = ch->ch7;
        _chs[8] = ch->ch8;
        _chs[9] = ch->ch9;
        _chs[10] = ch->ch10;
        _chs[11] = ch->ch11;
        _chs[12] = ch->ch12;
        _chs[13] = ch->ch13;
        _chs[14] = ch->ch14;
        _chs[15] = ch->ch15;

        //for (unsigned int i=0; i<CRSF_NUM_CHANNELS; ++i)
        //    _channels[i] = (int)map(_channels[i], CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000);
        _linkIsUp = true;
        _lastChannelsPacket = millis();

        unsigned validate = 0;
        for (unsigned int i=0; i<CRSF_NUM_CHANNELS; ++i)
        {
        	validate += _chs[i];
        }
        if (validate != ch->validate)
        {
        	LogE("validate != ch->validate");
        	return;
        }
        memcpy(_channels, _chs, sizeof(_chs));
        memcpy(&_channelsPacked, ch, sizeof(_channelsPacked));

        if (CrsfSerialEvent2.onPacketChannels)
        	CrsfSerialEvent2.onPacketChannels();
    }

    static void packetLinkStatistics(const crsf_header_t *p)
    {
        const crsfLinkStatistics_t *link = (crsfLinkStatistics_t *)p->data;
        memcpy(&_linkStatistics, link, sizeof(_linkStatistics));
    }

    static void packetGps(const crsf_header_t *p)
    {
        const crsf_sensor_gps_t *gps = (crsf_sensor_gps_t *)p->data;
        _gpsSensor.latitude = be32toh(gps->latitude);
        _gpsSensor.longitude = be32toh(gps->longitude);
        _gpsSensor.groundspeed = be16toh(gps->groundspeed);
        _gpsSensor.heading = be16toh(gps->heading);
        _gpsSensor.altitude = be16toh(gps->altitude);
        _gpsSensor.satellites = gps->satellites;
    }

    static void packetVario(const crsf_header_t *p)
    {
        const crsf_sensor_vario_t *vario = (crsf_sensor_vario_t *)p->data;
        _varioSensor.verticalspd = be16toh(vario->verticalspd);
    }

    static void packetFlightMode(const crsf_header_t *p)
    {
        if (CrsfSerialEvent2.onPacketFlightMode)
        	CrsfSerialEvent2.onPacketFlightMode((enum eFlightMode)p->data[0]);
    }

    static void packetBaroAltitude(const crsf_header_t *p)
    {
        const crsf_sensor_baro_altitude_t *baroAltitude = (crsf_sensor_baro_altitude_t *)p->data;
        _baroAltitudeSensor.altitude = be16toh(baroAltitude->altitude);
        _baroAltitudeSensor.verticalspd = be16toh(baroAltitude->verticalspd);
    }

    /*
    static void packetAttitude(const crsf_header_t *p)
    {
        const crsf_sensor_attitude_t *attitude = (crsf_sensor_attitude_t *)p->data;
        _attitudeSensor.pitch = be16toh(attitude->pitch);
        _attitudeSensor.roll = be16toh(attitude->roll);
        _attitudeSensor.yaw = be16toh(attitude->yaw);
    }
    */

    void crsf2_write(uint8_t b)
    {
    	crsf2_write2(&b, 1);
    }

    void crsf2_write2(const uint8_t *buf, size_t len)
    {
    	Radio.Send((uint8_t *)buf, (uint8_t)len);
    }

    void crsf2_queuePacket(uint8_t addr, uint8_t type, const void *payload, uint8_t len)
    {
    	/*
        if (!_linkIsUp)
            return;
        */
        if (len > CRSF_MAX_PACKET_LEN)
            return;

        uint8_t buf[CRSF_MAX_PACKET_LEN+4];
        buf[0] = addr;
        buf[1] = len + 2; // type + payload + crc
        buf[2] = type;
        memcpy(&buf[3], payload, len);
        buf[len+3] = crc8_calc(&buf[2], len + 1);
        crsf2_write2(buf, len + 4);
    }

    void crsf2_writePacket(uint8_t addr, uint8_t type, const void *payload, uint8_t len)
    {
        uint8_t buf[CRSF_MAX_PACKET_LEN+4];
        buf[0] = addr;
        buf[1] = len + 2; // type + payload + crc
        buf[2] = type;
        memcpy(&buf[3], payload, len);
        buf[len+3] = crc8_calc(&buf[2], len + 1);
        crsf2_write2(buf, len + 4);
    }

    void crsf2_sendChannels()
    {
	  crsf_channels_t crsfChannels = { 0 };
	  crsfChannels.ch0 = _channels_send[0];//(unsigned)map(_channels_send[0], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	  crsfChannels.ch1 = _channels_send[1];//(unsigned)map(_channels_send[1], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	  crsfChannels.ch2 = _channels_send[2];//(unsigned)map(_channels_send[2], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	  crsfChannels.ch3 = _channels_send[3];//(unsigned)map(_channels_send[3], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	  crsfChannels.ch4 = _channels_send[4];//(unsigned)map(_channels_send[4], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	  crsfChannels.ch5 = _channels_send[5];//(unsigned)map(_channels_send[5], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	  crsfChannels.ch6 = _channels_send[6];//(unsigned)map(_channels_send[6], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	  crsfChannels.ch7 = _channels_send[7];//(unsigned)map(_channels_send[7], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	  crsfChannels.ch8 = _channels_send[8];//(unsigned)map(_channels_send[8], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	  crsfChannels.ch9 = _channels_send[9];//(unsigned)map(_channels_send[9], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	  crsfChannels.ch10 = _channels_send[10];//(unsigned)map(_channels_send[10], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	  crsfChannels.ch11 = _channels_send[11];//(unsigned)map(_channels_send[11], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	  crsfChannels.ch12 = _channels_send[12];//(unsigned)map(_channels_send[12], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	  crsfChannels.ch13 = _channels_send[13];//(unsigned)map(_channels_send[13], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	  crsfChannels.ch14 = _channels_send[14];//(unsigned)map(_channels_send[14], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	  crsfChannels.ch15 = _channels_send[15];//(unsigned)map(_channels_send[15], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);

	  crsfChannels.validate = 0;
      for (unsigned int i=0; i<CRSF_NUM_CHANNELS; ++i)
      {
    	  crsfChannels.validate += _channels_send[i];
      }
      crsf2_writePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, &crsfChannels, sizeof(crsfChannels));
    }
