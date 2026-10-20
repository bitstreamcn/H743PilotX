/*
 * CRSF.c
 *
 *  Created on: Jun 11, 2025
 *      Author: bitstream
 */

#include "CRSF.h"
#include <string.h>
#include "radio.h"
#include "common.h"

#ifdef __cplusplus
extern "C" {  // C++编译器进入C模式
#endif


// Packet timeout where buffer is flushed if no data is received in this time
static const unsigned int CRSF_PACKET_TIMEOUT_MS = 100;
static const unsigned int CRSF_FAILSAFE_STAGE1_MS = 300;

uint8_t _rxBuf[CRSF_MAX_PACKET_SIZE];
uint8_t _rxBufPos;

crsfLinkStatistics_t _linkStatistics;
crsf_sensor_gps_t _gpsSensor;
uint32_t _baud;
uint32_t _lastReceive;
uint32_t _lastChannelsPacket;
bool _linkIsUp;
uint32_t _passthroughBaud;
int _channels[CRSF_NUM_CHANNELS];

void handleSerialIn(int8_t *payload, uint16_t size);
void handleByteReceived();
void shiftRxBuffer(uint8_t cnt);
void processPacketIn(uint8_t len);
void checkPacketTimeout();
void checkLinkDown();

// Packet Handlers
void packetChannelsPacked(const crsf_header_t *p);
void packetLinkStatistics(const crsf_header_t *p);
void packetGps(const crsf_header_t *p);
void packetFlightMode(const crsf_header_t *p);


struct CrsfSerialEvent_s CrsfSerialEvent = {
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
};

bool getPassthroughMode() { return _passthroughBaud != 0; }
void setPassthroughMode(bool val, uint32_t passthroughBaud);

void CrsfInit(uint32_t baud)
{
	crc8_init(0xd5);
	_baud = baud;
	_lastReceive = 0;
	_lastChannelsPacket = 0;
	_linkIsUp= false;
	_passthroughBaud = 0;
}

void CrsfSerial_begin(uint32_t baud)
{
	/*
    if (baud != 0)
        _port.begin(baud);
    else
        _port.begin(_baud);
    */
}

uint32_t CrsfSerial_getBaud()
{
	 return _baud;
}

int CrsfSerial_getChannel(unsigned int ch)  { return _channels[ch - 1]; }
void CrsfSerial_setChannel(unsigned int ch, unsigned int value_us) { _channels[ch - 1] = value_us; }
const crsfLinkStatistics_t *getLinkStatistics()  { return &_linkStatistics; }
const crsf_sensor_gps_t *getGpsSensor()  { return &_gpsSensor; }
bool CrsfSerial_isLinkUp()  { return _linkIsUp; }
bool CrsfSerial_getPassthroughMode()  { return _passthroughBaud != 0; }

void mixer(unsigned int ch, int step)
{
    unsigned val = CrsfSerial_getChannel(ch) + step;
    if (val > 2012)
        val = 988;
    CrsfSerial_setChannel(ch, val);
}

void CrsfSerial_loop()
{
    checkPacketTimeout();
    checkLinkDown();
}

// Call from main loop to update
void CrsfSerial_dataIn(int8_t *payload, uint16_t size)
{
    handleSerialIn(payload, size);
}


void handleSerialIn(int8_t *payload, uint16_t size)
{
    //while (_port.available())
	for (int i=0; i<size; i++)
    {
        uint8_t b = payload[i];//_port.read();
        _lastReceive = millis();

        if (getPassthroughMode())
        {
            if (CrsfSerialEvent.onOobData)
            	CrsfSerialEvent.onOobData(b);
            continue;
        }

        _rxBuf[_rxBufPos++] = b;
        handleByteReceived();

        if (_rxBufPos == (sizeof(_rxBuf)/sizeof(_rxBuf[0])))
        {
            // Packet buffer filled and no valid packet found, dump the whole thing
            _rxBufPos = 0;
        }
    }
}

void handleByteReceived()
{
    bool reprocess;
    do
    {
        reprocess = false;
        if (_rxBufPos > 1)
        {
            uint8_t len = _rxBuf[1];
            // Sanity check the declared length isn't outside Type + X{1,CRSF_MAX_PAYLOAD_LEN} + CRC
            // assumes there never will be a CRSF message that just has a type and no data (X)
            if (len < 3 || len > (CRSF_MAX_PAYLOAD_LEN + 2))
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

void checkPacketTimeout()
{
    // If we haven't received data in a long time, flush the buffer a byte at a time (to trigger shiftyByte)
    if (_rxBufPos > 0 && millis() - _lastReceive > CRSF_PACKET_TIMEOUT_MS)
        while (_rxBufPos)
            shiftRxBuffer(1);
}

void checkLinkDown()
{
    if (_linkIsUp && millis() - _lastChannelsPacket > CRSF_FAILSAFE_STAGE1_MS)
    {
        if (CrsfSerialEvent.onLinkDown)
        	CrsfSerialEvent.onLinkDown();
        _linkIsUp = false;
    }
}

void processPacketIn(uint8_t len)
{
    const crsf_header_t *hdr = (crsf_header_t *)_rxBuf;
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
    case CRSF_FRAMETYPE_FLIGHT_MODE:
    	packetFlightMode(hdr);
    	break;
    }
    if (CrsfSerialEvent.onPacketIn)
    	CrsfSerialEvent.onPacketIn();
}

// Shift the bytes in the RxBuf down by cnt bytes
void shiftRxBuffer(uint8_t cnt)
{
    // If removing the whole thing, just set pos to 0
    if (cnt >= _rxBufPos)
    {
        _rxBufPos = 0;
        return;
    }

    if (cnt == 1 && CrsfSerialEvent.onOobData)
    	CrsfSerialEvent.onOobData(_rxBuf[0]);

    // Otherwise do the slow shift down
    uint8_t *src = &_rxBuf[cnt];
    uint8_t *dst = &_rxBuf[0];
    _rxBufPos -= cnt;
    uint8_t left = _rxBufPos;
    while (left--)
        *dst++ = *src++;
}

void packetChannelsPacked(const crsf_header_t *p)
{
    // Unpack CRSF channel data stored bytewise 11 bits / channel
    // Code assumes there is enough payload for all the channels
    unsigned inputMask = (1 << CRSF_BITS_PER_CHANNEL) - 1;
    const uint8_t *buf = p->data;
    unsigned scratch = 0;
    unsigned bitsInScratch = 0;
    for (unsigned ch=0; ch<CRSF_NUM_CHANNELS; ++ch)
    {
        while (bitsInScratch < CRSF_BITS_PER_CHANNEL)
        {
            scratch |= (*buf++) << bitsInScratch;
            bitsInScratch += 8;
        }

        _channels[ch] = CRSF_to_US(scratch & inputMask);
        scratch >>= CRSF_BITS_PER_CHANNEL;
        bitsInScratch -= CRSF_BITS_PER_CHANNEL;
    }

    if (!_linkIsUp && CrsfSerialEvent.onLinkUp)
    	CrsfSerialEvent.onLinkUp();
    _linkIsUp = true;
    _lastChannelsPacket = millis();

    if (CrsfSerialEvent.onPacketChannels)
    	CrsfSerialEvent.onPacketChannels();
}

void packetLinkStatistics(const crsf_header_t *p)
{
    const crsfLinkStatistics_t *link = (crsfLinkStatistics_t *)p->data;
    memcpy(&_linkStatistics, link, sizeof(_linkStatistics));

    // This is for the TX, but checkLinkDown() will keep triggering
    // due to no channels coming in, so this is disabled for now
    // because the timeout needs to be a function of packet rate
    // bool linkIsUp = _linkStatistics.uplink_Link_quality != 0;
    // if (linkIsUp != _linkIsUp)
    // {
    //     _linkIsUp = linkIsUp;
    //     if (_linkIsUp && onLinkUp)
    //         onLinkUp();
    //     else if (!_linkIsUp && onLinkDown)
    //         onLinkDown();
    // }

    if (CrsfSerialEvent.onPacketLinkStatistics)
    	CrsfSerialEvent.onPacketLinkStatistics(&_linkStatistics);
}

void packetGps(const crsf_header_t *p)
{
    const crsf_sensor_gps_t *gps = (crsf_sensor_gps_t *)p->data;
    _gpsSensor.latitude = be32toh(gps->latitude);
    _gpsSensor.longitude = be32toh(gps->longitude);
    _gpsSensor.groundspeed = be16toh(gps->groundspeed);
    _gpsSensor.heading = be16toh(gps->heading);
    _gpsSensor.altitude = be16toh(gps->altitude);
    _gpsSensor.satellites = gps->satellites;

    if (CrsfSerialEvent.onPacketGps)
    	CrsfSerialEvent.onPacketGps(&_gpsSensor);
}

void packetFlightMode(const crsf_header_t *p)
{
    if (CrsfSerialEvent.onPacketFlightMode)
    	CrsfSerialEvent.onPacketFlightMode((const char*)p->data);
}

//void CrsfSerial_write(uint8_t b)
//{
    //_port.write(b);
//}

void CrsfSerial_write(const uint8_t *buf, uint8_t len)
{
    //_port.write(buf, len);
	Radio.Send((uint8_t *)buf, (uint8_t)len);
}

bool CrsfSerial_queuePacket(uint8_t type, const void *payload, uint8_t len)
{
    if (CrsfSerial_getPassthroughMode())
        return false;
    if (len > CRSF_MAX_PAYLOAD_LEN)
        return false;

    uint8_t buf[CRSF_MAX_PACKET_SIZE];
    buf[0] = CRSF_SYNC_BYTE;
    buf[1] = len + 2; // type + payload + crc
    buf[2] = type;
    memcpy(&buf[3], payload, len);
    buf[len+3] = crc8_calc(&buf[2], len + 1);

    CrsfSerial_write(buf, len + 4);
    return true;
}

/***
 * @brief: Write 16 channels out as a handset would
 * @details:    TX ONLY! Packs the us channel data which has been set with setChannel()
 *              and writes it to the connected CRSF transmitter module.
 *              External CRSF transmitters are usually half duplex inverted
 *              and also scheduled by the module sending timing data, this
 *              code handles none of that. This will, however, get a
 *              transmitter to start transmitting channels.
 */
void CrsfSerial_queuePacketChannels()
{
   // 11 bits per channel * 16 channels = 176 bits = 22 bytes
    uint8_t packedChannels[(CRSF_NUM_CHANNELS * CRSF_BITS_PER_CHANNEL + 7) / 8];
    uint8_t *pbuf = packedChannels;

    // Pack 11 bit channels low byte first into packedChannels
    uint32_t scratch = 0;
    uint32_t bitsInScratch = 0;
    for (unsigned ch=0; ch<CRSF_NUM_CHANNELS; ++ch)
    {
        uint32_t crsfVal = US_to_CRSF(_channels[ch]);
        scratch |= crsfVal << bitsInScratch;
        bitsInScratch += CRSF_BITS_PER_CHANNEL;
        while (bitsInScratch > 8)
        {
            *pbuf++ = scratch;
            scratch >>= 8;
            bitsInScratch -= 8;
        }
    }

    CrsfSerial_queuePacket(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, packedChannels, sizeof(packedChannels));
}

/**
 * @brief   Enter passthrough mode (serial sent directly to shiftybyte),
 *          optionally changing the baud rate used during passthrough mode
 * @param val
 *          True to start passthrough mode, false to resume processing CRSF
 * @param passthroughBaud
 *          New baud rate for passthrough mode, or 0 to not change baud
 *          Not used if disabling passthough
*/
void setPassthroughMode(bool val, uint32_t passthroughBaud)
{
    if (val)
    {
        // If not requesting any baud change
        if (passthroughBaud == 0)
        {
            // Enter passthrough mode if not yet
            if (_passthroughBaud == 0)
                _passthroughBaud = _baud;
            return;
        }

        _passthroughBaud = passthroughBaud;
    }
    else
    {
        // Not in passthrough, can't leave it any harder than we already are
        if (_passthroughBaud == 0)
            return;

        // Leaving passthrough, but going back to same baud, just disable
        if (_passthroughBaud == _baud)
        {
            _passthroughBaud = 0;
            return;
        }

        _passthroughBaud = 0;
    }

    // Can only get here if baud is changing, close and reopen the port
    //_port.end(); // assumes flush()
    CrsfSerial_begin(_passthroughBaud);
}


#ifdef __cplusplus
}
#endif






