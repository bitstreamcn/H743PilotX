/*
 * crc2.c
 *
 *  Created on: Jul 26, 2025
 *      Author: bitstream
 */

#include "crc2.h"
#include <stdlib.h>

static uint8_t crc_8_dvb_s2_table[256];

void crc2_init()
{
    for (uint16_t i = 0; i < 256; i++)
    {
        uint8_t crc = i;
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0xd5;
            }
            else
            {
                crc <<= 1;
            }
        }

        crc_8_dvb_s2_table[i] = crc & 0xff;
    }
}


uint8_t crc2_calculate(uint8_t start, const uint8_t *data, uint8_t length)
{
    uint8_t crc = crc_8_dvb_s2_table[0 ^ start];

    for (uint8_t i = 0; i < length; i++)
    {
        crc = crc_8_dvb_s2_table[crc ^ data[i]];
    }

    return crc;
}

uint8_t crc2_calculate2(uint8_t offset, uint8_t start, const uint8_t *data, uint8_t length)
{
	(void)start;
    uint8_t crc = crc_8_dvb_s2_table[0 ^ data[offset]];

    for (uint8_t i = offset + 1; i < length; i++)
    {
        crc = crc_8_dvb_s2_table[crc ^ data[i]];
    }

    return crc;
}


