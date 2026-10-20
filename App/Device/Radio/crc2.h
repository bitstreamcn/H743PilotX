/*
 * crc2.h
 *
 *  Created on: Jul 26, 2025
 *      Author: bitstream
 */

#ifndef RADIO_CRC2_H_
#define RADIO_CRC2_H_


#include <stdint.h>


#define CRC_OPTIMISATION_SPEED    0
#define CRC_OPTIMISATION_SIZE     1
#define CRC_OPTIMISATION_HARDWARE 2

void crc2_init();
uint8_t crc2_calculate(uint8_t start, const uint8_t *data, uint8_t length);
uint8_t crc2_calculate2(uint8_t offset, uint8_t start, const uint8_t *data, uint8_t length);


#endif /* RADIO_CRC2_H_ */
