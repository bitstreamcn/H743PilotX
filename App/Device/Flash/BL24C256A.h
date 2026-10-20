/*
 * BL24C256A.h
 *
 *  Created on: Jul 6, 2025
 *      Author: bitstream
 */

#ifndef FLASH_BL24C256A_H_
#define FLASH_BL24C256A_H_
#include <stdint.h>
#include <stdbool.h>

#define EEPROM_PAGE_SIZE 64
#define EEPROM_PAGE_COUNT 4096

bool eeprom_init();
bool eeprom_read(uint16_t address, uint8_t* buffer, uint16_t size);
bool eeprom_write(uint16_t address, uint8_t* buffer, uint16_t size);

uint8_t eeprom_read_byte(uint16_t address);
bool eeprom_write_byte(uint16_t address, uint8_t byte);

void eeprom_write_succ_callback();
void eeprom_write_fail_callback();


#endif /* FLASH_BL24C256A_H_ */
