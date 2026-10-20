/*
 * BL24C256A.c
 *
 *  Created on: Jul 6, 2025
 *      Author: bitstream
 */

#include "BL24C256A.h"
#include "main.h"
#include "i2c.h"
#include "common.h"
#include "log.h"

#define EEPROM_ADDR (0x50)

#define EEPROM_MAX_DELAY 200u
#define EEPROM_TRY_TIMES 10


//I2C读写大量数据时会因为RTOS的线程切换导致时序出错，引发I2C总线死锁，有条件尽量使用DMA

//手册上是3ms
void wait_for_eeprom_ready()
{
    // not sure if ACK polling is even posiible with pico sdk,
    // so we'll just wait a magic amout of 10ms
    HAL_Delay(10);
}

bool eeprom_init()
{
	return true;
}

bool eeprom_read(uint16_t address, uint8_t* buffer, uint16_t size)
{
	uint16_t w;
	while (1)
	{
		w = EEPROM_PAGE_SIZE - (address  % EEPROM_PAGE_SIZE);
		if (w > size)
		{
		  w = size;
		}
		bool succ = false;
		for (int n=0; n<EEPROM_TRY_TIMES && !succ; n++)
		{
			HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
					&hi2c2,
					EEPROM_ADDR<<1,
					address,
					I2C_MEMADD_SIZE_16BIT,
					buffer,
					w,
					EEPROM_MAX_DELAY);
			wait_for_eeprom_ready();
			if (status == HAL_OK)
			{
				succ = true;
			}
			else
			{
				LogE("eeprom_read error: %x", status);
			}
		}
		if (!succ)
		{
			LogE("eeprom_read false");
			return false;
		}
		size -= w;
		buffer += w;
		address += w;
		if (size == 0)
		{
			return true;
		}
	}
}

volatile int write_status = 0;

void eeprom_write_succ_callback()
{
	write_status = 1;
}

void eeprom_write_fail_callback()
{
	write_status = 2;
}

bool eeprom_write(uint16_t address, uint8_t* buffer, uint16_t size)
{
	uint16_t w;
	while (1)
	{
		w = EEPROM_PAGE_SIZE - (address  % EEPROM_PAGE_SIZE);
		if (w > size)
		{
			w = size;
		}
		bool succ = false;
		for (int n=0; n<EEPROM_TRY_TIMES && !succ; n++)
		{
			write_status = 0;
			HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
					&hi2c2,
					EEPROM_ADDR<<1,
					address,
					I2C_MEMADD_SIZE_16BIT,
					buffer,
					w,
					EEPROM_MAX_DELAY);
			if (status == HAL_OK)
			{
				//if (write_status == 1)
				{
					succ = true;
				}
			}
			else
			{
				LogE("eeprom_write error: %x", status);
			}
			wait_for_eeprom_ready();
		}
		if (!succ)
		{
			LogE("eeprom_write false");
			return false;
		}
		size -= w;
		buffer += w;
		address += w;
		if (size == 0)
		{
			return true;
		}
	}


}

uint8_t eeprom_read_byte(uint16_t address)
{
	bool succ = false;
	uint8_t byte = 0xff;
	for (int n=0; n<EEPROM_TRY_TIMES && !succ; n++)
	{
		HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
				&hi2c2,
				EEPROM_ADDR<<1,
				address,
				I2C_MEMADD_SIZE_16BIT,
				&byte,
				1,
				EEPROM_MAX_DELAY);
		if (status == HAL_OK)
		{
			succ = true;
		}
		else
		{
			LogE("eeprom_read_byte error: %x", status);
		}
		wait_for_eeprom_ready();
	}
	return byte;
}

bool eeprom_write_byte(uint16_t address, uint8_t byte)
{
	bool succ = false;
	for (int n=0; n<EEPROM_TRY_TIMES && !succ; n++)
	{
		HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c2, EEPROM_ADDR<<1, address,
				I2C_MEMADD_SIZE_16BIT, &byte, 1, EEPROM_MAX_DELAY);
		if (status == HAL_OK)
		{
			succ = true;
		}
		else
		{
			LogE("eeprom_write_byte error: %x", status);
		}
		wait_for_eeprom_ready();
	}
	return succ;
}

