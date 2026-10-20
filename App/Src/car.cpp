/*
 * car.cpp
 *
 *  Created on: 2026年9月4日
 *      Author: bitstream
 */
#include "main.h"
#include "car.h"
#include <stdio.h>
#include "log.h"
#include "i2c.h"
#include "common.h"

extern volatile bool IsConnect;

void Car::Loop()
{
    static uint32_t connect_tim = 0;
    if (!IsConnect)
    {
    	if ((HAL_GetTick() - connect_tim) > 2000)
    	{
    		rx_data.beep = 1;
    	}
    }
    else
    {
    	rx_data.beep = 0;
    	connect_tim = HAL_GetTick();
    }
	static bool read = false;
	if (!read)
	{
		//rx_data.beep = 1;
		//rx_data.led = 1;
		//rx_data.motor = -20;
		//rx_data.servo = 80;
		rx_data.validate = (rx_data.beep + rx_data.led + rx_data.motor + rx_data.servo) & 0xff;
		HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, (0x32<<1), (uint8_t *)&rx_data, sizeof(rx_data), 2);
		if (status != HAL_OK)
		{
			PERIODIC_LOGE(2000, "Car HAL_I2C_Master_Transmit Error: %x", status);
		}
	}
	else
	{
		HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&hi2c1, (0x32<<1), (uint8_t *)&tx_data, sizeof(tx_data), 2);
		uint8_t v = (tx_data.beep + tx_data.led + tx_data.voltage) & 0xff;
		if (status != HAL_OK)
		{
			PERIODIC_LOGE(2000, "Car HAL_I2C_Master_Transmit Error: %x", status);
		}
		if (v == tx_data.validate && tx_data.voltage > 0)
		{
			voltage = tx_data.voltage / 10.0f;
			if (voltage < 14.4)
			{
				PERIODIC_LOGW(5000, "bat:%f", voltage);
			}
		}
	}
	read = !read;
}

void Car::setLedOn()
{
	rx_data.led = 1;
}
void Car::setLedOff()
{
	rx_data.led = 0;
}
void Car::setBeepOn()
{
	rx_data.beep = 1;
}
void Car::setBeepOff()
{
	rx_data.beep = 0;
}
void Car::setMotor(int speed)
{
	rx_data.motor = speed;
}
void Car::setServo(int value)
{
	rx_data.servo = value;
}



