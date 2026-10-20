/*
 * imu_ex.cpp
 *
 *  Created on: May 11, 2026
 *      Author: bitstream
 */

#include "imu_ex.h"
#include "log.h"
#include "Config.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"

#ifdef __cplusplus
}
#endif

ImuEx imuex;

bool ImuEx::Init()
{
	while (1)
	{
		uint8_t validate = 0x22;
		HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c3, 0x30 << 1, 3, I2C_MEMADD_SIZE_8BIT, &validate, 1, 2);
		if (status != HAL_OK)
		{
			Log::E("ImuEx HAL_I2C_Mem_Write Error: %x", status);
		}
		else
		{
			uint8_t buffer[1];
			status = HAL_I2C_Mem_Read(&hi2c3, 0x30 << 1, 3, I2C_MEMADD_SIZE_8BIT, buffer, 1, 2);
			if (status != HAL_OK)
			{
				Log::E("ImuEx HAL_I2C_Mem_Read Error: %x", status);
			}
			else
			{
				if (buffer[0] != validate)
				{
					Log::E("ImuEx validate Error: %x %x", buffer[0], validate);
				}
				else
				{
					Log::I("ImuEx Init OK");
					WhoAmI();
					return true;
				}
			}
		}
		HAL_Delay(100);
	}
	return false;
}

bool ImuEx::Update()
{
	struct __attribute__((packed)){
			float ax, ay, az, gx, gy, gz, roll, pitch, yaw, alt, vz;
			float vx, vy;
			float validate;
	}imu_buffer;

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c3, 0x30 << 1, 1, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&imu_buffer, sizeof(imu_buffer), 3);
	if (status != HAL_OK)
	{
		if (status == HAL_ERROR)
		{
		    HAL_I2C_DeInit(&hi2c3);
		    HAL_I2C_Init(&hi2c3);
		}
		Log::E("ImuEx Update Error: %x", status);
		return false;
	}
	float validate =
			imu_buffer.ax + imu_buffer.ay + imu_buffer.az +
			imu_buffer.gx + imu_buffer.gy + imu_buffer.gz +
			imu_buffer.roll + imu_buffer.pitch + imu_buffer.yaw +
			imu_buffer.alt + imu_buffer.vz + imu_buffer.vx + imu_buffer.vy;
	if (imu_buffer.validate != validate)
	{
		Log::E("ImuEx validate Error: %f != %f", imu_buffer.validate, validate);
		return false;
	}
	//检查异常突变
	if (fabsf(this->roll - imu_buffer.roll) > 45.0)
	{
		Log::W("ImuEx roll: %f -> %f", this->roll, imu_buffer.roll);
	}
	if (fabsf(this->pitch - imu_buffer.pitch) > 45.0)
	{
		Log::W("ImuEx pitch: %f -> %f", this->pitch, imu_buffer.pitch);
	}
	if ((180 - (fabsf(fabsf(this->yaw - imu_buffer.yaw) - 180))) > 45.0)
	{
		Log::W("ImuEx yaw: %f -> %f", this->yaw, imu_buffer.yaw);
	}
	if (fabsf(gx - imu_buffer.gx) > 150.0)
	{
		Log::W("ImuEx gx: %f -> %f", gx, imu_buffer.gx);
	}
	if (fabsf(gy - imu_buffer.gy) > 150.0)
	{
		Log::W("ImuEx gy: %f -> %f", gy, imu_buffer.gy);
	}
	if (fabsf(gz - imu_buffer.gz) > 150.0)
	{
		Log::W("ImuEx gz: %f -> %f", gz, imu_buffer.gz);
	}

	float _ax = imu_buffer.ax;
	float _ay = imu_buffer.ay;
	float _az = imu_buffer.az;
	ax_lpf.alpha = cfg.param.acc_x_alpha;
	ay_lpf.alpha = cfg.param.acc_y_alpha;
	az_lpf.alpha = cfg.param.acc_z_alpha;
	ax = lpf_update(&ax_lpf, _ax);
	ay = lpf_update(&ay_lpf, _ay);
	az = lpf_update(&az_lpf, _az);
	float _gx = imu_buffer.gx;
	float _gy = imu_buffer.gy;
	float _gz = imu_buffer.gz;
	gx = lpf_update(&gx_lpf, _gx);
	gy = lpf_update(&gy_lpf, _gy);
	gz = lpf_update(&gz_lpf, _gz);
	this->roll = imu_buffer.roll;
	this->pitch = imu_buffer.pitch;
	this->yaw = imu_buffer.yaw;
	alt = imu_buffer.alt;
	vz = imu_buffer.vz;
	vx = imu_buffer.vx;
	vy = imu_buffer.vy;
	return true;
}

void ImuEx::WhoAmI()
{
	uint8_t buffer[1];
	if (HAL_I2C_Mem_Read(&hi2c3, 0x30 << 1, 2, I2C_MEMADD_SIZE_8BIT, buffer, 1, 2) != HAL_OK)
	{
		Log::E("ImuEx WhoAmI HAL_I2C_Mem_Read Error");
	}
	else
	{
		Log::I("ImuEx IS: %x", buffer[0]);
	}
}


extern "C" {
float GetAlt()
{
	return imuex.alt;
}
}

