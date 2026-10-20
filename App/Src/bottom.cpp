/*
 * bottom.cpp
 *
 *  Created on: 2026年7月18日
 *      Author: bitstream
 */


#include "main.h"
#include "bottom.h"
#include "log.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "../Device/Imu/imu.h"
#include "i2c.h"

#ifdef __cplusplus
}
#endif

Bottom bottom;

#define DEG_TO_RAD 0.017453292519943295f

bool Bottom::Init()
{
	//while (1)
	{
		uint8_t validate = 0x22;
		HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, 0x31 << 1, 3, I2C_MEMADD_SIZE_8BIT, &validate, 1, 5);
		if (status != HAL_OK)
		{
			Log::E("bottom HAL_I2C_Mem_Write Error: %x", status);
		}
		else
		{
			HAL_Delay(5);
			uint8_t buffer[1];
			status = HAL_I2C_Mem_Read(&hi2c1, 0x31 << 1, 3, I2C_MEMADD_SIZE_8BIT, buffer, 1, 5);
			if (status != HAL_OK)
			{
				Log::E("bottom HAL_I2C_Mem_Read Error: %x", status);
			}
			else
			{
				if (buffer[0] != validate)
				{
					Log::E("bottom validate Error: %x %x", buffer[0], validate);
				}
				else
				{
					Log::I("bottom Init OK");
					WhoAmI();
					return true;
				}
			}
		}
		HAL_Delay(1000);
	}
	return false;
}

/**
 * @brief 计算倾斜补偿后的垂直高度
 * @param alt_raw: TOF测到的原始斜距 (单位：米)
 * @param pitch_deg: 当前俯仰角 (度, -180~180, 抬头为正)
 * @param roll_deg: 当前横滚角 (度, -180~180, 右倾为正)
 * @retval 真实垂直高度 (单位：米)，返回负数代表角度过大数据无效
 */
float Calculate_Vertical_Alt(float alt_raw, float pitch_deg, float roll_deg)
{
    // 1. 角度转弧度
    float pitch_rad = pitch_deg * DEG_TO_RAD;
    float roll_rad  = roll_deg * DEG_TO_RAD;

    // 2. 计算余弦值
    float cos_pitch = cosf(pitch_rad);
    float cos_roll  = cosf(roll_rad);

    // 3. 安全限制：如果倾斜角度过大（超过60度），TOF 数据不可信
    // 返回负数告诉上层调用者丢弃这帧数据
    if (fabsf(pitch_deg) > 60.0f || fabsf(roll_deg) > 60.0f)
    {
        return -1.0f;
    }

    // 4. 计算垂直高度
    float alt_vertical = alt_raw * cos_pitch * cos_roll;

    return alt_vertical;
}

bool Bottom::Update()
{
	struct __attribute__((packed)){
			int16_t alt;
			int16_t x, y;
			int16_t x1, y1;
			uint16_t quality;
			uint8_t validate;
	}bottom_buffer;

	static uint32_t update_tick = 0;
	uint32_t now = HAL_GetTick();
	uint32_t remain = now - update_tick;
	if (remain > 100)
	{
		alt = 0;
		x = 0;
		y = 0;
		offset_x = 0;
		offset_y = 0;
	}

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, 0x31 << 1, 1, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&bottom_buffer, sizeof(bottom_buffer), 2);
	if (status != HAL_OK)
	{
		if (status == HAL_ERROR)
		{
		    //HAL_I2C_DeInit(&hi2c1);
		    //HAL_I2C_Init(&hi2c1);
		}
		PERIODIC_LOGE(2000, "bottom Update Error: %x", status);
		return false;
	}
	int16_t validate =
			bottom_buffer.alt + bottom_buffer.x + bottom_buffer.y + bottom_buffer.x1 + bottom_buffer.y1 + bottom_buffer.quality;
	if (bottom_buffer.validate != (validate & 0xff))
	{
		Log::E("bottom validate Error: %f != %f", bottom_buffer.validate, validate);
		return false;
	}

	//检查异常突变
	if (this->alt != 0 && bottom_buffer.alt !=0 && fabsf(this->alt - bottom_buffer.alt / 100.0f) > 1.0)
	{
		Log::W("bottom alt: %f -> %f", this->alt, bottom_buffer.alt / 100.0f);
	}
	static float prev_x = 0;
	if (this->x != 0 && bottom_buffer.x !=0 && bottom_buffer.x != prev_x && fabsf(this->x - bottom_buffer.x / 100.0f / 10.0f) > 0.5)
	{
		Log::W("bottom x: %f -> %f", this->x, bottom_buffer.x / 100.0f / 10.0f);
		offset_x = offset_x + (bottom_buffer.x / 100.0f / 10.0f) - x;
	}
	static float prev_y = 0;
	if (this->y != 0 && bottom_buffer.y !=0 && bottom_buffer.y != prev_y && fabsf(this->y - bottom_buffer.y / 100.0f / 10.0f) > 0.5)
	{
		Log::W("bottom y: %f -> %f", this->y, bottom_buffer.y / 100.0f / 10.0f);
		offset_y = offset_y + (bottom_buffer.y / 100.0f / 10.0f) - y;
	}

	if (bottom_buffer.alt != 0)
	{
		if (!isnan(bottom_buffer.alt))
		{
			static float prev_alt = 0;
			if (bottom_buffer.alt != prev_alt)
			{
				prev_alt = bottom_buffer.alt;
				alt = bottom_buffer.alt / 100.0f;
				//计算倾斜补偿
				float valt = Calculate_Vertical_Alt(alt, pitch, roll);
				if (valt < alt && valt > 0)
				{
					alt = valt;
				}
				update_tick = now;
			}
		}
	}
	if (bottom_buffer.x != 0 && bottom_buffer.x != prev_x)
	{
		if (!isnan(bottom_buffer.x) && !isnan(offset_x))
		{
			x = bottom_buffer.x / 100.0f / 10.0f - offset_x;
			update_tick = now;
		}
	}
	if (bottom_buffer.y != 0 && bottom_buffer.y != prev_y)
	{
		if (!isnan(bottom_buffer.y) && !isnan(offset_y))
		{
			y = bottom_buffer.y / 100.0f / 10.0f - offset_y;
			update_tick = now;
		}
	}
	if (!isnan(bottom_buffer.x1))
	{
		x1 = bottom_buffer.x1 / 100.0f / 10.0f;
	}
	if (!isnan(bottom_buffer.y1))
	{
		y1 = bottom_buffer.y1 / 100.0f / 10.0f;
	}
	prev_x = bottom_buffer.x;
	prev_y = bottom_buffer.y;
	quality = bottom_buffer.quality;
	return true;
}

void Bottom::WhoAmI()
{
	uint8_t buffer[1];
	if (HAL_I2C_Mem_Read(&hi2c1, 0x31 << 1, 2, I2C_MEMADD_SIZE_8BIT, buffer, 1, 2) != HAL_OK)
	{
		Log::E("bottom WhoAmI HAL_I2C_Mem_Read Error");
	}
	else
	{
		Log::I("bottom IS: %x", buffer[0]);
	}
}

bool Bottom::SetAhr(float roll, float pitch, float yaw, float gx, float gy)
{
	int16_t buff[6];
	buff[0] = (int16_t)roll;
	buff[1] = (int16_t)pitch;
	buff[2] = (int16_t)yaw;
	buff[3] = (int16_t)gx;
	buff[4] = (int16_t)gy;
	buff[5] = (int16_t)(buff[0] + buff[1] + buff[2] + buff[3] + buff[4]);
	//Log::I("bottom SetAhr buffsize: %d", sizeof(buff));
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, 0x31 << 1, 4, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buff, sizeof(buff), 2);
	if (status == HAL_OK)
	{
		return true;
	}
	PERIODIC_LOGE(2000, "bottom SetAhr Error: %x", status);
	return false;
}

bool Bottom::Reset(float yaw)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, 0x31 << 1, 5, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&yaw, sizeof(yaw), 2);
	if (status == HAL_OK)
	{
		x = 0;
		y = 0;
		offset_x = 0;
		offset_y = 0;
		return true;
	}
	PERIODIC_LOGE(2000, "bottom Reset Error: %x", status);
	return false;
}

