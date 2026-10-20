/*
 * qmc5883l.cpp
 *
 *  Created on: 2026年8月14日
 *      Author: bitstream
 */
#include "main.h"
#include "qmc5883l.h"
#include "i2c.h"
#include "log.h"
#include <math.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "../Imu/imu.h"

#ifdef __cplusplus
}
#endif

#ifndef PI
#define PI 3.14159265358979323846f
#endif
#define DEG_TO_RAD 0.017453292519943295f


HAL_StatusTypeDef IIC_Write_Byte(uint16_t WriteAddr,uint8_t data)//直接写一个字节
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c4, QMC5883L_ADDRESS << 1, WriteAddr, I2C_MEMADD_SIZE_8BIT, (unsigned char*)&data, 1, 2);
	if (status != HAL_OK)
	{
		Log::E("qmc588l HAL_I2C_Mem_Write Error: %x", status);
	}
	return status;
}
HAL_StatusTypeDef IIC_ReadMulByte(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)//读字节
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c4, QMC5883L_ADDRESS << 1, ReadAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, NumToRead, 2);
	if (status != HAL_OK)
	{
		Log::E("qmc588l HAL_I2C_Mem_Read Error: %x", status);
	}
	return status;
}

void QMC5883L_Init(void)
{
	uint8_t Config=0;
	// 1. 软复位 (向 0x0A 写入 0x80)
	IIC_Write_Byte(0x0A, 0x80);
	HAL_Delay(10); // 必须等待复位完成

	IIC_Write_Byte(0x0B,	0x01);
	IIC_Write_Byte(0x20,	0x40);
	IIC_Write_Byte(0x21,	0x01);
	HAL_Delay(1);

	Config =	OVER_SAMPLE_RATIO_512\
						|FULL_SCALE_2G\
						|OUTPUT_DATA_RATE_200HZ\
						|MODE_CONTROL_CONTINUOUS;
	/****OSR=512,RNG=+/-2G,ODR=200Hz,MODE= continuous*******/
	IIC_Write_Byte(0x09,	Config);
	HAL_Delay(10); // 🚨 给点时间让 ADC 完成第一次测量
}

bool QMC5883L_GetData(float* data)
{
    uint8_t status, Mag[6];

    // 1. 检查数据就绪标志 (DRDY = Bit 7)
    if (HAL_OK != IIC_ReadMulByte(0x06, &status, 1))
    {
        return false;
    }
    if ((status & 0x80) == 0) // 如果第7位为0，说明数据没准备好
    {
        //return false;
    }

    // 2. 读取磁力计数据
    if (HAL_OK != IIC_ReadMulByte(0x00, Mag, 6))
    {
        return false;
    }

    // 拼接 16 位数据
    int16_t mx = (int16_t)((Mag[1] << 8) | Mag[0]);
    int16_t my = (int16_t)((Mag[3] << 8) | Mag[2]);
    int16_t mz = (int16_t)((Mag[5] << 8) | Mag[4]); // 如果不需要倾斜补偿，Z可暂不处理
    data[0] = mx;
    data[1] = my;
    data[2] = mz;
    return true;
}

bool QMC5883L_GetAngle(float* data) // 0为北，90为东，180为南，270为西
{
    uint8_t status, Mag[6];

    // 1. 检查数据就绪标志 (DRDY = Bit 7)
    if (HAL_OK != IIC_ReadMulByte(0x06, &status, 1))
    {
        return false;
    }
    if ((status & 0x80) == 0) // 如果第7位为0，说明数据没准备好
    {
        //return false; // 必须返回，否则一直处理旧数据
    }

    // 2. 读取磁力计数据
    if (HAL_OK != IIC_ReadMulByte(0x00, Mag, 6))
    {
        return false;
    }

    // 拼接 16 位数据 (必须读取Z轴)
    int16_t mx = (int16_t)((Mag[1] << 8) | Mag[0]);
    int16_t my = (int16_t)((Mag[3] << 8) | Mag[2]);
    int16_t mz = (int16_t)((Mag[5] << 8) | Mag[4]);

    if (abs(mx) > 2000 || abs(my) > 2000 || abs(mz) > 2000) {
        //return false; // 干扰太大，丢弃这一帧
    }

    // ================= 必须加的步骤：硬磁校准 =================
    // 🚨 如果你没做校准，倾斜补偿效果会极差！必须把 offset 算出来填进去。
    // mx -= offset_x;
    // my -= offset_y;
    // mz -= offset_z;
    // ========================================================

    // 3. 倾斜补偿 (使用标准的从机体坐标系到地理水平坐标系的旋转矩阵)
    float pitch_rad = pitch * DEG_TO_RAD;
    float roll_rad  = roll * DEG_TO_RAD;

    float cos_pitch = cosf(pitch_rad);
    float sin_pitch = sinf(pitch_rad);
    float cos_roll  = cosf(roll_rad);
    float sin_roll  = sinf(roll_rad);

    // 标准倾斜补偿公式 (假设 X指北，Y指东，Z指下)
    float mx_h = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch;
    float my_h = my * cos_roll - mz * sin_roll;

    // 4. 用水平面上的分量计算航向角
    // atan2f(my_h, mx_h) 输出弧度，范围 -pi 到 pi
    float heading_rad = atan2f(my_h, mx_h);

    // 5. 弧度转为角度
    float heading_deg = heading_rad * 180.0f / PI;

    // 6. 将角度限制在 0 ~ 360 度之间
    if (heading_deg < 0.0f)
    {
        heading_deg += 360.0f;
    }
    else if (heading_deg >= 360.0f)
    {
        heading_deg -= 360.0f;
    }

    *data = heading_deg;
    return true;
}



