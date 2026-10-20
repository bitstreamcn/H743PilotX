/*
 * i2cx.cpp
 *
 *  Created on: May 5, 2026
 *      Author: bitstream
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "i2c.h"

// ---------------------------
// I2C2 DMA 完成回调
// ---------------------------
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance == I2C2){
        // 发送完成
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance == I2C2){
        // 接收完成
    }
}

// ---------------------------
// I2C2 错误回调
// ---------------------------
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance == I2C2){
        // 出错处理
    }
}


#ifdef __cplusplus
}
#endif
