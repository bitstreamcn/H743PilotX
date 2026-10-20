/*
 * timx.cpp
 *
 *  Created on: May 4, 2026
 *      Author: bitstream
 */

#include "application.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1)
    {
        // 这里 200Hz 执行（5ms一次）
    	app.Tim200hz();
    	__HAL_TIM_SET_PRESCALER(htim, 119);
    	__HAL_TIM_SET_AUTORELOAD(htim, 4999);
    }
    else if (htim->Instance == TIM2) {
        // 在这里执行每 2ms 需要处理的任务
        // 例如：读取传感器、更新卡尔曼滤波器、翻转调试引脚等
    	app._500Hz_Event();
    	__HAL_TIM_SET_PRESCALER(htim, 119);
    	__HAL_TIM_SET_AUTORELOAD(htim, 1999);
    }
}


#ifdef __cplusplus
}
#endif


