/*
 * gpiox.cpp
 *
 *  Created on: Apr 15, 2026
 *      Author: bitstream
 */
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "../Device/Imu/imu.h"

#include "application.h"
#include "cppmain.h"

extern void ( *Dio1IrqHandler )( void );

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch(GPIO_Pin)
    {
    	case BMP_INT_Pin:
    		//GPIO_PIN_11
    		app.BmpIrqEvent();
        case RF_DIO1_Pin:
            //GPIO_PIN_2
        	if (0 != Dio1IrqHandler){
        		Dio1IrqHandler();
        	}
            break;
        case GYRO_EXTL1_Pin:
        	//GPIO_PIN_10
        	imu_data_irq++;
        	break;
        case VL53_EVENT1_Pin:
        	//GPIO_PIN_8
        	//下视雷达
        	//ToF_EventDetected++;
			//tx_semaphore_put(&tof_semaphore);
			app.BottomEvent();
        	break;
        //case I2C3_INT_Pin:
        //	app.ImuEvent();
        //	break;
        case PMW_MOTION_Pin:
        	//GPIO_PIN_9
        	//PMWMotionHandler();
        	app.BottomEvent();
        	break;
        /*
        case VL53_EVENT2_Pin:
        	//GPIO_PIN_11
        	//前视雷达
        	FrontToFEvent();
        	break;
        case TIMEPULSE_Pin:
        	//GPS脉冲 GPIO_PIN_4
        	break;
        default:
            // 未处理的引脚
            break;
            */
    }
}


#ifdef __cplusplus
}
#endif
