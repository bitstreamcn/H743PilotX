/*
 * syscalls.cpp
 *
 *  Created on: Apr 2, 2026
 *      Author: bitstream
 */
#include "main.h"
#include "uartx.h"
#include "led.h"


#ifdef __cplusplus
extern "C" {
#endif


//printf输出重定向
/*
int _write(int file, char *ptr, int len)
{
	(void)file;
	if (0 != UART1_DMA_Send((uint8_t*)ptr, len))
	{
		//串口出问题，亮灯显示错误
		led.Error();
	}
	return 0;
}
*/

#ifdef __cplusplus
}
#endif

