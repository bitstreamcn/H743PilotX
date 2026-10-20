/*
 * log.cpp
 *
 *  Created on: Apr 12, 2026
 *      Author: bitstream
 */
#include "main.h"
#include "uartx.h"
#include "log.h"
#include "led.h"
#include "ring_buffer.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>


//#define TX_STR(s) const_cast<CHAR*>(s)
// 最大日志长度
#define LOG_MAX_LEN 256

//extern TX_QUEUE log_queue;

uint8_t log_buffer_data[1024 * 16];
RingBuffer log_buffer;

bool ispause = false;
bool isinit = false;

void Log::I(const char *format, ...)
{
    va_list ap;
    va_start(ap, format);       // 绑定可变参数

    // 【关键】调用 vwrite 版本，把可变参数转发过去
    write("[I] ", format, ap);

    va_end(ap);
}

void Log::W(const char *format, ...)
{
    va_list ap;
    va_start(ap, format);       // 绑定可变参数

    // 【关键】调用 vwrite 版本，把可变参数转发过去
    write("[W] ", format, ap);

    va_end(ap);
}

void Log::E(const char *format, ...)
{
    va_list ap;
    va_start(ap, format);       // 绑定可变参数

    // 【关键】调用 vwrite 版本，把可变参数转发过去
    write("[E] ", format, ap);

    va_end(ap);
}


void Log::Init()
{
	rb_init(&log_buffer, log_buffer_data, sizeof(log_buffer_data));
	isinit = true;
}

void Log::Pause()
{
	ispause = true;
}

void Log::Resume()
{
	ispause = false;
}

void Log::write(const char * tag, const char *format, va_list ap)
{
	//下载log，暂停输出
	if (ispause)
	{
		return;
	}
	if (!isinit)
	{
		Init();
	}
    // 获取互斥锁，如果已被占用则无限期等待
	//UINT status;
    //UINT status = tx_mutex_get(&log_mutex, TX_WAIT_FOREVER);
	//if (status == TX_SUCCESS)
	{
	    char buffer[LOG_MAX_LEN];

	    int taglen = strlen(tag);

	    char * p = buffer;
	    snprintf(buffer, LOG_MAX_LEN, "%s", tag);
	    p += taglen;

	    vsnprintf(p, LOG_MAX_LEN - taglen, format, ap);
	    uint32_t len = strlen(buffer);

		if (0 != UART1_DMA_Send((uint8_t*)buffer, len))
		{
			//串口出问题，亮灯显示错误
			led.Error();
		}

		//写入SD卡队列（若队列满则等待，可根据需求改为 TX_NO_WAIT）
    	char time[20];
    	snprintf(time, sizeof(time), "%ld: ", HAL_GetTick());
    	int timelen = strlen(time);
    	uint32_t free = rb_free(&log_buffer);
    	uint32_t queuelen = timelen + len + 1;
    	if (free > queuelen)
    	{
			rb_put(&log_buffer, (uint8_t*)time, timelen);
			if (len != rb_put(&log_buffer, (uint8_t*)buffer, len))
			{
				//缓存满
				//assert(false);
				//led.Error();
			}
			rb_put(&log_buffer, (const uint8_t*)"\n", 1);
			//status = tx_queue_send(&log_queue, &queuelen, TX_NO_WAIT);
			//if (status != TX_SUCCESS)
			{
				//队列满
				//led.Error();
			}
    	}
    	else
    	{
    		//缓存满
    		//led.Error();
    	}
		// 释放互斥锁，供其他线程使用
		//tx_mutex_put(&log_mutex);
	}
}

#ifdef __cplusplus
extern "C" {
#endif

void LogInit(void)
{
	Log::Init();
}

void LogI(const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    Log::write("[I] ", format, ap);
    va_end(ap);
}
void LogW(const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    //Log::W(format, ap); // 调用 C++ 类
    Log::write("[W] ", format, ap);
    va_end(ap);
}
void LogE(const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    //Log::E(format, ap); // 调用 C++ 类
    Log::write("[E] ", format, ap);
    va_end(ap);
}

void LogPause()
{
	Log::Pause();
}
void LogResume()
{
	Log::Resume();
}

#ifdef __cplusplus
}
#endif

