/*
 * configlink.cpp
 *
 *  Created on: May 5, 2026
 *      Author: bitstream
 */
#include "configlink.h"
#include "uartx.h"
#include "log.h"
#include "Config.h"
#include <stdio.h>
#include <string.h>
#include "crc.h"
#include "application.h"
#include "file.h"

ConfigLink::ConfigLink()
{

}

ConfigLink::~ConfigLink()
{

}

void ConfigLink::RecvLoop()
{
	uint8_t buff[RX_BUFFER_SIZE+4];

	uint16_t recv_len;
	if (UART1_Recv(&buff[0], RX_BUFFER_SIZE, &recv_len))
	{
		//一次只处理一个数据包，不做粘包相关处理
		uint16_t startflag = 0xff;
		for (int i=0; i<recv_len-1; i++)
		{
			if (buff[i] == 0xAA && buff[i+1] == 0x55)
			{
				startflag = i;
				break;
			}
		}
		if (startflag == 0xff)
		{
			return;
		}
		if (startflag+4 >= recv_len)
		{
			return;
		}
		uint16_t datalen = (buff[startflag+2] & 0xff) + (buff[startflag+3] << 8);
		//检查数据是否完整
		if ((datalen + startflag + 4 + 1) > recv_len)
		{
			return;
		}
		//检查crc
		uint8_t crc = buff[startflag+4+datalen];
		//数据尾部填充，用于计算crc
		buff[startflag+4+datalen + 0] = 0;
		buff[startflag+4+datalen + 1] = 0;
		buff[startflag+4+datalen + 2] = 0;
		buff[startflag+4+datalen + 3] = 0;
		uint8_t * pData = &buff[startflag+4];
		uint32_t crc_result = HAL_CRC_Calculate(&hcrc, (uint32_t*)pData, (datalen / 4) + 1);
		uint8_t crc_calc = (crc_result & 0xff) + ((crc_result >> 8) & 0xff) + ((crc_result >> 16) & 0xff) + ((crc_result >> 24) & 0xff);
		if (crc_calc != crc)
		{
			//crc校验失败
			return;
		}

		//命令处理
		//查找:
		char * split = strchr((const char*)pData, ':');
		if (split != NULL)
		{
			*split = '\0';
		}
		char * cmd = (char*)pData;
		char * param = split+1;
		if (strcmp(cmd, "getconfig") == 0)
		{
			char * name = param;
			int param_index = cfg_get_index(name);
			SendConfig(param_index);
		}
		if (strcmp(cmd, "setconfig") == 0)
		{
			//查找=
			char * p = strchr(param, '=');
			if (p == NULL)
			{
				return;
			}
			*p = '\0';
			char * name = param;
			char * value = p+1;
			float fvalue = 0;
			if (sscanf(value, "%f", &fvalue) == 1) {
			    // 成功
				cfg_set_param_mavlink(name, fvalue);
				ConfirmSet(name);
			}
		}
		if (strcmp(cmd, "saveconfig") == 0)
		{
			//检查是否解锁，如果飞机已解锁，禁止保存，因为保存操作会导致处理频率出问题
			bool fail = false;
		    if (app.Mode() != MODE_NONE && app.Mode() != MODE_CAR)
		    {
		    	fail = true;
		    }
		    if (app.IsArmed())
		    {
		    	fail = true;
		    }
		    if (fail)
		    {
		    	Log::E("check mode and armed fail");
		    	return;
		    }
			if (cfg_write_to_eeprom())
			{
				ConfirmSave();
			}
			else
			{
				Log::E("cfg_write_to_eeprom error");
			}
		}

		//删除所有log文件
		if (strcmp(cmd, "clearlog") == 0)
		{
			file_clearlog();
		}
		//下载所有log文件
		if (strcmp(cmd, "downlog") == 0)
		{
			file_downlog();
		}
	}
}

void ConfigLink::ConfirmSet(char *name)
{
	char buffer[128];
	snprintf(buffer, 128, "[C]%s ok", name);
	UART1_DMA_Send((uint8_t *)buffer, strlen(buffer));
}

void ConfigLink::ConfirmSave()
{
	char buffer[128];
	snprintf(buffer, 128, "[C]save ok");
	UART1_DMA_Send((uint8_t *)buffer, strlen(buffer));
}

void ConfigLink::SendConfig(int param_index)
{
	uint16_t param_count = cfg_param_count();
	if(param_index >= param_count) return; //return true (i.e. command executed), even if nothing was send...

	const char *param_id = NULL;
	float param_value = 0;
	cfg_get_name_and_value(param_index, &param_id, &param_value);

	if (NULL != param_id)
	{
		char buffer[128];
		snprintf(buffer, 128, "[C]%s=%f", param_id, param_value);
		UART1_DMA_Send((uint8_t *)buffer, strlen(buffer));
	}
}


