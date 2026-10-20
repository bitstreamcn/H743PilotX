/*
 * uart.cpp
 *
 *  Created on: Apr 2, 2026
 *      Author: bitstream
 */
#include "main.h"
#include "usart.h"
#include "uartx.h"
#include <stdlib.h>
#include <cstring>
#include "crc.h"
#include "log.h"
#include "gps.h"

// 发送缓存 放在 D2 RAM（H7必须）
uint8_t UART1_Tx_Buffer[2][TX_BUFFER_SIZE] __attribute__((section(".RAM")));
volatile uint8_t UART1_Tx_Buffer_Index = 0;
uint16_t UART1_Tx_Buffer_Data_Index = 0;

uint8_t UART1_Rx_Buffer[UART1_RX_BUFFER_COUNT][RX_BUFFER_SIZE + 1] __attribute__((section(".RAM")));   // 两块物理缓冲
volatile uint8_t UART1_Rx_Buffer_Index = 0;    // DMA 当前正在填的缓冲下标
volatile uint16_t UART1_Rx_Buffer_Data_Index[UART1_RX_BUFFER_COUNT] = {0, 0};		   //双缓冲的数据索引

// 发送状态标志
volatile uint8_t UART1_Tx_Busy = 0;  // 1=发送中 0=空闲

volatile bool uart1_recv_error = false;

#define UART1_CRITICAL_ENTER()  do { \
    HAL_NVIC_DisableIRQ(USART1_IRQn); \
    HAL_NVIC_DisableIRQ(DMA1_Stream6_IRQn); \
} while(0)
#define UART1_CRITICAL_EXIT()   do { \
    HAL_NVIC_EnableIRQ(USART1_IRQn); \
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn); \
} while(0)

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        // 发送完成，释放标志
        UART1_Tx_Busy = 0;
    }
}


// 功能：UART1 DMA 发送数据（非阻塞）
// 参数：数据指针、长度
// 返回：0成功 1忙
uint8_t UART1_DMA_Send(uint8_t *pData, uint16_t len)
{
    // 长度保护
    if(len > TX_BUFFER_SIZE)
    {
        return 1;
    }
    //缓冲区爆炸，发送数据太多，串口速度跟不上
    if ((UART1_Tx_Buffer_Data_Index + len + 5) > TX_BUFFER_SIZE)
    {
    	UART1_Tx_Buffer_Data_Index = 0;
    	return 1;
    }
    //开始标志 AA55
    UART1_Tx_Buffer[UART1_Tx_Buffer_Index][UART1_Tx_Buffer_Data_Index] = 0xaa;
    UART1_Tx_Buffer[UART1_Tx_Buffer_Index][UART1_Tx_Buffer_Data_Index + 1] = 0x55;
    UART1_Tx_Buffer[UART1_Tx_Buffer_Index][UART1_Tx_Buffer_Data_Index + 2] = len & 0xff;
    UART1_Tx_Buffer[UART1_Tx_Buffer_Index][UART1_Tx_Buffer_Data_Index + 3] = (len >> 8) & 0xff;
    UART1_Tx_Buffer_Data_Index += 4;

    uint8_t * pDataBuff = &(UART1_Tx_Buffer[UART1_Tx_Buffer_Index][UART1_Tx_Buffer_Data_Index]);

    // 复制数据到发送缓存
    memcpy(pDataBuff, pData, len);
    UART1_Tx_Buffer_Data_Index += len;

    //为了crc计算对齐
    pDataBuff[len + 0] = 0;
    pDataBuff[len + 1] = 0;
    pDataBuff[len + 2] = 0;
    pDataBuff[len + 3] = 0;

    uint8_t crc = 0;
    uint32_t crc_result = HAL_CRC_Calculate(&hcrc, (uint32_t*)pDataBuff, (len / 4) + 1);
    crc = (crc_result & 0xff) + ((crc_result >> 8) & 0xff) + ((crc_result >> 16) & 0xff) + ((crc_result >> 24) & 0xff);

    UART1_Tx_Buffer[UART1_Tx_Buffer_Index][UART1_Tx_Buffer_Data_Index] = crc;
    //crc校验码
    UART1_Tx_Buffer_Data_Index++;

    if (UART1_Tx_Busy == 0)
    {
		// 标记发送中
		UART1_Tx_Busy = 1;

		// 启动 DMA 发送
		if(HAL_UART_Transmit_DMA(&huart1, UART1_Tx_Buffer[UART1_Tx_Buffer_Index], UART1_Tx_Buffer_Data_Index) != HAL_OK)
		{
			UART1_Tx_Busy = 0;
			return 1;
		}

		UART1_Tx_Buffer_Data_Index = 0;
		UART1_Tx_Buffer_Index = (UART1_Tx_Buffer_Index + 1) % 2;
    }

    return 0; // 启动成功
}

void UART1_DMA_Recv_Start()
{
	static bool once = true;
	if (!once)
	{
		Log::E("recall UART1_DMA_Recv_Start");
		return;
	}
	HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_Rx_Buffer[UART1_Rx_Buffer_Index], RX_BUFFER_SIZE);
	if (HAL_OK != status)
	{
		Log::E("UART1_DMA_Recv_Start fail: %x", status);
		uart1_recv_error = true;
	}
	once = false;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART1)  // 判断是UART1
    {
    	UART1_Rx_Buffer_Data_Index[UART1_Rx_Buffer_Index] += Size;
    	UART1_Rx_Buffer[UART1_Rx_Buffer_Index][UART1_Rx_Buffer_Data_Index[UART1_Rx_Buffer_Index]] = 0;

    	bool isfull = true;
    	uint8_t next_buffer_index = (UART1_Rx_Buffer_Index + 1) % UART1_RX_BUFFER_COUNT;
    	for (uint8_t i=0; i<UART1_RX_BUFFER_COUNT; i++)
    	{
    		//如果缓冲数据没有被取走，就跳过
    		if (UART1_Rx_Buffer_Data_Index[next_buffer_index] > 0)
    		{
    			next_buffer_index++;
    			next_buffer_index %= UART1_RX_BUFFER_COUNT;
    			continue;
    		}
    		isfull = false;
    		break;
    	}
    	if (isfull)
    	{
    		UART1_Rx_Buffer_Index++;
    		UART1_Rx_Buffer_Index %= UART1_RX_BUFFER_COUNT;
    		//缓冲区满
    		uart1_recv_error = true;
    		return;
    	}
    	UART1_Rx_Buffer_Index = next_buffer_index;
		// 重新开启下一次接收（必须写！）
		HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, &UART1_Rx_Buffer[UART1_Rx_Buffer_Index][UART1_Rx_Buffer_Data_Index[UART1_Rx_Buffer_Index]],
				RX_BUFFER_SIZE - UART1_Rx_Buffer_Data_Index[UART1_Rx_Buffer_Index]);
		if (HAL_OK != status)
		{
			uart1_recv_error = true;
		}
    }
    else if(huart->Instance == USART2)
    {
    	uart2_data_size = Size;
    	uart2_data_recv = true;
    }
}

bool UART1_Recv(uint8_t * buff, uint16_t buff_size, uint16_t * recv_len)
{
	bool isfull = true;
	//检查缓冲区满
	for (int i=0; i<UART1_RX_BUFFER_COUNT; i++)
	{
		if (UART1_Rx_Buffer_Data_Index[i] == 0)
		{
			isfull = false;
			break;
		}
	}
	if (isfull)
	{
		Log::E("UART1_Recv Full");
	}

	//检查是否需要重新启动接收
	if (uart1_recv_error)
	{
		UART1_CRITICAL_ENTER();
		if (UART1_Rx_Buffer_Data_Index[UART1_Rx_Buffer_Index] == 0)
		{
			HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, &UART1_Rx_Buffer[UART1_Rx_Buffer_Index][UART1_Rx_Buffer_Data_Index[UART1_Rx_Buffer_Index]],
					RX_BUFFER_SIZE - UART1_Rx_Buffer_Data_Index[UART1_Rx_Buffer_Index]);
			if (HAL_OK != status)
			{
				uart1_recv_error = true;
			}
			else
			{
				uart1_recv_error = false;
			}
		}
		UART1_CRITICAL_EXIT();
	}

	if (!buff || !recv_len || 0 == buff_size)
	{
		return false;
	}

    uint8_t data_index;
    uint16_t data_len;
    bool found = false;

    // 统一在临界区内完成缓冲区选择和长度记录
    UART1_CRITICAL_ENTER();
    // 全满情况：只能处理当前 DMA 缓冲
    if (isfull) {
        data_index = UART1_Rx_Buffer_Index;
        data_len = UART1_Rx_Buffer_Data_Index[data_index];
        found = (data_len > 0);
    }
    // 错误恢复且当前缓冲仍有残留数据
    else if (uart1_recv_error && UART1_Rx_Buffer_Data_Index[UART1_Rx_Buffer_Index] > 0) {
        data_index = UART1_Rx_Buffer_Index;
        data_len = UART1_Rx_Buffer_Data_Index[data_index];
        found = true;
    }
    // 常规情况：从非当前缓冲中取出数据
    else {
        data_index = (UART1_Rx_Buffer_Index + 1) % UART1_RX_BUFFER_COUNT;
        for (int i = 0; i < UART1_RX_BUFFER_COUNT; i++) {
            if (UART1_Rx_Buffer_Data_Index[data_index] == 0 ||
                data_index == UART1_Rx_Buffer_Index) {
                data_index = (data_index + 1) % UART1_RX_BUFFER_COUNT;
                continue;
            }
            data_len = UART1_Rx_Buffer_Data_Index[data_index];
            found = true;
            break;
        }
    }
    UART1_CRITICAL_EXIT();

    if (!found)
        return false;

    // 在中断开放的环境下安全拷贝（DMA 永远不会写这块缓冲）
    if (buff_size <= data_len) {
        memcpy(buff, UART1_Rx_Buffer[data_index], buff_size);
        *recv_len = buff_size;
        uint16_t remain = data_len - buff_size;
        if (remain > 0) {
            memmove(UART1_Rx_Buffer[data_index],
                    &UART1_Rx_Buffer[data_index][buff_size], remain);
        }
    } else {
        memcpy(buff, UART1_Rx_Buffer[data_index], data_len);
        *recv_len = data_len;
    }

    // 原子更新 Data_Index
    UART1_CRITICAL_ENTER();
    if (buff_size <= UART1_Rx_Buffer_Data_Index[data_index]) {
        UART1_Rx_Buffer_Data_Index[data_index] -= buff_size;
    } else {
        UART1_Rx_Buffer_Data_Index[data_index] = 0;
    }
    UART1_CRITICAL_EXIT();

	return true;
}
