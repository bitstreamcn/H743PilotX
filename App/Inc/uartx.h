/*
 * uartx.h
 *
 *  Created on: Apr 2, 2026
 *      Author: bitstream
 */

#ifndef INC_UARTX_H_
#define INC_UARTX_H_

#include <cstdint>
#include <stdbool.h>

#define TX_BUFFER_SIZE    8192
#define RX_BUFFER_SIZE 256
#define UART1_RX_BUFFER_COUNT 2

void UART1_DMA_Recv_Start();
uint8_t UART1_DMA_Send(uint8_t *pData, uint16_t len);
bool UART1_Recv(uint8_t * buff, uint16_t buff_size, uint16_t * recv_len);

#endif /* INC_UARTX_H_ */
