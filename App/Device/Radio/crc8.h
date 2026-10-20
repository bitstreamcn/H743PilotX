#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {  // C++编译器进入C模式
#endif


void crc8_init(uint8_t poly);
uint8_t crc8_calc(uint8_t *data, uint8_t len);


#ifdef __cplusplus
}
#endif
