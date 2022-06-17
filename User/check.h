//
// Created by 26380 on 2022/5/31.
//

#ifndef SHIPMASTER_CUBE_CHECK_H
#define SHIPMASTER_CUBE_CHECK_H
#include "stm32f4xx_hal.h"
#define X25_INIT_CRC 0xffff
#define X25_VALIDATE_CRC 0xf0b8
void crc_accumulate(uint8_t data, uint16_t *crcAccum);
void crc_init(uint16_t* crcAccum);
uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length);
void crc_accumulate_buffer(uint16_t *crcAccum, const uint8_t *pBuffer, uint16_t length);
#endif //SHIPMASTER_CUBE_CHECK_H
