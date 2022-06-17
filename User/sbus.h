//
// Created by 26380 on 2022/5/30.
//

#ifndef SHIPMASTER_CUBE_SBUS_H
#define SHIPMASTER_CUBE_SBUS_H
#include "stm32f4xx_hal.h"

void SBUS_ReadData(uint16_t *ch_value, const uint8_t *data);
#endif //SHIPMASTER_CUBE_SBUS_H
