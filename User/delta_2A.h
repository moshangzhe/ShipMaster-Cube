//
// Created by 26380 on 2022/5/29.
//

#ifndef RADARTEST_DELTA_2A_H
#define RADARTEST_DELTA_2A_H
#include "stm32f4xx_hal.h"

void RADAR_ReadData(uint16_t *distance_buf,float *angle_buf, uint8_t *data);

#endif //RADARTEST_DELTA_2A_H
