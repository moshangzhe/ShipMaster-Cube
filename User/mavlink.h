//
// Created by 26380 on 2022/6/1.
//

#ifndef SHIPMASTER_CUBE_MAVLINK_H
#define SHIPMASTER_CUBE_MAVLINK_H
#include "stm32f4xx_hal.h"

void MAVLINK_GetData(uint8_t *buf, uint16_t message_id, const uint8_t *payload, uint8_t len);
void MAVLINK_SysStatus(uint8_t *data, uint16_t voltage);
void MAVLINK_HeartBeat(uint8_t *data);
void MAVLINK_Attitude(uint8_t * data, float roll, float pitch, float yaw);
void MAVLINK_Gps(uint8_t *data, double  latitude, double longitude);
#endif //SHIPMASTER_CUBE_MAVLINK_H
