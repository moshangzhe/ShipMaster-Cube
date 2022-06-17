//
// Created by 26380 on 2022/6/1.
//

#include "mavlink.h"
#include "check.h"
#include "usart.h"
uint8_t crc_extra_array[256]={
        50 ,124,137,0 ,237,217,104,119,0 ,0 ,
        0 ,89 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
        214,159,220,168,24 ,23 ,170,144,67 ,115,
        39 ,246,185,104,237,244,222,212,9 ,254,
        230,28 ,28 ,132,221,232,11 ,153,41 ,39 ,
        78 ,0 ,0 ,0 ,15 ,3 ,0 ,0 ,0 ,0 ,
        0 ,153,183,51 ,59 ,118,148,21 ,0 ,243,
        124,0 ,0 ,38 ,20 ,158,152,143,0 ,0 ,
        0 ,106,49 ,22 ,143,140,5 ,150,0 ,231,
        183,63 ,54 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
        175,102,158,208,56 ,93 ,138,108,32 ,185,
        84 ,34 ,174,124,237,4 ,76 ,128,56 ,116,
        134,237,203,250,87 ,203,220,25 ,226,46 ,
        29 ,223,85 ,6 ,229,203,1 ,195,109,168,
        181,148,72 ,0 ,0 ,0 ,103,154,178,200,
        0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
        0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
        0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
        0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
        0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
        0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
        0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
        0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
        0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 , 0 ,
        0 ,90 ,104,85 ,95 ,83 ,0 ,0 ,8 ,204,
        49 ,170,44 ,83 ,46 ,0
};
void MAVLINK_GetData(uint8_t *buf, uint16_t message_id, const uint8_t *payload, uint8_t len)
{
    static uint8_t seq=0;
    buf[0] = 0xfd;
    buf[1] = len;
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4] = 0x2c;
    buf[5] = 0x01;
    buf[6] = 0x01;
//    buf[7] = message_id >> 16;
//    buf[8] = message_id >>8;
//    buf[9] = message_id & 0xff;
    *(uint32_t *)(buf+7) = message_id;
    for(uint8_t i = 0;i<len;i++)
    {
        *(buf+10+i) = payload[i];
    }
    uint16_t check_data = 0;
    check_data = crc_calculate(buf+1, 9);
    crc_accumulate_buffer(&check_data, buf + 10, len);
    crc_accumulate(crc_extra_array[message_id], &check_data);
    *(uint16_t *)(buf+10+len) = check_data;
    seq++;
}

void MAVLINK_SysStatus(uint8_t *data, uint16_t voltage)
{
    uint8_t buf[21] = {0};
    *(uint32_t *)buf = 0x00;
    *(uint32_t *)(buf+4) = 0x00;
    *(uint32_t *)(buf+8) = 0x01;
    buf[12] = 0x00;
    buf[13] = 0x00;
    *(uint16_t *)(buf+14) = voltage;
    buf[16] = 0x00;
    buf[17] = 0x00;
    buf[18] = 100;
    buf[19] = 0x00;
    buf[20] = 0x00;
    MAVLINK_GetData(data, 0x01, buf, 21);
    HAL_UART_Transmit(&huart1, data, 21 + 12,0xffff);
}

void MAVLINK_HeartBeat(uint8_t *data)
{
    uint8_t heart_beat[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x51, 0x03, 0x03};
    MAVLINK_GetData(data, 0x00, heart_beat, 9);
    HAL_UART_Transmit(&huart1, data, 21,0xffff);
}

void MAVLINK_Gps(uint8_t *data, double  latitude, double longitude)
{
    uint8_t buf[30];
    *(uint32_t *)buf = 0x100;
    *(int32_t *)(buf+4) = (int32_t)(latitude * 10000000);
    *(int32_t *)(buf+8) = (int32_t)(longitude * 10000000);
    *(int32_t *)(buf+12) = 10000;
    *(int32_t *)(buf+16) = 20000;
    *(int16_t *)(buf+20) = 0;
    *(int16_t *)(buf+22) = 0;
    *(int16_t *)(buf+24) = 0;
    *(uint16_t *)(buf+26) = 0xffff;
    MAVLINK_GetData(data, 33, buf, 28);
    HAL_UART_Transmit(&huart1, data, 28 + 12,0xffff);
}

void MAVLINK_Attitude(uint8_t * data, float roll, float pitch, float yaw)
{
    uint8_t buf[28];
    *(uint32_t *)buf = 0x00;
    *(float *)(buf + 4) = pitch;
    *(float *)(buf+8) = roll;
    *(float *)(buf+12) = yaw;
    *(float *)(buf+16) = 1.1f;
    *(float *)(buf+20) = 1.1f;
    *(float *)(buf+24) = 1.1f;
    MAVLINK_GetData(data, 30, buf, 28);
    HAL_UART_Transmit(&huart1, data, 28 + 12,0xffff);
}
