//
// Created by 26380 on 2022/5/30.
//

#include "sbus.h"

void SBUS_ReadData(uint16_t *ch_value, const uint8_t *data)
{
    //读取遥控sbus协议
    ch_value[0] = (data[1]>>0|data[2]<<8)&0x7ff;
    ch_value[1] = data[2]>>3|data[3]<<5&0x7ff;
    ch_value[2] = data[3]>>6|data[4]<<2|data[5]<<10&0x7ff;
    ch_value[3] = data[5]>>1|data[6]<<7&0x7ff;
    ch_value[4] = data[6]>>4|data[7]<<4&0x7ff;
    ch_value[5] = data[7]>>7|data[8]<<1|data[9]<<9&0x7ff;
    ch_value[6] = data[9]>>2|data[10]<<6&0x7ff;
    ch_value[7] = data[10]>>5|data[11]<<3&0x7ff;
    ch_value[8] = data[12]>>0|data[13]<<8&0x7ff;
    ch_value[9] = data[13]>>3|data[14]<<5&0x7ff;
    ch_value[10] = data[14]>>6|data[15]<<2|data[16]<<10&0x7ff;
    ch_value[11] = data[16]>>1|data[17]<<7&0x7ff;
    ch_value[12] = data[17]>>4|data[18]<<4&0x7ff;
    ch_value[13] = data[18]>>7|data[19]<<1|data[20]<<9&0x7ff;
    ch_value[14] = data[20]>>2|data[21]<<6&0x7ff;
    ch_value[15] = data[21]>>5|data[22]<<3&0x7ff;
}