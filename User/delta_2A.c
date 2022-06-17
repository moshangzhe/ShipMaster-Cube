//
// Created by 26380 on 2022/5/29.
//
#include "delta_2A.h"

void RADAR_ReadData(uint16_t *distance_buf,float *angle_buf, uint8_t *data)
{
    uint16_t data_len = (data[6]<<8) + data[7];
    uint8_t *param_data = data + 8;
    uint16_t start_angle = ((param_data[3] << 8) + param_data[4])/100;
    uint16_t j = 0;
    for(uint16_t  i=6; i<data_len; i+=3)
    {
        uint16_t distance = (param_data[i] <<8) + param_data[i+1];
        *distance_buf = distance/4;
        *angle_buf = (float)(start_angle + (j*22.5/((data_len-5)/3.0)));
        angle_buf++;
        j++;
        distance_buf++;
    }
}
