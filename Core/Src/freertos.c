/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "delta_2A.h"
#include "sbus.h"
#include "tim.h"
#include "check.h"
#include "mavlink.h"
#include "adc.h"
#include "string.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t aRxBuffer = 0;
uint8_t sbusRxBuffer = 0;
uint8_t attitudeRxBuffer = 0;
// 串口数据结构�?????
struct RxData{
    uint8_t RxState;
    uint16_t RxCounter;
    uint8_t RxBuff[2048];
};
struct RxData radar_rx_data={0, 0, {0}}; // 雷达数据接收缓存
struct RxData sbus_rx_data = {0, 0, {0}}; // sbus遥控数据接收缓存
struct RxData attitude_rx_data = {0, 0, {0}};
struct RxData gps_rx_data = {0, 0, {0}};
// 遥控按键舵量结构�?????
struct TeleControl{
    uint16_t X1;
    uint16_t Y1;
    uint16_t X2;
    uint16_t Y2;
    uint16_t A;
    uint16_t B;
    uint16_t C;
    uint16_t D;
    uint16_t E;
    uint16_t F;
    uint16_t G;
    uint16_t H;
}tele_value = {1500, 1500, 1500, 1500,
                1500, 1500, 1500, 1500,
                1500, 1500, 1500, 1500,};

struct Attitude{
    float Roll;
    float Pitch;
    float Yaw;
}attitude_value = {0, 0, 0};
struct GPS{
    double Latitude;
    double Longitude;
}gps_data = {0, 0};
uint8_t mode_flag = 0;
uint16_t servo_pwm = 250;
/* USER CODE END Variables */
/* Definitions for radarTask */
osThreadId_t radarTaskHandle;
const osThreadAttr_t radarTask_attributes = {
  .name = "radarTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sbusTask */
osThreadId_t sbusTaskHandle;
const osThreadAttr_t sbusTask_attributes = {
  .name = "sbusTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for mavlinkTask */
osThreadId_t mavlinkTaskHandle;
const osThreadAttr_t mavlinkTask_attributes = {
  .name = "mavlinkTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for attitudeTask */
osThreadId_t attitudeTaskHandle;
const osThreadAttr_t attitudeTask_attributes = {
  .name = "attitudeTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for gpsTask */
osThreadId_t gpsTaskHandle;
const osThreadAttr_t gpsTask_attributes = {
  .name = "gpsTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//校验函数
uint16_t CRC16(uint8_t *start_byte, uint16_t num_bytes);
//电机控制函数
void MOTOR_DirCtrl(uint8_t addr, uint8_t dir);
double str2float(char *buf, uint8_t len);
double ddmm2d(double data);
/* USER CODE END FunctionPrototypes */

void StartRadarTask(void *argument);
void LedTask(void *argument);
void SubsTask(void *argument);
void ControlTask(void *argument);
void MotorTask(void *argument);
void MavLinkTask(void *argument);
void AttitudeTask(void *argument);
void GpsTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of radarTask */
  radarTaskHandle = osThreadNew(StartRadarTask, NULL, &radarTask_attributes);

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(LedTask, NULL, &ledTask_attributes);

  /* creation of sbusTask */
  sbusTaskHandle = osThreadNew(SubsTask, NULL, &sbusTask_attributes);

  /* creation of controlTask */
  controlTaskHandle = osThreadNew(ControlTask, NULL, &controlTask_attributes);

  /* creation of motorTask */
  motorTaskHandle = osThreadNew(MotorTask, NULL, &motorTask_attributes);

  /* creation of mavlinkTask */
  mavlinkTaskHandle = osThreadNew(MavLinkTask, NULL, &mavlinkTask_attributes);

  /* creation of attitudeTask */
  attitudeTaskHandle = osThreadNew(AttitudeTask, NULL, &attitudeTask_attributes);

  /* creation of gpsTask */
  gpsTaskHandle = osThreadNew(GpsTask, NULL, &gpsTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartRadarTask */
/**
  * @brief  雷达接收任务.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartRadarTask */
void StartRadarTask(void *argument)
{
  /* USER CODE BEGIN StartRadarTask */
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer, 1);
    uint16_t dis_data[256]= {0};
    float angle_data[256]= {0};
  /* Infinite loop */
  for(;;)
  {
    if(radar_rx_data.RxState == 1)
    {
      RADAR_ReadData(dis_data, angle_data, radar_rx_data.RxBuff);
      radar_rx_data.RxState = 0;
      radar_rx_data.RxCounter = 0;
    }
    osDelay(10);
  }
  /* USER CODE END StartRadarTask */
}

/* USER CODE BEGIN Header_LedTask */
/**
* @brief led任务.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LedTask */
void LedTask(void *argument)
{
  /* USER CODE BEGIN LedTask */
  uint16_t  bb =0;
  /* Infinite loop */
  for(;;)
  {

    osDelay(1000);
  }
  /* USER CODE END LedTask */
}

/* USER CODE BEGIN Header_SubsTask */
/**
* @brief sbus数据接收处理任务.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SubsTask */
void SubsTask(void *argument)
{
  /* USER CODE BEGIN SubsTask */
  uint16_t sbus_ch_value[16] = {0};
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&sbusRxBuffer, 1);
  /* Infinite loop */
  for(;;)
  {
      if(sbus_rx_data.RxState == 1)
      {
          SBUS_ReadData(sbus_ch_value, sbus_rx_data.RxBuff);
          tele_value.X1 = (uint16_t)(sbus_ch_value[0] * (1500.0/1002.0));
          tele_value.Y1 = (uint16_t)(sbus_ch_value[1] * (1500.0/1002.0));
          tele_value.X2 = (uint16_t)(sbus_ch_value[2] * (1500.0/1002.0));
          tele_value.Y2 = (uint16_t)(sbus_ch_value[3] * (1500.0/1002.0));
          tele_value.A = (uint16_t)(sbus_ch_value[4] * (1500.0/1002.0));
          tele_value.B = (uint16_t)(sbus_ch_value[5] * (1500.0/1002.0));
          tele_value.C = (uint16_t)(sbus_ch_value[6] * (1500.0/1002.0));
          tele_value.D = (uint16_t)(sbus_ch_value[7] * (1500.0/1002.0));
          tele_value.E = (uint16_t)(sbus_ch_value[8] * (1500.0/1002.0));
          tele_value.F = (uint16_t)(sbus_ch_value[9] * (1500.0/1002.0));
          tele_value.G = (uint16_t)(sbus_ch_value[10] * (1500.0/1002.0));
          tele_value.H = (uint16_t)(sbus_ch_value[11] * (1500.0/1002.0));
          sbus_rx_data.RxState = 0;
          sbus_rx_data.RxCounter = 0;
      }
      osDelay(5);
  }
  /* USER CODE END SubsTask */
}

/* USER CODE BEGIN Header_ControlTask */
/**
* @brief 电机控制任务.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ControlTask */
void ControlTask(void *argument)
{
  /* USER CODE BEGIN ControlTask */
  struct TeleControl old_tele_data;
  old_tele_data.G = 1500;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 250);
  /* Infinite loop */
  for(;;)
  {
    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, tele_value.Y1);
    if(old_tele_data.G != tele_value.G)
    {
        if(tele_value.G < 1300)
        {
            mode_flag = 0;
            MOTOR_DirCtrl(0x01, 0x10);
            osDelay(5);
            MOTOR_DirCtrl(0x02, 0x10);
            osDelay(5);
            MOTOR_DirCtrl(0x03, 0x11);
            servo_pwm = 250;
        }else if(tele_value.G > 1700)
        {
            mode_flag = 2;
            MOTOR_DirCtrl(0x01, 0x11);
            osDelay(5);
            MOTOR_DirCtrl(0x02, 0x11);
            osDelay(5);
            MOTOR_DirCtrl(0x03, 0x11);
            servo_pwm = 750;
        }else
        {
            mode_flag = 1;
            MOTOR_DirCtrl(0x01, 0x10);
            osDelay(5);
            MOTOR_DirCtrl(0x02, 0x11);
            osDelay(5);
            MOTOR_DirCtrl(0x03, 0x10);
            servo_pwm = 1250;
        }
    }

    if(tele_value.X2 > 1700)
        servo_pwm+=30;
    if(tele_value.X2 < 1300)
      servo_pwm-=30;
    if(servo_pwm < 250)
        servo_pwm = 250;
    if(servo_pwm >1250)
        servo_pwm = 1250;
    old_tele_data.G = tele_value.G;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, servo_pwm);
    osDelay(100);
  }
  /* USER CODE END ControlTask */
}

/* USER CODE BEGIN Header_MotorTask */
/**
* @brief 无刷电机控制任务
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorTask */
void MotorTask(void *argument)
{
  /* USER CODE BEGIN MotorTask */
  //初始化pwm输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1500);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1500);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1500);
    struct MotorOutValue{
        uint16_t M1;
        uint16_t M2;
        uint16_t M3;
        uint16_t M4;
    }out_value = {1500, 1500, 1500, 1500};
  /* Infinite loop */
  for(;;)
  {
    if(mode_flag == 0)
    {
        out_value.M1 = tele_value.Y1 - (1500 - tele_value.X1);
        out_value.M2 = tele_value.Y1 + (1500 - tele_value.X1);
        out_value.M3 = tele_value.Y1 + (1500 - tele_value.X1);
        out_value.M4 = tele_value.Y1 - (1500 - tele_value.X1);
    }
    else if(mode_flag == 1)
    {
        out_value.M1 = 1500 - (tele_value.Y1 - 1500) - (1500 - tele_value.X1);
        out_value.M2 = 1500 - (tele_value.Y1 - 1500) + (1500 - tele_value.X1);
        out_value.M3 = 1500 - (tele_value.Y1 - 1500) + (1500 - tele_value.X1);
        out_value.M4 = 1500 - (tele_value.Y1 - 1500) - (1500 - tele_value.X1);
    }
    else
    {
        out_value.M1 = tele_value.Y1 - (1500 - tele_value.X1);
        out_value.M2 = tele_value.Y1 + (1500 - tele_value.X1);
        out_value.M3 = 1500 - (tele_value.Y1 - 1500) + (1500 - tele_value.X1);
        out_value.M4 = 1500 - (tele_value.Y1 - 1500) - (1500 - tele_value.X1);
    }
    if(out_value.M1 < 800)
        out_value.M1 = 800;
    if(out_value.M1 > 2100)
        out_value.M1 = 2100;
    if(out_value.M2 < 800)
      out_value.M2 = 800;
    if(out_value.M2 > 2100)
      out_value.M2 = 2100;
    if(out_value.M3 < 800)
      out_value.M3 = 800;
    if(out_value.M3 > 2100)
      out_value.M3 = 2100;
    if(out_value.M4 < 800)
      out_value.M4 = 800;
    if(out_value.M4 > 2100)
      out_value.M4 = 2100;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, out_value.M1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, out_value.M2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, out_value.M3);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, out_value.M4);
    osDelay(5);
  }
  /* USER CODE END MotorTask */
}

/* USER CODE BEGIN Header_MavLinkTask */
/**
* @brief Function implementing the mavlinkTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MavLinkTask */
void MavLinkTask(void *argument)
{
  /* USER CODE BEGIN MavLinkTask */
  uint8_t mavlink_buf[50]={0};
  uint8_t heart_beat[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x51, 0x03, 0x03};
  uint8_t payload[21] = {0};
  uint16_t adc_value = 0;
  float voltage = 0;
  /* Infinite loop */
  for(;;)
  {

    MAVLINK_HeartBeat(mavlink_buf);
    osDelay(5);
    adc_value = HAL_ADC_GetValue(&hadc1);
    voltage = (float)(adc_value/4095.0)*3300;
    voltage = voltage*(float)(72.7/4.7);
    MAVLINK_SysStatus(mavlink_buf, (uint16_t)voltage);
    HAL_ADC_Start(&hadc1);
    osDelay(5);
    MAVLINK_Gps(mavlink_buf, gps_data.Latitude, gps_data.Longitude);
    osDelay(5);
    if(attitude_value.Roll > 180)
        attitude_value.Roll = attitude_value.Roll -360;
    MAVLINK_Attitude(mavlink_buf, attitude_value.Roll/57, attitude_value.Pitch/57, attitude_value.Yaw/57);
    osDelay(100);
  }
  /* USER CODE END MavLinkTask */
}

/* USER CODE BEGIN Header_AttitudeTask */
/**
* @brief Function implementing the attitudeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AttitudeTask */
void AttitudeTask(void *argument)
{
  /* USER CODE BEGIN AttitudeTask */
  HAL_UART_Receive_IT(&huart4, &attitudeRxBuffer, 1);
  /* Infinite loop */
  for(;;)
  {
    if(attitude_rx_data.RxState == 1)
    {
        attitude_value.Roll = (float)((attitude_rx_data.RxBuff[3]<<8)|attitude_rx_data.RxBuff[2])/32768.0f*180.0f;
        attitude_value.Pitch = (float)((attitude_rx_data.RxBuff[5]<<8)|attitude_rx_data.RxBuff[4])/32768.0f*180.0f;
        attitude_value.Yaw = (float)((attitude_rx_data.RxBuff[7]<<8)|attitude_rx_data.RxBuff[6])/32768.0f*180.0f;
        attitude_rx_data.RxCounter = 0;
        attitude_rx_data.RxState = 0;
    }
    osDelay(100);
  }
  /* USER CODE END AttitudeTask */
}

/* USER CODE BEGIN Header_GpsTask */
/**
* @brief Function implementing the gpsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GpsTask */
void GpsTask(void *argument)
{
  /* USER CODE BEGIN GpsTask */
    static double old_latitude = 0;
    static double old_longitude = 0;
    static uint8_t flag = 0;
    HAL_UART_Receive_IT(&huart5, gps_rx_data.RxBuff, 1024);
  /* Infinite loop */
  for(;;)
  {
    if(gps_rx_data.RxState == 1)
    {
        char *gnrmc = strstr((char *)gps_rx_data.RxBuff, "$GNRMC");
        if(gnrmc)
        {
            if(strstr(gnrmc, ",V,"))
            {
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                continue;
            }

            char *latitude = strstr(gnrmc, ",A,") + 3;
            char *latitude_end = strstr(latitude, ",N,");
            char *longitude = latitude_end + 3;
            char *longitude_end = strstr(latitude, ",E,");
            double latitude_data =  str2float(latitude, latitude_end - latitude);
            double longitude_data =  str2float(longitude, longitude_end - longitude);
            latitude_data = ddmm2d(latitude_data);
            longitude_data = ddmm2d(longitude_data);
            if(flag == 0)
            {
                old_latitude = latitude_data;
                old_longitude = longitude_data;
                flag = 1;
            }
            if(old_latitude - latitude_data > 0.003 || old_latitude - latitude_data < -0.003)
                continue;
            if(old_longitude - longitude_data > 0.003 || old_longitude - longitude_data < -0.003)
                continue;
            old_longitude = longitude_data;
            old_latitude = latitude_data;
            gps_data.Latitude = latitude_data;
            gps_data.Longitude = longitude_data;
        }
        gps_rx_data.RxCounter = 0;
        gps_rx_data.RxState = 0;
    }
    osDelay(100);
  }
  /* USER CODE END GpsTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
//电机方向控制
void MOTOR_DirCtrl(uint8_t addr, uint8_t dir)
{
    uint8_t data[8];
    data[0] = 0xa5;
    data[1] = 0xa5;
    data[2] = addr;
    data[3] = 0x10;
    data[4] = dir;
    data[5] = 0x5a;
    data[6] = 0x5a;
    data[7] = CRC16(data, 7);
    HAL_UART_Transmit(&huart6, data, 8, 0xffff);
}

//串口中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    //�?????光雷达数据接�?????
    if(huart == &huart3)
    {
        static uint16_t len = 0;
        radar_rx_data.RxBuff[radar_rx_data.RxCounter] = aRxBuffer;
        if(radar_rx_data.RxCounter > 0 && radar_rx_data.RxState == 0)
        {
            if(radar_rx_data.RxCounter == 3)
                len = (radar_rx_data.RxBuff[1] << 8) + radar_rx_data.RxBuff[2];
            if(radar_rx_data.RxCounter == len + 1 && radar_rx_data.RxCounter > 3)
            {
                radar_rx_data.RxState = 1;
                len = 0;
            }
            radar_rx_data.RxCounter++;
        }
        if(aRxBuffer == 0xaa && radar_rx_data.RxCounter==0)
            radar_rx_data.RxCounter++;
        HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer, 1);
    }
    //sbus遥控数据接收
    else if(huart == &huart2)
    {
        sbus_rx_data.RxBuff[sbus_rx_data.RxCounter] = sbusRxBuffer;
        if(sbus_rx_data.RxCounter >0)
        {
            if(sbus_rx_data.RxCounter == 24)
                sbus_rx_data.RxState = 1;
            sbus_rx_data.RxCounter ++;
        }
        if(sbusRxBuffer == 0x0f && sbus_rx_data.RxCounter == 0)
        {
            sbus_rx_data.RxCounter ++;
        }
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&sbusRxBuffer, 1);
    }
    else if(huart == &huart4)
    {
        attitude_rx_data.RxBuff[attitude_rx_data.RxCounter] = attitudeRxBuffer;
        if(attitude_rx_data.RxCounter >0)
        {
            if(attitude_rx_data.RxCounter == 10)
                attitude_rx_data.RxState = 1;
            attitude_rx_data.RxCounter ++;
        }
        if(attitudeRxBuffer == 0x55 && attitude_rx_data.RxCounter == 0)
        {
            attitude_rx_data.RxCounter ++;
        }
        HAL_UART_Receive_IT(&huart4, &attitudeRxBuffer, 1);
    }
    else if(huart == &huart5)
    {
        gps_rx_data.RxState = 1;
        HAL_UART_Receive_IT(&huart5, gps_rx_data.RxBuff, 1024);
    }
}

//CRC16校验
uint16_t CRC16(uint8_t *start_byte, uint16_t num_bytes)
{
    uint16_t check_sum = 0;
    while (num_bytes--)
    {
        check_sum += *start_byte++;
    }
    return check_sum;
}

uint16_t str2int(const char *buf, uint8_t len)
{
    uint16_t num =0;
    for(uint8_t i=0;i<len;i++)
    {
        num += (*(buf +i) -'0') * pow(10, (len-1 -i));
    }
    return num;
}
double str2float(char *buf, uint8_t len)
{
    char *point = strstr(buf, ".");
    uint16_t integral = str2int(buf, point - buf);
    uint16_t decimals = str2int(point+1, len-(point -buf)-1);
    double dec = decimals/pow(10, len -(point-buf)-1);
    return (double)integral + dec;
}

double ddmm2d(double data)
{
    data /=100;
    double d = data;
    uint16_t dd = (uint16_t)d;
    double m = (data - dd) * 100;
    double out = dd+ m/60;
    return out;
}
/* USER CODE END Application */

