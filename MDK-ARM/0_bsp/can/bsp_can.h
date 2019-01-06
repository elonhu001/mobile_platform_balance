/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_can.h
 * @brief      this file contains sd card basic operating function
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-23-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */
  
#ifndef _BSP__CAN_H
#define _BSP__CAN_H

#include "stm32f4xx_HAL.h"


#define CAN_RX_FIFO_SIZE    (8)
#define CAN_TX_FIFO_SIZE    (8)

/* CAN send and receive ID */
typedef enum
{
  CAN_3508_M1_ID       = 0x201,
  CAN_3508_M2_ID       = 0x202,
  CAN_3508_M3_ID       = 0x203,
  CAN_3508_M4_ID       = 0x204,
  CAN_YAW_MOTOR_ID     = 0x205,
  CAN_PIT_MOTOR_ID     = 0x206, 
  CAN_TRIGGER_MOTOR_ID = 0x207,
  CAN_CHASSIS_ZGYRO_ID = 0x401,
  CAN_GIMBAL_ZGYRO_ID  = 0x402,

  CAN_ZGYRO_RST_ID     = 0x406,
  CAN_CHASSIS_ALL_ID   = 0x200,
  CAN_GIMBAL_ALL_ID    = 0x1ff,

} can_msg_id_e;

/* can receive motor parameter structure */
#define FILTER_BUF 5
typedef struct
{
  uint16_t ecd;
  uint16_t last_ecd;
  
  int16_t  speed_rpm;      //unit:r/min
  int16_t  speed_rpm_last;
  int16_t  acc_speed_rad;  //unit:rad/s^2
  int16_t  given_current;

  int32_t  round_cnt;
  int32_t  total_ecd;
  int32_t  total_angle;
  
  uint16_t offset_ecd;
  uint32_t msg_cnt;
  
  int32_t  ecd_raw_rate;
  int32_t  rate_buf[FILTER_BUF];
  uint8_t  buf_cut;
  int32_t  filter_rate;
} moto_measure_t;

extern moto_measure_t moto_chassis[2];

HAL_StatusTypeDef can_filter_init(CAN_HandleTypeDef* hcan);
void get_moto_offset(moto_measure_t* ptr, uint8_t* data);
void encoder_data_handler(moto_measure_t* ptr, uint8_t* data);
void send_chassis_cur(int16_t *mot_current);
#endif
