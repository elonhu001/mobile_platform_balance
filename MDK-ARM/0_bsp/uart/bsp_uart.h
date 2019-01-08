#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "usart.h"
#include "sys_config.h"

/**************************************wheel uart*******************************************/
#define WHEEL_L_HUART huart2
#define WHEEL_R_HUART huart3
#define WHEEL_HUART_RX_BUFFER_SIZE (20)
#define WHEEL_MSG_DEFAULT {0xbb, 0x81, 0, 0, 0, 0, 0, 0, 0, 0xEE}

typedef struct
{
	uint8_t start;
	uint8_t msg_type;
	uint8_t status;
	int16_t speed_get;
	int16_t speed_set;
	int16_t s_vol;
	int16_t s_cur;
	uint8_t err;
	uint8_t dir;
	uint8_t end;
}wheel_msg_t;

extern uint8_t wheel_left_uart_rx_buffer[WHEEL_HUART_RX_BUFFER_SIZE];
extern uint8_t wheel_right_uart_rx_buffer[WHEEL_HUART_RX_BUFFER_SIZE];
extern wheel_msg_t wheel_left_msg;
extern wheel_msg_t wheel_right_msg;

void wheel_message_handle(wheel_msg_t *wheel_msg, uint8_t *rx_buf);
/**************************************debug and control uart*******************************************/
#define DEBUG_CTRL_HUART       huart1
#define DEBUG_CTRL_HUART_RX_SIZE (200)
typedef struct
{
	float set_speed;
	float set_rotat;
}debug_ctrl_msg_t;
extern debug_ctrl_msg_t debug_ctrl_msg;
extern uint8_t debug_ctrl_rx_buffer[DEBUG_CTRL_HUART_RX_SIZE];



void debug_ctrl_message_handle(debug_ctrl_msg_t *debug_ctrl_msg, uint8_t *rx_buf);
/******************************************************************************************************/
void uart_init_idle(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);
void uart_receive_handler(UART_HandleTypeDef *huart);
#endif

