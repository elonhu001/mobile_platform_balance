#include "string.h"
#include "stdlib.h"
#include "bsp_uart.h"
#include "usart.h"
#include "main.h"
#include "sys_config.h"

/*轮毂电机串口部分*/
uint8_t wheel_left_uart_rx_buffer[WHEEL_HUART_RX_BUFFER_SIZE];//左轮接收buf
uint8_t wheel_right_uart_rx_buffer[WHEEL_HUART_RX_BUFFER_SIZE];//右轮接收buf
wheel_msg_t wheel_left_msg = WHEEL_MSG_DEFAULT;//左轮信息
wheel_msg_t wheel_right_msg = WHEEL_MSG_DEFAULT;//右轮信息

void wheel_message_handle(wheel_msg_t *wheel_msg, uint8_t *rx_buf)
{
	if((rx_buf[0] == wheel_msg->start) || (rx_buf[1] == wheel_msg->msg_type) || (rx_buf[16] == wheel_msg->end))
	{
		wheel_msg->status = rx_buf[2];
		wheel_msg->speed_get = (rx_buf[3] << 8) | rx_buf[4];
		wheel_msg->speed_set = (rx_buf[5] << 8) | rx_buf[6];
		wheel_msg->s_vol = (rx_buf[7] << 8) | rx_buf[8];
		wheel_msg->s_cur = (rx_buf[9] << 8) | rx_buf[10];
		wheel_msg->err = rx_buf[11];
		wheel_msg->dir = rx_buf[12];
	}
}

/* debug and control part */
uint8_t debug_ctrl_rx_buffer[DEBUG_CTRL_HUART_RX_SIZE];
void debug_ctrl_message_handle(debug_ctrl_msg_t *debug_ctrl_msg, uint8_t *rx_buf)
{

}


/*串口空闲中断通用部分*/
void uart_init_idle(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	HAL_UART_Receive_DMA(huart, pData, Size);
}

uint32_t time_left_uart, time_left_uart_last;
uint32_t time_right_uart, time_right_uart_last;
uint32_t time_imu_uart, time_imu_uart_last;
void uart_receive_handler(UART_HandleTypeDef *huart)
{
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && 
			__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		HAL_UART_DMAStop(huart);
		
		/* wheel message handle */
		if(huart == &WHEEL_L_HUART)
		{
			time_left_uart = HAL_GetTick() - time_left_uart_last;
			time_left_uart_last = HAL_GetTick();
			wheel_message_handle(&wheel_left_msg, wheel_left_uart_rx_buffer);
			HAL_UART_Receive_DMA(huart, wheel_left_uart_rx_buffer, WHEEL_HUART_RX_BUFFER_SIZE);
		}
		else if(huart == &WHEEL_R_HUART)
		{
			time_right_uart = HAL_GetTick() - time_right_uart_last;
			time_right_uart_last = HAL_GetTick();
			wheel_message_handle(&wheel_right_msg, wheel_right_uart_rx_buffer);
//			memset(wheel_left_uart_rx_buffer, 0, sizeof(&wheel_left_uart_rx_buffer));
			HAL_UART_Receive_DMA(huart, wheel_right_uart_rx_buffer, WHEEL_HUART_RX_BUFFER_SIZE);
		}	
//		else if(huart == &IMU_HUART)
//		{
//			time_imu_uart = HAL_GetTick() - time_imu_uart_last;
//			time_imu_uart_last = HAL_GetTick();

//		}
		else{}
	}
}








