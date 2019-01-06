/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_can.c
 * @brief      this file contains sd card basic operating function
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-23-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#include "bsp_can.h"
#include "can.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "sys_config.h"

//extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

uint8_t can1_rx_data[CAN_RX_FIFO_SIZE];
moto_measure_t moto_chassis[2];

/**
  * @brief  Configures the CAN, transmit and receive by polling
  * @param  None
  * @retval PASSED if the reception is well done, FAILED in other case
  */
HAL_StatusTypeDef can_filter_init(CAN_HandleTypeDef* hcan)
{
  CAN_FilterTypeDef  sFilterConfig;

  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
//  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  
//	if(hcan == &hcan1)
//	{
//	  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
//		sFilterConfig.FilterBank = 0;
//	}
	if(hcan == &hcan2)
	{
	  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
		sFilterConfig.FilterBank = 14;
	}
	
  if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  /*##-3- Start the CAN peripheral ###########################################*/
  if (HAL_CAN_Start(hcan) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
	
  /*##-4- Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }

	return HAL_OK;
}

/**
  * @brief     get motor initialize offset value
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after system can init
  */
void get_moto_offset(moto_measure_t* ptr, uint8_t* data)
{
    ptr->ecd        = (uint16_t)(data[0] << 8 | data[1]);
    ptr->offset_ecd = ptr->ecd;
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  __HAL_CAN_ENABLE_IT(hcan, CAN_IT_ERROR_WARNING |   \
														CAN_IT_ERROR_PASSIVE |   \
														CAN_IT_BUSOFF |          \
														CAN_IT_LAST_ERROR_CODE | \
														CAN_IT_ERROR);
}

/**
  * @brief     get motor rpm and calculate motor round_count/total_encoder/total_angle
  * @param     ptr: Pointer to a moto_measure_t structure
  * @attention this function should be called after get_moto_offset() function
  */
void encoder_data_handler(moto_measure_t* ptr, uint8_t* data)
{
  ptr->last_ecd = ptr->ecd;
  ptr->ecd      = (uint16_t)(data[0] << 8 | data[1]);
  
  if (ptr->ecd - ptr->last_ecd > 4096)
  {
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
  else if (ptr->ecd - ptr->last_ecd < -4096)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  /* total angle, unit is degree */
  ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;

  ptr->speed_rpm     = (int16_t)(data[2] << 8 | data[3]);
  ptr->given_current = (int16_t)(data[4] << 8 | data[5]);
	ptr->acc_speed_rad = (ptr->speed_rpm - ptr->speed_rpm_last) * (1000 / 60);
	ptr->speed_rpm_last = ptr->speed_rpm;
}

uint32_t time_can, time_can_last;
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef   hcan1_rx_header;
	
  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &hcan1_rx_header, can1_rx_data) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }
	if((hcan1_rx_header.StdId == CAN_3508_M1_ID) || \
		 (hcan1_rx_header.StdId == CAN_3508_M2_ID) || \
		 (hcan1_rx_header.StdId == CAN_3508_M3_ID) || \
		 (hcan1_rx_header.StdId == CAN_3508_M4_ID))
	{
		
		time_can = HAL_GetTick() - time_can_last;
		time_can_last = HAL_GetTick();
		static uint8_t i;
		i = hcan1_rx_header.StdId - CAN_3508_M1_ID;

		moto_chassis[i].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[i], can1_rx_data) : encoder_data_handler(&moto_chassis[i], can1_rx_data);
	}
	 __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}



/**
  * @brief  send calculated current to motor
  * @param  3508 motor ESC id
  */
void send_chassis_cur(int16_t *mot_current)
{
	CAN_TxHeaderTypeDef CHASSIS_CAN;
	uint8_t can1_tx_data[CAN_TX_FIFO_SIZE];
	uint32_t              TxMailbox;
	
  CHASSIS_CAN.StdId   = 0x200;
  CHASSIS_CAN.IDE     = CAN_ID_STD;
  CHASSIS_CAN.RTR     = CAN_RTR_DATA;
  CHASSIS_CAN.DLC     = 0x08;
	CHASSIS_CAN.TransmitGlobalTime = DISABLE;
  can1_tx_data[0] = mot_current[0] >> 8;
  can1_tx_data[1] = mot_current[0];
  can1_tx_data[2] = mot_current[1] >> 8;
  can1_tx_data[3] = mot_current[1];
  can1_tx_data[4] = mot_current[2] >> 8;
  can1_tx_data[5] = mot_current[2];
  can1_tx_data[6] = mot_current[3] >> 8;
  can1_tx_data[7] = mot_current[3];
	
  /* Request transmission */
  if(HAL_CAN_AddTxMessage(&hcan2, &CHASSIS_CAN, can1_tx_data, &TxMailbox) != HAL_OK)
  {
    /* Transmission request Error */
    Error_Handler();
  }
  
  /* Wait transmission complete */
  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) != 3) {}
}




