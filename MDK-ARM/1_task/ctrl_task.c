#include "sys_config.h"

imu_t imu;
void start_get_imu_task(void const * argument)
{
	led_all_off();
	MPU_Init();//³õÊ¼»¯MPU6050
  while(mpu_dmp_init());//innit dmp
	led_blink(LED0, 1, 0);
  uint32_t get_imu_task_wake_time = osKernelSysTick();
  while(1)
  {
		if(mpu_dmp_get_data(&imu.pitch,&imu.roll,&imu.yaw)==0)
    {
			imu.temp=MPU_Get_Temperature();	              //µÃµ½ÎÂ¶ÈÖµ
			MPU_Get_Accelerometer((short *)&imu.aacx,(short *)&imu.aacy,(short *)&imu.aacz);	//µÃµ½¼ÓËÙ¶È´«¸ÐÆ÷Êý¾Ý
			MPU_Get_Gyroscope((short *)&imu.gyrox,(short *)&imu.aacy,(short *)&imu.aacz);	//µÃµ½ÍÓÂÝÒÇÊý¾Ý
    }
		osDelayUntil(&get_imu_task_wake_time, GET_IMU_TASK_PERIOD);
  }
}

void led_all_off(void)
{
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
}

void led_blink(LED led, uint8_t time, uint16_t delay_time)
{
	uint8_t i;
	if(led == LED0)
	{
		for(i = 0; i < time; i++)
		{
			HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
			osDelay(200);
			HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
			osDelay(200);
		}
		HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
		osDelay(1000);	
	}
	else if(led == LED1)
	{
		for(i = 0; i < time; i++)
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			osDelay(200);
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
			osDelay(200);
		}
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		osDelay(delay_time);	
	}
}

/**********************************************control part*************************************************/
balanc_pid_pd_t balanc_pid_pd;
int16_t chassis_speed_set_value[2];
int16_t mubiao_sudu[2];
void start_ctrl_task(void const * argument)
{
	chassis_pid_init();
  uint32_t ctrl_task_wake_time = osKernelSysTick();
  while(1)
  {
		balance_pid_calc(imu.pitch, imu.gyroy);
		mubiao_sudu[WHEEL_L] = -balanc_pid_pd.out;
		mubiao_sudu[WHEEL_R] = balanc_pid_pd.out;

		for (int i = 0; i < 2; i++)
		{
			chassis_speed_set_value[i] = pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, mubiao_sudu[i]);
		}
		VAL_LIMIT(chassis_speed_set_value[WHEEL_L], -10000, 10000);
		VAL_LIMIT(chassis_speed_set_value[WHEEL_R], -10000, 10000);
		send_chassis_cur(chassis_speed_set_value);
		osDelayUntil(&ctrl_task_wake_time, CTRL_TASK_PERIOD);
  }
}

void chassis_pid_init(void)
{
	balanc_pid_pd.kp = 200;
	balanc_pid_pd.kd = 0.0f;
	balanc_pid_pd.middle = 0;
	
  for (int k = 0; k < 2; k++)
  {
    PID_struct_init(&pid_spd[k], POSITION_PID, 10000, 500, 20.5f, 0.05, 0);
  }
}
void balance_pid_calc(float angle_pitch, float gyro_pitch)
{
	balanc_pid_pd.bias = angle_pitch - balanc_pid_pd.middle;
	balanc_pid_pd.out = balanc_pid_pd.kp * balanc_pid_pd.bias + balanc_pid_pd.kd * gyro_pitch;
}


/**********************************************key scan part*************************************************/
uint32_t FirstSector = 0, NbOfSectors = 0;
uint32_t Address = 0, SECTORError = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;
/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;
float kp_flash, kd_flash;
uint8_t flash_count;
void start_key_scan_task(void const * argument)
{
  uint32_t key_scan_task_wake_time = osKernelSysTick();
  while(1)
  {
		switch (key_scan(0))
		{
			case WKUP_PRES: balanc_pid_pd.kp += 10.0f; led_blink(LED0, 1, 0);break;
			case KEY1_PRES: balanc_pid_pd.kp -= 10.0f; led_blink(LED0, 1, 0);break;
			case KEY2_PRES: balanc_pid_pd.kd += 1.0f; led_blink(LED0, 1, 0);break;
			case KEY0_PRES: balanc_pid_pd.kd -= 1.0f; led_blink(LED0, 1, 0);break;
			default:break;
		}
		flash_count ++;
		if(flash_count == 50)
		{
				/*write kp kd into flash*/
				/* Unlock the Flash to enable the flash control register access *************/
				HAL_FLASH_Unlock();
				Address = FLASH_USER_START_ADDR;
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, (uint32_t)balanc_pid_pd.kp);
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address + 4, (uint32_t)balanc_pid_pd.kd);
				/* Lock the Flash to disable the flash control register access (recommended
					 to protect the FLASH memory against possible unwanted operation) *********/
				HAL_FLASH_Lock();
				
				/*read kp kd from flash*/
				Address = FLASH_USER_START_ADDR;
				kp_flash = *(__IO uint32_t *)Address;
				kd_flash = *(__IO uint32_t *)Address + 4;		
				flash_count = 0;
		}

		
		osDelayUntil(&key_scan_task_wake_time, KEY_SCAN_TASK_PERIOD);
  }
}

//mode:0,not seq
//mode:1,seq
uint8_t key_scan(uint8_t mode)
{
	static u8 key_up=1;//Â°Â´Â¼Ã¼Â°Â´Ã‹Ã‰Â¿ÂªÂ±ÃªÃ–Â¾
	if(mode)key_up=1;  //Ã–Â§Â³Ã–ÃÂ¬Â°Â´		  
	if(key_up&&(KEY0==0||KEY1==0||KEY2==0||WK_UP==1))
	{
		delay_ms(10);//ÃˆÂ¥Â¶Â¶Â¶Â¯ 
		key_up=0;
		if(KEY0==0)return KEY0_PRES;
		else if(KEY1==0)return KEY1_PRES;
		else if(KEY2==0)return KEY2_PRES;
		else if(WK_UP==1)return WKUP_PRES;
	}else if(KEY0==1&&KEY1==1&&KEY2==1&&WK_UP==0)key_up=1; 	    
 	return 0;// ÃŽÃžÂ°Â´Â¼Ã¼Â°Â´ÃÃ‚
}

/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7) */
  {
    sector = FLASH_SECTOR_7;
  }
  return sector;
}

/**
  * @brief  Gets sector Size
  * @param  None
  * @retval The size of a given sector
  */
static uint32_t GetSectorSize(uint32_t Sector)
{
  uint32_t sectorsize = 0x00;
  if((Sector == FLASH_SECTOR_0) || (Sector == FLASH_SECTOR_1) || (Sector == FLASH_SECTOR_2) || (Sector == FLASH_SECTOR_3))
  {
    sectorsize = 16 * 1024;
  }
  else if(Sector == FLASH_SECTOR_4)
  {
    sectorsize = 64 * 1024;
  }
  else
  {
    sectorsize = 128 * 1024;
  }  
  return sectorsize;
}

