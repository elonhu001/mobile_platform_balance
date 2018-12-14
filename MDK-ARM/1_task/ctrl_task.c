#include "sys_config.h"
/*new value to be used*/
param_adjust_t param_adjust;
imu_t imu;


/*for lcd show*/
#define LCD_BASE_X 30
#define LCD_BASE_Y 70
#define LCD_WORD_SIZE 24
#define LCD_LINE_SCAPE 30
char lcd_buf[200];
#define COUNTOF(_BUFFER_) (sizeof(_BUFFER_)/sizeof(*(_BUFFER_)))
/*for imu*/

uint8_t imu_init_ok_flag;

balanc_pid_pd_t balanc_pid_pd;
int16_t chassis_speed_set_value[2];
float mubiao_sudu[2];
/*to get programe excute time*/
uint32_t imu_time_ms;
uint32_t imu_time_last;
/*ramp for big input*/
ramp_t fb_ramp = RAMP_GEN_DAFAULT;
#define INPUT_ACC_TIME     500  //ms

extern UART_HandleTypeDef huart1;


void start_get_imu_task(void const * argument)
{
	led_all_off();
	MPU_Init();//初始化MPU6050
	while(mpu_dmp_init());//innit dmp
	imu_init_ok_flag = 1;
	chassis_pid_init();
	osDelay(3000);
	led_blink(LED0, 1, 0);
//	ramp_init(&fb_ramp, INPUT_ACC_TIME/GET_IMU_TASK_PERIOD);
  uint32_t get_imu_task_wake_time = osKernelSysTick();
  while(1)
  {
//		taskENTER_CRITICAL();
//		if(mpu_dmp_get_data(&imu.pitch,&imu.roll,&imu.yaw)==0)
//    {
//			imu.temp=MPU_Get_Temperature();	              //得到温度值
//			MPU_Get_Accelerometer((short *)&imu.aacx,(short *)&imu.aacy,(short *)&imu.aacz);	//得到加速度传感器数据
//			MPU_Get_Gyroscope((short *)&imu.gyrox,(short *)&imu.aacy,(short *)&imu.aacz);	//得到陀螺仪数据
//    }
//		taskEXIT_CRITICAL();
//		
//		param_adjust.balance_out = pid_calc(&pid_balcance, imu.pitch, balanc_pid_pd.middle);
////		param_adjust.speed_out = pid_calc(&pid_move_speed, imu.pitch, balanc_pid_pd.middle);


//		if(balanc_pid_pd.out > 3000)
//		{
//			balanc_pid_pd.out = balanc_pid_pd.out * ramp_calc(&fb_ramp);
//		}
				
		
		
		
		
//		imu_time_last = HAL_GetTick();
//		

//		mubiao_sudu[WHEEL_L] = balanc_pid_pd.out;
//		mubiao_sudu[WHEEL_R] = -balanc_pid_pd.out;
		chassis_speed_set_value[WHEEL_L] = (int16_t)balanc_pid_pd.out;
		chassis_speed_set_value[WHEEL_R] =(int16_t) -balanc_pid_pd.out;
////		VAL_LIMIT(mubiao_sudu[WHEEL_L], -4000, 4000);
////		VAL_LIMIT(mubiao_sudu[WHEEL_R], -4000, 4000);
//		for (int i = 0; i < 2; i++)
//		{
//			chassis_speed_set_value[i] = pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, mubiao_sudu[i]);
//		}
//		VAL_LIMIT(chassis_speed_set_value[WHEEL_L], -4000, 4000);
//		VAL_LIMIT(chassis_speed_set_value[WHEEL_R], -4000, 4000);
		

		send_chassis_cur(chassis_speed_set_value);
		
		pid_balcance.p = balanc_pid_pd.kp;
		pid_balcance.d = balanc_pid_pd.kd;
		
//		sprintf(lcd_buf, "pitch: %5.3f  mid: %5.3f  kp: %5.3f  kd: %5.3f  out:%7.3f  \n\n", imu.pitch, balanc_pid_pd.middle, balanc_pid_pd.kp, balanc_pid_pd.kd, mubiao_sudu[1]);
//		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)lcd_buf, COUNTOF(lcd_buf) - 1);

	
		
		
//		osDelayUntil(&get_imu_task_wake_time, GET_IMU_TASK_PERIOD);
		osDelay(5);
		imu_time_ms = HAL_GetTick() - imu_time_last;
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

void chassis_pid_init(void)
{
	param_adjust.balance_kp = 200.0f;
	param_adjust.balance_kd = 0.0f;
	param_adjust.speed_kp = 0;
	param_adjust.speed_ki = 0;
	param_adjust.mid_angle = 0;
  for (int k = 0; k < 2; k++)
  {
    PID_struct_init(&pid_spd[k], POSITION_PID, 10000, 500, 20.5f, 0.05, 0);
  }  
	PID_struct_init(&pid_balcance, POSITION_PID, 4000, 500, param_adjust.balance_kp, 0, param_adjust.balance_kd);
	PID_struct_init(&pid_move_speed, POSITION_PID, 4000, 500, param_adjust.speed_kp, 0, param_adjust.speed_ki);
}


/**********************************************control part*************************************************/
/*to get programe excute time*/
uint32_t _time_ms;
uint32_t imu_time_last;
void start_ctrl_task(void const * argument)
{
	chassis_pid_init();
  uint32_t ctrl_task_wake_time = osKernelSysTick();
  while(1)
  {
		balance_pid_calc(imu.pitch, imu.gyroy);
		mubiao_sudu[WHEEL_L] = -balanc_pid_pd.out;
		mubiao_sudu[WHEEL_R] = balanc_pid_pd.out;
//		VAL_LIMIT(mubiao_sudu[WHEEL_L], -10000, 10000);
//		VAL_LIMIT(mubiao_sudu[WHEEL_R], -10000, 10000);
//		send_chassis_cur(mubiao_sudu);
		for (int i = 0; i < 2; i++)
		{
			chassis_speed_set_value[i] = pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, mubiao_sudu[i]);
		}
		VAL_LIMIT(chassis_speed_set_value[WHEEL_L], -8000, 8000);
		VAL_LIMIT(chassis_speed_set_value[WHEEL_R], -8000, 8000);
		send_chassis_cur(chassis_speed_set_value);
		osDelayUntil(&ctrl_task_wake_time, CTRL_TASK_PERIOD);
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

uint8_t key_flag;
int8_t modify_option;
//uint8_t songshou = 1;
void start_key_scan_task(void const * argument)
{
  uint32_t key_scan_task_wake_time = osKernelSysTick();
  while(1)
  {
		key_flag = key_scan(0);
		
//		if(songshou && HAL_GPIO_ReadPin(KEY_WK_UP_GPIO_Port,KEY_WK_UP_Pin))
//		{
//			songshou=0;
//			osDelay(10);
//			if(HAL_GPIO_ReadPin(KEY_WK_UP_GPIO_Port,KEY_WK_UP_Pin))
//			{
//				key_flag = WKUP_PRES;
//			}
//		}
//		else{key_flag = 0;songshou=1;}
		
		
		switch (key_flag)
		{
			case WKUP_PRES: modify_option--;if(modify_option < 0)modify_option = 2;break;
			case KEY1_PRES: modify_option++;if(modify_option > 2)modify_option = 0;break;
			case KEY2_PRES: 
			{
				if(modify_option == 0)
				{
					balanc_pid_pd.kp += 10.0f;
					balanc_pid_pd.out += 10.0f;
				}
				else if(modify_option == 1)
				{
					balanc_pid_pd.kd += 1.0f;
				}
				else if(modify_option == 2)
				{
					balanc_pid_pd.middle += 0.1f;
				}
				else{}
			}break;
			case KEY0_PRES: 
			{
				if(modify_option == 0)
				{
					balanc_pid_pd.kp -= 10.0f;
					balanc_pid_pd.out -= 10.0f;
				}
				else if(modify_option == 1)
				{
					balanc_pid_pd.kd -= 1.0f;
				}
				else if(modify_option == 2)
				{
					balanc_pid_pd.middle -= 0.1f;
				}
				else{}
			}break;
			default:break;
		}

		
//		flash_count ++;
//		if(flash_count == 50)
//		{
//				/*write kp kd into flash*/
//				/* Unlock the Flash to enable the flash control register access *************/
//				HAL_FLASH_Unlock();
//				Address = FLASH_USER_START_ADDR;
//				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, (uint32_t)balanc_pid_pd.kp);
//				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address + 4, (uint32_t)balanc_pid_pd.kd);
//				/* Lock the Flash to disable the flash control register access (recommended
//					 to protect the FLASH memory against possible unwanted operation) *********/
//				HAL_FLASH_Lock();
//				
//				/*read kp kd from flash*/
//				Address = FLASH_USER_START_ADDR;
//				kp_flash = *(__IO uint32_t *)Address;
//				kd_flash = *(__IO uint32_t *)Address + 4;		
//				flash_count = 0;
//		}

		
		osDelayUntil(&key_scan_task_wake_time, KEY_SCAN_TASK_PERIOD);
  }
}

//mode:0,not seq
//mode:1,seq
uint8_t key_scan(uint8_t mode)
{
	static u8 key_up=1;
	if(mode)key_up=1;  
	if(key_up&&(KEY0==0||KEY1==0||KEY2==0||WK_UP==1))
	{
		osDelay(5);
		key_up=0;
		if(KEY0==0)return KEY0_PRES;
		else if(KEY1==0)return KEY1_PRES;
		else if(KEY2==0)return KEY2_PRES;
		else if(WK_UP==1)return WKUP_PRES;
	}else if(KEY0==1&&KEY1==1&&KEY2==1&&WK_UP==0)key_up=1; 	    
 	return 0;
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


/**********************************************lcd scan part*************************************************/


void start_lcd_scan_task(void const * argument)
{

  uint32_t lcd_scan_task_wake_time = osKernelSysTick();
  while(1)
  {
		if(imu_init_ok_flag == 1)
		{
//			uart_send_senser();
			sprintf(lcd_buf, "pitch: %5.3f  mid: %5.3f  kp: %5.3f  kd: %5.3f  out:%7.3f  \n", imu.pitch, balanc_pid_pd.middle, balanc_pid_pd.kp, balanc_pid_pd.kd, mubiao_sudu[1]);
			HAL_UART_Transmit_DMA(&huart1, (uint8_t *)lcd_buf, COUNTOF(lcd_buf) - 1);
			osDelay(500);
//			LCD_ShowString(LCD_BASE_X,(LCD_BASE_Y + LCD_LINE_SCAPE * 1),300,24,24,(u8 *)lcd_buf);
//			sprintf(lcd_buf, "roll: %5.3f", imu.roll);
//			LCD_ShowString(LCD_BASE_X,(LCD_BASE_Y + LCD_LINE_SCAPE * 2),300,24,24,(u8 *)lcd_buf);
//			sprintf(lcd_buf, "yaw: %5.3f", imu.yaw);
//			LCD_ShowString(LCD_BASE_X,(LCD_BASE_Y + LCD_LINE_SCAPE * 3),300,24,24,(u8 *)lcd_buf);

//			sprintf(lcd_buf, "gyrox: %5d", imu.gyrox);
//			LCD_ShowString(LCD_BASE_X,(LCD_BASE_Y + LCD_LINE_SCAPE * 4),300,24,24,(u8 *)lcd_buf);
//			sprintf(lcd_buf, "gyroy: %5d", imu.gyroy);
//			LCD_ShowString(LCD_BASE_X,(LCD_BASE_Y + LCD_LINE_SCAPE * 5),300,24,24,(u8 *)lcd_buf);
//			sprintf(lcd_buf, "gyroz: %5d", imu.gyroz);
//			LCD_ShowString(LCD_BASE_X,(LCD_BASE_Y + LCD_LINE_SCAPE * 6),300,24,24,(u8 *)lcd_buf);

//			sprintf(lcd_buf, "aacx: %5d", imu.aacx);
//			LCD_ShowString(LCD_BASE_X,(LCD_BASE_Y + LCD_LINE_SCAPE * 7),300,24,24,(u8 *)lcd_buf);
//			sprintf(lcd_buf, "aacy: %5d", imu.aacy);
//			LCD_ShowString(LCD_BASE_X,(LCD_BASE_Y + LCD_LINE_SCAPE * 8),300,24,24,(u8 *)lcd_buf);
//			sprintf(lcd_buf, "aacz: %5d", imu.aacz);
//			LCD_ShowString(LCD_BASE_X,(LCD_BASE_Y + LCD_LINE_SCAPE * 9),300,24,24,(u8 *)lcd_buf);
				
//			sprintf(lcd_buf, "temp: %5d", imu.temp);
//			LCD_ShowString(LCD_BASE_X,(LCD_BASE_Y + LCD_LINE_SCAPE * 10),300,24,24,(u8 *)lcd_buf);
//			sprintf(lcd_buf, "  balance_kp: %5.3f", balanc_pid_pd.kp);
//			LCD_ShowString(LCD_BASE_X,(LCD_BASE_Y + LCD_LINE_SCAPE * 11),300,24,24,(u8 *)lcd_buf);
//			sprintf(lcd_buf, "  balance_kd: %5.3f", balanc_pid_pd.kd);
//			LCD_ShowString(LCD_BASE_X,(LCD_BASE_Y + LCD_LINE_SCAPE * 12),300,24,24,(u8 *)lcd_buf);		
//			sprintf(lcd_buf, "  middle: %5.3f", balanc_pid_pd.middle);
//			LCD_ShowString(LCD_BASE_X,(LCD_BASE_Y + LCD_LINE_SCAPE * 13),300,24,24,(u8 *)lcd_buf);	
			
//			switch(modify_option)
//			{
//				case 0:LCD_ShowString(LCD_BASE_X,(LCD_BASE_Y + LCD_LINE_SCAPE * 11),8,24,24,(u8 *)">");break;
//				case 1:LCD_ShowString(LCD_BASE_X,(LCD_BASE_Y + LCD_LINE_SCAPE * 12),8,24,24,(u8 *)">");break;
//				case 2:LCD_ShowString(LCD_BASE_X,(LCD_BASE_Y + LCD_LINE_SCAPE * 13),8,24,24,(u8 *)">");break;
//			  default: break;
//			}
		}

		osDelayUntil(&lcd_scan_task_wake_time, LCD_SCAN_TASK_PERIOD);
  }
}

const double eps = 1e-11;
void float_to_str(char *str,double num)
{
	int high;//float_鏁存暟閮ㄥ垎  
	double low;//float_灏忔暟閮ㄥ垎 
	char *start=str;
	int n=0;
	char ch[20];
	int i;
	high=(int)num;
	low=num-high;

	while(high>0)
	{
		ch[n++]='0'+high%10;
		high=high/10;
	}
	
	for(i=n-1;i>=0;i--)
	{
		*str++=ch[i];
	}
	
	num -= (int)num;
	double tp = 0.1;
	*str++='.';
	
	while(num > eps)
	{//绮惧害闄愬埗 
		num -= tp * (int)(low * 10);
		tp /= 10;
		*str++='0'+(int)(low*10);
		low=low*10.0-(int)(low*10);
	}
	*str='\0';
	str=start;
}


/**********为了匿名四轴上位机的协议定义的变量****************************/
//cup为小端模式存储，也就是在存储的时候，低位被存在0字节，高位在1字节
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))	 //取出int型变量的低字节
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))	 //	取存储在此变量下一内存字节的内容，高字节
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


/******************************************
** 说明：
	1、 发送给上位机的数据帧定义 
		@桢头--功能字--长度--数据（一个或多个，具体看协议说明）-校验
		@前2个字节为帧头0xAAAA 
		@第3个字节为帧ID，也就是功能字，应设置为0xF1~0xFA中的一个 
		@第4个字节为报文数据长度(dlc) 
		@第5个字节开始到第5+dlc-1个字节为要传输的数据内容段，每个数据场为高字节在前，地字节在后 
		@第5+dlc个字节为CheckSum,为第1个字节到第5+dlc-1个字节所有字节的值相加后，保留结果的低八位作为CheckSum 
	2、 外部直接调用这个函数。
	3、 需要在此文件中引用需要发送的其他文件中的数据。
	4、 发送的数据必须是 int_16 型的数据
*****************************************/  
void uart_send_senser(void)
{
	unsigned char   data_to_send[23] = {0};
	unsigned char i = 0;
	unsigned char cnt = 0;
	unsigned char sum = 0;
 
	int int_set_distance_2 = (int)100;
	int int_real_distance = (int)200;
	int int_ASR_output = (int)300;
	
 
	data_to_send[cnt++]=0xAA;	 //帧头：AAAA
	data_to_send[cnt++]=0xAA;
	data_to_send[cnt++]=0x02;	 //功能字：OXFn只接受数据，不显示图像。0x0n显示数据和图像
	data_to_send[cnt++]=0;	     //需要发送数据的字节数，暂时给0，后面在赋值。
 
	data_to_send[cnt++] = BYTE1(int_set_distance_2);	//高字节
	data_to_send[cnt++] = BYTE0(int_set_distance_2);	//低字节
	data_to_send[cnt++] = BYTE1(int_real_distance);
	data_to_send[cnt++] = BYTE0(int_real_distance);
	data_to_send[cnt++] = BYTE1(int_ASR_output);
	data_to_send[cnt++] = BYTE0(int_ASR_output);
 
	data_to_send[cnt++] = 0;
	data_to_send[cnt++] = 0;
	data_to_send[cnt++] = 0;
	data_to_send[cnt++] = 0;
	data_to_send[cnt++] = 0;
	data_to_send[cnt++] = 0;
 
	data_to_send[cnt++] = 0;
	data_to_send[cnt++] = 0;
	data_to_send[cnt++] = 0;
	data_to_send[cnt++] = 0;
	data_to_send[cnt++] = 0;
	data_to_send[cnt++] = 0;
 
	data_to_send[3] = cnt-4;//计算总数据的字节数。
 
	for(i=0;i<cnt;i++) //对于for语句，当不写大括号的时候，只执行到下面第一个分号结束。
	{
		sum+=data_to_send[i];
	}
 
	data_to_send[cnt++] = sum;	//计算校验位
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)data_to_send, COUNTOF(data_to_send) - 1);
}


