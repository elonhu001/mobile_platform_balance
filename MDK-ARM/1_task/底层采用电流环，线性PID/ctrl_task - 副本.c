#include "sys_config.h"
/*new value to be used*/
imu_t imu;
sys_variable_t sys_variable;
/*for param show*/
char lcd_buf[300];
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/*for imu*/
uint8_t imu_init_ok_flag;

//balanc_pid_pd_t balanc_pid_pd;

/*ramp for big input*/
//ramp_t fb_ramp = RAMP_GEN_DAFAULT;
//#define INPUT_ACC_TIME     500  //ms
//	ramp_init(&fb_ramp, INPUT_ACC_TIME/GET_IMU_TASK_PERIOD);
balance_pid_t balance_pid;
extern UART_HandleTypeDef huart1;
int16_t test_acc;
/*to get task  period*/
uint32_t imu_time_ms;
uint32_t imu_time_last;

uint16_t test_current = 700;
void start_get_imu_task(void const * argument)
{
	uint8_t speed_circle_cnt;
	led_all_off();
	MPU_Init();//初始化MPU6050
	while(mpu_dmp_init());//innit dmp
	imu_init_ok_flag = 1;
	chassis_pid_init();
	osDelay(3000);
	led_blink(LED0, 1, 0);

  uint32_t get_imu_task_wake_time = osKernelSysTick();
  while(1)
  {
		imu_time_ms = HAL_GetTick() - imu_time_last;
		imu_time_last = HAL_GetTick();
//		taskENTER_CRITICAL();
		mpu_dmp_get_data(&imu.pitch,&imu.roll,&imu.yaw);
		MPU_Get_Accelerometer(&imu.accel[0],&imu.accel[1],&imu.accel[2]);	//得到加速度传感器数据
		MPU_Get_Gyroscope(&imu.gyro[0],&imu.gyro[1],&imu.gyro[2]);	//得到陀螺仪数据
//		imu.temp=MPU_Get_Temperature();	              //得到温度值
		sys_variable.theta = imu.pitch;
//		taskEXIT_CRITICAL();
		
		/*balance circle | current circle*/
//		sys_variable.balance_out = pid_calc(&pid_balcance, sys_variable.theta, sys_variable.mid_angle);
		sys_variable.balance_out = balance_pid_calc(imu.pitch, sys_variable.mid_angle, imu.gyro[1]);
		if(sys_variable.balance_out > 0)
		{
			sys_variable.balance_out += sys_variable.deat_balance_out;
		}
		else if(sys_variable.balance_out < 0)
		{
			sys_variable.balance_out -= sys_variable.deat_balance_out;
		}
		else
		{
			sys_variable.balance_out = 0;
		}
		
		/*speed circle*/
		speed_circle_cnt++;
		sys_variable.vl = moto_chassis[WHEEL_L].speed_rpm;//左轮方向正确
		sys_variable.vr = -moto_chassis[WHEEL_R].speed_rpm;//右轮方向相反
		sys_variable.v = (sys_variable.vl + sys_variable.vr)/2.0f;
		if(speed_circle_cnt == 10)
		{
			speed_circle_cnt= 0 ;
			sys_variable.speed_out = pid_calc(&pid_move_speed, sys_variable.v, sys_variable.set_v_speed);//最后一个参数是目标速度，这里暂时不用，给0		     
		}
		/*direction circle*/
		sys_variable.turn_out = pid_calc(&pid_turn, 0, 0);		

		sys_variable.totall_out[WHEEL_L] = (int16_t)sys_variable.balance_out + sys_variable.speed_out + sys_variable.turn_out;
		sys_variable.totall_out[WHEEL_R] = (int16_t)-sys_variable.balance_out - sys_variable.speed_out + sys_variable.turn_out;

		VAL_LIMIT(sys_variable.totall_out[WHEEL_L], -MAX_WHEEL_RPM, MAX_WHEEL_RPM);
		VAL_LIMIT(sys_variable.totall_out[WHEEL_R], -MAX_WHEEL_RPM, MAX_WHEEL_RPM);
		send_chassis_cur(sys_variable.totall_out);
//		sys_variable.acc_get = (moto_chassis[0].acc_speed_rad + moto_chassis[1].acc_speed_rad) / 2.0f;
////		sys_variable.acc_set = sys_variable.balance_out;
////		for (int i=0; i<2; i++)                   
////		{
////			 sys_variable.totall_out[i] = (int16_t)pid_calc(&pid_motor_speed[i], moto_chassis[i].speed_rpm, sys_variable.totall_out[i]);
//		
//			 sys_variable.totall_out[0] = (int16_t)pid_calc(&pid_motor_acc_speed[0], moto_chassis[0].acc_speed_rad, sys_variable.acc_set);
//				sys_variable.totall_out[1] = (int16_t)pid_calc(&pid_motor_speed[1], moto_chassis[1].speed_rpm, sys_variable.speed_set);
////		}




////			sprintf(lcd_buf, "acc_speed_rad: %7d  \n", moto_chassis[0].acc_speed_rad);
//////		sprintf(lcd_buf, "pitch: %5.3f  mid: %5.3f  kp: %5.3f  kd: %5.3f  deat_out: %7.3f  out:%6d  \n", sys_variable.theta, sys_variable.mid_angle, sys_variable.balance_kp, sys_variable.balance_kd, sys_variable.deat_balance_out, sys_variable.totall_out[WHEEL_L]);
////			HAL_UART_Transmit_DMA(&huart1, (uint8_t *)lcd_buf, (COUNTOF(lcd_buf) - 1));

			
		osDelayUntil(&get_imu_task_wake_time, GET_IMU_TASK_PERIOD);
		
  }
}

uint16_t count_buf(uint8_t *buf)
{
	int16_t i = 0;
	while(buf[i] != '\0')
	{
		i++;
	}
	return i;
}


float balance_pid_calc(float get_pitch, float set_pitch, int16_t gyro_y)
{
	balance_pid.get_pitch = get_pitch;
	balance_pid.set_pitch = set_pitch;
	balance_pid.gyro_y = gyro_y;
	balance_pid.err = balance_pid.set_pitch - balance_pid.get_pitch;
	balance_pid.p_out = balance_pid.kp * balance_pid.err;
	balance_pid.d_out = -balance_pid.kd * balance_pid.gyro_y;
	balance_pid.out = balance_pid.p_out + balance_pid.d_out; 
	return balance_pid.out;
}

void chassis_pid_init(void)
{
	sys_variable.balance_kp = 209.0f;
	sys_variable.balance_kd = 2.0f;                                                                                    
	
	sys_variable.speed_kp = 0;
	sys_variable.speed_ki = 0;
	
	
	balance_pid.kp = 801;
	balance_pid.kd = -2.39;
	sys_variable.mid_angle = -0.6f;
	sys_variable.deat_balance_out = 0;
	for (int i=0; i<2; i++)                   
	{
		PID_struct_init(&pid_motor_speed[i], POSITION_PID, 8000, 500, 20.5, 0.05, 0);
		PID_struct_init(&pid_motor_acc_speed[i], POSITION_PID, 8000, 500, 0, 0, 0);
	}
	
	PID_struct_init(&pid_balcance, POSITION_PID, 8000, 0, sys_variable.balance_kp, 0, sys_variable.balance_kd);
	PID_struct_init(&pid_move_speed, POSITION_PID, 8000, 500, sys_variable.speed_kp, sys_variable.speed_ki, 0);
	PID_struct_init(&pid_turn, POSITION_PID, 8000, 0, sys_variable.turn_kp, 0, 0);
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
/*to get programe excute time*/
uint32_t _time_ms;
uint32_t imu_time_last;
void start_ctrl_task(void const * argument)
{
	chassis_pid_init();
  uint32_t ctrl_task_wake_time = osKernelSysTick();
  while(1)
  {
//		balance_pid_calc(imu.pitch, imu.gyroy);
//		mubiao_sudu[WHEEL_L] = -balanc_pid_pd.out;
//		mubiao_sudu[WHEEL_R] = balanc_pid_pd.out;
////		VAL_LIMIT(mubiao_sudu[WHEEL_L], -10000, 10000);
////		VAL_LIMIT(mubiao_sudu[WHEEL_R], -10000, 10000);
////		send_chassis_cur(mubiao_sudu);
//		for (int i = 0; i < 2; i++)
//		{
//			chassis_speed_set_value[i] = pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, mubiao_sudu[i]);
//		}
//		VAL_LIMIT(chassis_speed_set_value[WHEEL_L], -8000, 8000);
//		VAL_LIMIT(chassis_speed_set_value[WHEEL_R], -8000, 8000);
//		send_chassis_cur(chassis_speed_set_value);
//		osDelayUntil(&ctrl_task_wake_time, CTRL_TASK_PERIOD);
  }
}




/**********************************************key scan part*************************************************/
uint8_t key_flag;
int8_t modify_option;
void start_key_scan_task(void const * argument)
{
  uint32_t key_scan_task_wake_time = osKernelSysTick();
  while(1)
  {
		key_flag = key_scan(0);
		
		switch (key_flag)
		{
			case WKUP_PRES: modify_option--;if(modify_option < 0)modify_option = 6;break;
			case KEY1_PRES: modify_option++;if(modify_option > 6)modify_option = 0;break;
			case KEY2_PRES: 
			{
				if(modify_option == 0)
				{
					balance_pid.kp += 1.0f;
				}
				else if(modify_option == 1)
				{
					balance_pid.kd += 0.01f;
				}
				else if(modify_option == 2)
				{
					sys_variable.mid_angle += 0.1f;
				}
				else if(modify_option == 3)
				{
				  sys_variable.deat_balance_out += 5;
				}
				else if(modify_option == 4)
				{
				  pid_move_speed.p += 1.0f;
				}
				else if(modify_option == 5)
				{
				  pid_move_speed.i += 0.01f;
				}
				else if(modify_option == 6)
				{
				  sys_variable.set_v_speed += 5;
				}
				else{}
			}break;
			case KEY0_PRES: 
			{
				if(modify_option == 0)
				{
					balance_pid.kp -= 1.0f;
				}
				else if(modify_option == 1)
				{
					balance_pid.kd -= 0.01f;
				}
				else if(modify_option == 2)
				{
					sys_variable.mid_angle -= 0.1f;
				}
				else if(modify_option == 3)
				{
				  sys_variable.deat_balance_out -= 5;
				}
				else if(modify_option == 4)
				{
				  pid_move_speed.p -= 1.0f;
				}
				else if(modify_option == 5)
				{
				  pid_move_speed.i -= 0.01f;
				}
				else if(modify_option == 6)
				{
				  sys_variable.set_v_speed -= 5;
				}
				else{}
			}break;
			default:break;
		}

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



/**********************************************lcd scan part*************************************************/

/*to get task  period*/
uint32_t msg_send_time_ms, msg_send_time_last;
uint32_t send_cnt;
void start_lcd_scan_task(void const * argument)
{

  uint32_t lcd_scan_task_wake_time = osKernelSysTick();
  while(1)
  {
		msg_send_time_ms = HAL_GetTick() - msg_send_time_last;
		msg_send_time_last = HAL_GetTick();
		if(imu_init_ok_flag == 1)
		{	
			send_cnt++;
//			sprintf(lcd_buf, "$ID: %5d pitch: %5.3f  gyro: %5.3f  out:%6d  $$\n", send_cnt, imu.pitch, (float)imu.gyro[1], sys_variable.totall_out[WHEEL_L]);
//			HAL_UART_Transmit_DMA(&huart1, (uint8_t *)lcd_buf, count_buf((uint8_t *)lcd_buf));
//			osDelay(5);
//			sprintf(lcd_buf, "$mid: %5.3f  kp: %5.3f  kd: %5.3f  deat_out: %7.3f  $$\n",sys_variable.mid_angle, balance_pid.kp, balance_pid.kd, sys_variable.deat_balance_out);
			sprintf(lcd_buf, "$ID: %5d   modify_option %1d   pitch: %5.3f   balance_kp: %5.3f   balance_kd: %5.3f   mid: %5.3f   deat_out: %7.3f   speed_kp: %5.3f   speed_kd: %5.3f   speed_set: %5.3f   out:%6d  $END\n",     \
			                   send_cnt, modify_option, sys_variable.theta, balance_pid.kp, balance_pid.kd, sys_variable.mid_angle, sys_variable.deat_balance_out, pid_move_speed.p, pid_move_speed.d, sys_variable.set_v_speed, sys_variable.totall_out[WHEEL_L]);
			HAL_UART_Transmit_DMA(&huart1, (uint8_t *)lcd_buf, count_buf((uint8_t *)lcd_buf));
			if(send_cnt == 60000)
			{
				send_cnt = 0;
			}
		}

		osDelayUntil(&lcd_scan_task_wake_time, LCD_SCAN_TASK_PERIOD);
  }
}
