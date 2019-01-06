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

extern UART_HandleTypeDef huart1;
int16_t test_acc;
/*to get task  period*/
uint32_t imu_time_ms;
uint32_t imu_time_last;

int16_t test_current = 0;
uint8_t wheel_ccmd[5];




typedef struct
{
	/* 驱动器采集的参数 */
  uint32_t ch;//采集通道（TIM3＿ 
  uint32_t capt_buf[3];//记录跳变时间，用于计算周期，频率
  int16_t  capt_cnt; //记录跳变次数，用于计算转动角度，圈数
  float    frequncy;//频率
  float    cycle_time;//周期
	uint32_t dir_buf[2];
	/* 电机参数 */
	uint8_t  level_chg_flag;
	uint8_t  wheel_type;//电机名称
	uint8_t  pole_num;//极对敿
	uint8_t  level_num;//丿圈的电平转换次敿
	dir_e    dir;//当前转动方向＿-1为负向，0为不转，+1为正吿
	float    speed_rpm;//转鿟，前进为正，后鿿为贿
	float    speed_rpm_last;
  int32_t   round_cnt;//圈数，前进为正圈数，后鿿为负圈数
}motor_t;

#define MOTOR_LEFT_DEFAULT  {TIM_CHANNEL_1, {0}, 0, 0, 0, {0}, 0, WHEEL_L, 30, 90, DIR_NULL, 0, 0}
#define MOTOR_RIGHT_DEFAULT {TIM_CHANNEL_2, {0}, 0, 0, 0, {0}, 0, WHEEL_R, 30, 90, DIR_NULL, 0, 0}
#define MAX_SPEED (3000)
#define ERR_SPEED (1000)
motor_t motor_left = MOTOR_LEFT_DEFAULT;
motor_t motor_right = MOTOR_RIGHT_DEFAULT;


/**********************************************************************************************/
balance_pid_t balance_pid;
speed_t speed;
turn_t turn;
static void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,float duty)
{
	switch(tim_channel)
	{	
		case TIM_CHANNEL_1: tim->Instance->CCR1 = (PWM_RESOLUTION*duty) - 1;break;
		case TIM_CHANNEL_2: tim->Instance->CCR2 = (PWM_RESOLUTION*duty) - 1;break;
		case TIM_CHANNEL_3: tim->Instance->CCR3 = (PWM_RESOLUTION*duty) - 1;break;
		case TIM_CHANNEL_4: tim->Instance->CCR4 = (PWM_RESOLUTION*duty) - 1;break;
	}
}

motor_msg_t motor_msg_left = MOTOR_MSG_LEFT_DEFAULT;
motor_msg_t motor_msg_right = MOTOR_MSG_RIGHT_DEFAULT;

/*-223---+223rpm/min*/
static void motor_ctrl(motor_msg_t *motor, float speed)
{
	float duty;
	if(motor->wheel_type  == WHEEL_L)
	{
		if(speed > 0)//正向
		{
			HAL_GPIO_WritePin(DIRECTION_L_GPIO_Port, DIRECTION_L_Pin, GPIO_PIN_RESET);//低电平有敿
			duty = speed / 5000.0f;
		}
		else if(speed < 0)
		{
			HAL_GPIO_WritePin(DIRECTION_L_GPIO_Port, DIRECTION_L_Pin, GPIO_PIN_SET);
			duty = -speed / 5000.0f;
		}
		else
		{
			duty = 0;
		}
		PWM_SetDuty(&htim2, motor->ch, duty);	
	}
	else if(motor->wheel_type == WHEEL_R)
	{
		if(speed > 0)//正向
		{
			HAL_GPIO_WritePin(DIRECTION_R_GPIO_Port, DIRECTION_R_Pin, GPIO_PIN_RESET);//低电平正吿
			duty = speed / 5000.0f;
		}
		else if(speed < 0)
		{
			HAL_GPIO_WritePin(DIRECTION_R_GPIO_Port, DIRECTION_R_Pin, GPIO_PIN_SET);
			duty = -speed / 5000.0f;
		}
		else
		{
			duty = 0;
		}
		PWM_SetDuty(&htim2, motor->ch, duty);	
	}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM3)
    {
			if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
			{
				if(motor_msg_left.encoder.dir == DIR_POS)
				{
					motor_msg_left.circle_cnt++;
				}
				else if(motor_msg_left.encoder.dir == DIR_NEG)
				{
					motor_msg_left.circle_cnt--;
				}else{}
			}
			/* 左轮电机方向参数更新HA */
			if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
			{
				if(motor_msg_left.encoder.dir == DIR_POS)
				{
					motor_msg_left.circle_cnt++;
				}
				else if(motor_msg_left.encoder.dir == DIR_NEG)
				{
					motor_msg_left.circle_cnt--;
				}else{}
			}
    }
		if(htim->Instance == TIM4)
		{
			/* 右轮电机速度参数更新 */
			if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
			{
				if(motor_msg_right.encoder.dir == DIR_POS)
				{
					motor_msg_right.circle_cnt++;
				}
				else if(motor_msg_right.encoder.dir == DIR_NEG)
				{
					motor_msg_right.circle_cnt--;
				}else{}
			}
			if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
			{
				if(motor_msg_right.encoder.dir == DIR_POS)
				{
					motor_msg_right.circle_cnt++;
				}
				else if(motor_msg_right.encoder.dir == DIR_NEG)
				{
					motor_msg_right.circle_cnt--;
				}else{}
			}
		}
}



static void get_imu_data(imu_t *imu)
{
	mpu_dmp_get_data(&imu->pitch,&imu->roll,&imu->yaw);
	MPU_Get_Accelerometer(&imu->accel[0],&imu->accel[1],&imu->accel[2]);
	MPU_Get_Gyroscope(&imu->gyro[0],&imu->gyro[1],&imu->gyro[2]);
}
uint8_t cmd_buf_clear[5] = {0xaa, 0x1a, 0x00, 0x01, 0xff};
uint8_t cmd_buf_start[5] = {0xaa, 0x1a, 0x00, 0x01, 0xff};
static void get_motor_msg(wheel_msg_t *wheel_msg, motor_msg_t *motor_msg)
{
	/* get motor message */
	static uint8_t dir_cnt;
	dir_cnt++;
	if(dir_cnt == 15)//45ms
	{
		dir_cnt = 0;
		motor_msg->speed_rpm_last = motor_msg->speed_rpm;
		motor_msg->encoder.encoder[0] = (uint32_t)(__HAL_TIM_GET_COUNTER(&htim3));
		motor_msg->encoder.encoder_err = motor_msg->encoder.encoder[0] - motor_msg->encoder.encoder[1];
		motor_msg->encoder.encoder[1] = motor_msg->encoder.encoder[0];
		if(motor_msg->encoder.encoder_err > 0)
		{
			motor_msg->encoder.dir = DIR_POS;
			motor_msg->speed_rpm = wheel_msg->speed_get;
		}
		else if(motor_msg->encoder.encoder_err < 0)
		{
			motor_msg->encoder.dir = DIR_NEG;
			motor_msg->speed_rpm = -wheel_msg->speed_get;
		}
		else{motor_msg->encoder.dir = DIR_NULL;}
	}
	/* cancel the stall protect */
	if(wheel_msg->err == 0x07)
	{
		if(motor_msg->wheel_type == WHEEL_L)
		{
			HAL_UART_Transmit_DMA(&WHEEL_L_HUART, (uint8_t *)cmd_buf_clear, 5);
			motor_ctrl(motor_msg, 0);
//			HAL_UART_Transmit_DMA(&WHEEL_L_HUART, (uint8_t *)cmd_buf_start, 5);
		}
		if(motor_msg->wheel_type == WHEEL_R)
		{
			HAL_UART_Transmit_DMA(&WHEEL_R_HUART, (uint8_t *)cmd_buf_clear, 5);	
			motor_ctrl(motor_msg, 0);
//			HAL_UART_Transmit_DMA(&WHEEL_R_HUART, (uint8_t *)cmd_buf_start, 5);	
		}
	}
}

static void get_debug_ctrl_msg(void)
{

}

static float sgn(float num)
{
	if(num>0)return 1;
	else if(num<0)return -1;
	else return 0;
}

float balance_pid_calc(float get_pitch, float set_pitch, int16_t gyro_y)
{
	double err_tan;
	balance_pid.get_pitch = get_pitch;
	balance_pid.set_pitch = set_pitch;
	balance_pid.gyro_y = gyro_y;
	balance_pid.err = balance_pid.set_pitch - balance_pid.get_pitch;
	VAL_LIMIT(balance_pid.err, -20.0f, 20.0f);
	err_tan = 0.078539816*balance_pid.err;/* 0.1047根据15°算出 */
	/* 计算KP输出 */
	balance_pid.p_out = balance_pid.kp * tan(err_tan);
	/* 计算KD输出 */
	balance_pid.d_out = -sgn(balance_pid.gyro_y) * balance_pid.kd * balance_pid.gyro_y * balance_pid.gyro_y;
	/* 计算总输出 */
	balance_pid.out = balance_pid.p_out + balance_pid.d_out; 
	/* 输出补偿，此项根据电机特性不同修改 */
	balance_pid.balance_out[WHEEL_L] = balance_pid.out + sgn(balance_pid.out)*balance_pid.cur_comp[WHEEL_L];
	balance_pid.balance_out[WHEEL_R] = balance_pid.out + sgn(balance_pid.out)*balance_pid.cur_comp[WHEEL_R];
	/* 平衡环电流输出限幅 */
	VAL_LIMIT(balance_pid.balance_out[WHEEL_L], -2500.0f, 2500.0f);
	VAL_LIMIT(balance_pid.balance_out[WHEEL_R], -2500.0f, 2500.0f);
	return 0;
}

void start_get_imu_task(void const * argument)
{
	/* init imu */
	MPU_Init();
	while(mpu_dmp_init());	
	/* init motor control resource */
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_GPIO_WritePin(ENABLE_L_GPIO_Port, ENABLE_L_Pin, GPIO_PIN_RESET);//low level enable
	HAL_GPIO_WritePin(ENABLE_R_GPIO_Port, ENABLE_R_Pin, GPIO_PIN_RESET);//low level enable
	/* init motor direction test resource */
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
	/* init motor driver uart */
	uart_init_idle(&WHEEL_L_HUART, wheel_left_uart_rx_buffer, WHEEL_HUART_RX_BUFFER_SIZE);
	uart_init_idle(&WHEEL_R_HUART, wheel_right_uart_rx_buffer, WHEEL_HUART_RX_BUFFER_SIZE);
	/* init debug and control uart */
	uart_init_idle(&DEBUG_CTRL_HUART, debug_ctrl_rx_buffer, DEBUG_CTRL_HUART_RX_SIZE);
	/* init other resource and parameters */
	led_all_off();	
	osDelay(3000);
	led_blink(LED0, 1, 0);	
	
	imu_init_ok_flag = 1;
	chassis_pid_init();
  uint32_t get_imu_task_wake_time = osKernelSysTick();
  while(1)
  {
		imu_time_ms = HAL_GetTick() - imu_time_last;
		imu_time_last = HAL_GetTick();
		/* get imu data */
		get_imu_data(&imu);
		VAL_LIMIT(imu.pitch, -20.0f, 20.0f);
		/* get motor message, delet cancle the stall protect */
		get_motor_msg(&wheel_left_msg, &motor_msg_left);
		get_motor_msg(&wheel_right_msg, &motor_msg_right);
		/* get debug and control message */
		get_debug_ctrl_msg();
		
		/*balance circle | current circle*/
		balance_pid_calc(imu.pitch, sys_variable.mid_angle, (0.1f * imu.gyro[1]));
		balance_pid.balance_out[WHEEL_L] = -balance_pid.balance_out[WHEEL_L];
		balance_pid.balance_out[WHEEL_R] = balance_pid.balance_out[WHEEL_R];
		
		/*speed circle*/
		speed.speed_circle_cnt++;
		sys_variable.vl = motor_msg_left.speed_rpm;
		sys_variable.vr = -motor_msg_right.speed_rpm;
		sys_variable.v = (sys_variable.vl + sys_variable.vr)/2.0f;
		if(speed.speed_circle_cnt == 20)//60ms
		{
			speed.speed_circle_cnt= 0 ;
			speed.speed_out = 0;//这里需要串口控制输入	     
		}		
		
		/*direction circle*/
		if(turn.turn_circle_cnt == 10)//30ms
		{
			turn.turn_circle_cnt = 0 ;
			turn.turn_out = 0;//这里需要串口控制输入	 turn.turn_kp *    
		}	
		motor_msg_left.ctrl_give = balance_pid.balance_out[WHEEL_L] + speed.speed_out + turn.turn_out;
		motor_msg_right.ctrl_give = balance_pid.balance_out[WHEEL_R] + speed.speed_out + turn.turn_out;
		VAL_LIMIT(motor_msg_left.ctrl_give, -5000.0f, 5000.0f);
		VAL_LIMIT(motor_msg_right.ctrl_give, -5000.0f, 5000.0f);
		motor_ctrl(&motor_msg_left, motor_msg_left.ctrl_give);
		motor_ctrl(&motor_msg_right,motor_msg_right.ctrl_give);

		sprintf(lcd_buf, "pitch: %5.3f  ctrl: %5.3f  \n",  imu.pitch, motor_msg_left.ctrl_give);
		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)lcd_buf, count_buf((uint8_t *)lcd_buf));	
		
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


void chassis_pid_init(void)
{
	balance_pid.kp = -3000;
	balance_pid.kd = 0.1;
	balance_pid.cur_comp[WHEEL_L] = 220;
	balance_pid.cur_comp[WHEEL_R] = 220;
	sys_variable.mid_angle = 0.0f;
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
					balance_pid.kd += 0.001f;
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
					balance_pid.kd -= 0.001f;
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
uint16_t sen_cnt;
char usart2_tx_buf[300];
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
//////			sprintf(lcd_buf, "$ID: %5d   pitch: %5.3f   gyro: %5d   out:%6d   $END\n", send_cnt, imu.pitch, imu.gyro[1], sys_variable.totall_out[WHEEL_L]);
//////			HAL_UART_Transmit_DMA(&huart1, (uint8_t *)lcd_buf, count_buf((uint8_t *)lcd_buf));
//////			sprintf(usart2_tx_buf, "$ID: %5d   option %1d   pit: %5.3f   blc_kp: %5.3f   blc_kd: %5.3f   mid: %5.3f   comp_l: %5.3f   comp_r: %5.3f   speed_kp: %5.3f   speed_kd: %5.3f   speed_set: %5.3f   out:%6d  $END\n", send_cnt, modify_option, sys_variable.theta, balance_pid.kp, balance_pid.kd, sys_variable.mid_angle, balance_pid.cur_comp[WHEEL_L], balance_pid.cur_comp[WHEEL_R], pid_move_speed.p, pid_move_speed.d, sys_variable.set_v_speed, sys_variable.totall_out[WHEEL_L]);
//////			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)usart2_tx_buf, count_buf((uint8_t *)usart2_tx_buf));
//////			if(send_cnt == 60000)
//////			{
//////				send_cnt = 0;
//////			}
		}

		osDelayUntil(&lcd_scan_task_wake_time, LCD_SCAN_TASK_PERIOD);
  }
}
