#ifndef __CTRL_TASK_H
#define __CTRL_TASK_H

#define GET_IMU_TASK_PERIOD (2)
typedef struct 
{
	float pitch;
	float roll;
	float yaw; 		//欧拉角
	int16_t accel[3];//加速度传感器原始数据
	int16_t gyro[3];//陀螺仪原始数据
	int16_t temp;	//温度
}imu_t;

typedef struct
{
	/*第一部分参数将在调参阶段发挥作用，之后将被固定*/
	float balance_kp;
	float balance_kd;
	float deat_balance_out;
	float speed_kp;
	float speed_ki;
	float turn_kp;
	float mid_angle;
	float set_v_speed;
	
	/*第二部分为系统重要变量*/
	float balance_out;
	float speed_out;
	float turn_out;
	int16_t totall_out[2];
	
	float theta; //posture_angle
	float psi;//direction_angle
	float v; /*!< system speed, (vl + vr)/2 */
	float vl;/*!< left wheel line speed */
	float vr;/*!< right wheel line speed */
	
}sys_variable_t;

extern imu_t imu;

typedef enum
{
	LED0 = 0,
	LED1 = 1
}LED;
void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,float duty);
void start_get_imu_task(void const * argument);
void led_all_off(void);
void led_blink(LED led, uint8_t time, uint16_t delay_time);

/**********************************************control part*************************************************/
#define CTRL_TASK_PERIOD (5)

typedef struct
{
	float get_pitch;
	float set_pitch;
	float gyro_y;
	float kp;
	float kd;
	float err;
	float p_out;
	float d_out;
	float pd_out;
}balance_pid_t;

enum
{
	WHEEL_L = 0,
	WHEEL_R = 1
};

void start_ctrl_task(void const * argument);
void chassis_pid_init(void);
float balance_pid_calc(float get_pitch, float set_pitch, int16_t gyro_y);


#define KEY_SCAN_TASK_PERIOD (10)

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base address of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base address of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base address of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base address of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base address of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base address of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base address of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base address of Sector 7, 128 Kbytes */

#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_4   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_7  +  GetSectorSize(ADDR_FLASH_SECTOR_7) -1 /* End @ of user Flash area : sector start address + sector size -1 */

#define KEY0 		HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin) //PE4
#define KEY1 		HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)	//PE3 
#define KEY2 		HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin) //PE2
#define WK_UP 	HAL_GPIO_ReadPin(KEY_WK_UP_GPIO_Port,KEY_WK_UP_Pin)	//PA0

#define KEY0_PRES 	1
#define KEY1_PRES   2
#define KEY2_PRES	3
#define WKUP_PRES   4
void start_key_scan_task(void const * argument);
uint8_t key_scan(uint8_t mode);
static uint32_t GetSector(uint32_t Address);
static uint32_t GetSectorSize(uint32_t Sector);

/**********************************************lcd part*************************************************/
#define LCD_SCAN_TASK_PERIOD (200)
void start_lcd_scan_task(void const * argument);
void float_to_str(char *str,double num);
void uart_send_senser(void);
#endif

