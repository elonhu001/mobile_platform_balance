#ifndef __CTRL_TASK_H
#define __CTRL_TASK_H
#include "bsp_uart.h"
#include "sys_config.h"

#define GET_IMU_TASK_PERIOD (3)
typedef struct 
{
	float pitch;
	float roll;
	float yaw; 		
	int16_t accel[3];
	int16_t gyro[3];
	int16_t temp;	
}imu_t;
extern imu_t imu;

enum
{
	WHEEL_L = 0,
	WHEEL_R = 1
};

typedef enum
{
	DIR_NULL = 0,
	DIR_POS = 1,
	DIR_NEG = 2
}dir_e;

typedef struct
{
	uint32_t encoder[2];
	int32_t  encoder_err;
	dir_e    dir;
}motor_ecoder_t;

typedef struct 
{
	uint32_t ch;
	uint8_t  wheel_type;//ç”µæœºåç§°
	float    speed_rpm;
	float    speed_rpm_last;
	int32_t  circle_cnt;
	int32_t  circle;
	motor_ecoder_t encoder;
	float ctrl_give;
}motor_msg_t;
#define MOTOR_MSG_LEFT_DEFAULT {TIM_CHANNEL_1, WHEEL_L, 0, 0, 0, 0, {0}}
#define MOTOR_MSG_RIGHT_DEFAULT {TIM_CHANNEL_2, WHEEL_R, 0, 0, 0, 0, {0}}
extern motor_msg_t motor_msg_left;
extern motor_msg_t motor_msg_right;

/*è¯¥ç»“æ„ä½“å¹³è¡¡ç¯å‚æ•°ï¼Œå¹³è¡¡ç¯é‡‡ç”¨PDæ§åˆ¶å™¨*/
typedef struct
{
	float get_pitch;    /*<! è·å–çš„pitchè½´ */
	float set_pitch;
	float gyro_y;
	float kp;
	float kd;
	float err;
	float p_out;
	float d_out;
	float out;           /*<! æ§åˆ¶è¾“å‡ºé‡ */
	float cur_comp[2];   /*<! æ§åˆ¶è¡¥å¿é‡ */
	float balance_out[2];/*<! æ§åˆ¶è¾“å‡ºé‡ */
}balance_pid_t;

typedef struct
{
	uint8_t speed_circle_cnt;
	float speed_out;
}speed_t;

typedef struct
{
	uint8_t turn_circle_cnt;
	float turn_kp;
	float turn_out;
}turn_t;

typedef struct
{
	/*µÚÒ»²¿·Ö²ÎÊı½«ÔÚµ÷²Î½×¶Î·¢»Ó×÷ÓÃ£¬Ö®ºó½«±»¹Ì¶¨*/
	float balance_kp;
	float balance_kd;
	float deat_balance_out;
	float speed_kp;
	float speed_ki;
	float turn_kp;
	float mid_angle;
	float set_v_speed;
	float acc_get;
	float acc_set;
	float speed_set;
	/*µÚ¶ş²¿·ÖÎªÏµÍ³ÖØÒª±äÁ¿*/
	float balance_out;
	float speed_out;
	float turn_out;
	int16_t totall_out[4];
	
	float theta; //posture_angle
	float psi;//direction_angle
	float v; /*!< system speed, (vl + vr)/2 */
	float vl;/*!< left wheel line speed */
	float vr;/*!< right wheel line speed */
	
}sys_variable_t;



typedef enum
{
	LED0 = 0,
	LED1 = 1
}LED;
void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,float duty);
void start_get_imu_task(void const * argument);
void led_all_off(void);
void led_blink(LED led, uint8_t time, uint16_t delay_time);
uint16_t count_buf(uint8_t *buf);
/**********************************************control part*************************************************/
#define CTRL_TASK_PERIOD (5)


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
#define KEY2_PRES		3
#define WKUP_PRES   4
void start_key_scan_task(void const * argument);
uint8_t key_scan(uint8_t mode);
static uint32_t GetSector(uint32_t Address);
static uint32_t GetSectorSize(uint32_t Sector);

/**********************************************lcd part*************************************************/
#define LCD_SCAN_TASK_PERIOD (10)
void start_lcd_scan_task(void const * argument);
void float_to_str(char *str,double num);
void uart_send_senser(void);
#endif

