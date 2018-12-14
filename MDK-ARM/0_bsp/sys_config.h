#ifndef __SYS_H__
#define __SYS_H__
/*standerd lib*/
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "cmsis_os.h"
/*imu*/
#include "myiic.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "sys.h"
/*task*/
#include "ctrl_task.h"
/*bsp*/
#include "bsp_can.h"
#include "bsp_uart.h"
/*lib*/
#include "pid.h"
#include "ramp.h"
/*lcd*/
#include "ili93xx.h"
#include "malloc.h"


typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;


/*choose the type of chassis, only can choose one*/
#define TWO_WHEELS_ON  1
#define FOUR_WHEELS_ON 0

/**********************remote control setting***************************/
/* normalized remote controller proportion */
#define RC_RESOLUTION     660.0f


#if TWO_WHEELS_ON
/* the radius of wheel(mm) */
#define RADIUS                 62.5f
/* the perimeter of wheel(mm) */
#define PERIMETER              393
/* wheel track distance(mm) */
#define WHEELTRACK             415

/* chassis motor use 3508 default */
/* chassis motor use 3508 */
/* the deceleration ratio of chassis motor */
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)
/* single 3508 motor maximum speed, unit is rpm */
#define MAX_WHEEL_RPM        8000  //8347rpm = 3500mm/s
/* chassis maximum translation speed, unit is mm/s */
#define MAX_CHASSIS_VX_SPEED 8000  //8000rpm
#define MAX_CHASSIS_VY_SPEED 5000
/* chassis maximum rotation speed, unit is degree/s */
#define MAX_CHASSIS_VR_SPEED 300   //5000rpm
/* the ratio of motor encoder value translate to degree */
#define ENCODER_ANGLE_RATIO    (8192.0f/360.0f)
#endif


#if FOUR_WHEELS_ON
/*************************chassis setting*******************************/
/* remote mode chassis move speed limit */
/* left and right speed (mm/s) */
#define CHASSIS_RC_MAX_SPEED_X  5000.0f
#define CHASSIS_RC_MOVE_RATIO_X 1.0f
/* back and forward speed (mm/s) */
#define CHASSIS_RC_MAX_SPEED_Y  5000.0f
#define CHASSIS_RC_MOVE_RATIO_Y 1.0f
/* chassis rotation speed (deg/s) */
/* used only chassis open loop mode */
#define CHASSIS_RC_MAX_SPEED_R  30.0f
#define CHASSIS_RC_MOVE_RATIO_R 1.0f

/************************ chassis parameter ****************************/
/* the radius of wheel(mm) */
#define RADIUS                 62.5f
/* the perimeter of wheel(mm) */
#define PERIMETER              393

/* wheel track distance(mm) */
#define WHEELTRACK             415
/* wheelbase distance(mm) */
#define WHEELBASE              406

/* gimbal is relative to chassis center x axis offset(mm) */
#define GIMBAL_X_OFFSET        130
/* gimbal is relative to chassis center y axis offset(mm) */
#define GIMBAL_Y_OFFSET        0

/* chassis motor use 3508 default */
/* chassis motor use 3508 */
/* the deceleration ratio of chassis motor */
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)
/* single 3508 motor maximum speed, unit is rpm */
#define MAX_WHEEL_RPM        10000  //8347rpm = 3500mm/s
/* chassis maximum translation speed, unit is mm/s */
#define MAX_CHASSIS_VX_SPEED 8000  //8000rpm
#define MAX_CHASSIS_VY_SPEED 5000
/* chassis maximum rotation speed, unit is degree/s */
#define MAX_CHASSIS_VR_SPEED 300   //5000rpm
/* the ratio of motor encoder value translate to degree */
#define ENCODER_ANGLE_RATIO    (8192.0f/360.0f)
#endif



#define VAL_LIMIT(val, min, max) \
do {\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\
} while(0)\

#endif
