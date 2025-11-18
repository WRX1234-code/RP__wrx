#ifndef __SHOOT_BASE_H
#define __SHOOT_BASE_H

#include "stm32f4xx.h"

#define  DIAL_MOTOR_TYPE                  M_2006_

#define  DIAL_MEC_LIMIT                   0                        //拨盘有无机械限位，无为 0，有为 1

#define  DIAL_ANGLE_DATA_TYPE             uint16_t     
#define  DIAL_SPEED_DATA_TYPE             int16_t
#define  DIAL_CURRENT_DATA_TYPE           int16_t
#define  DIAL_ANGLE_SUM_DATA_TYPE         int32_t



#define  FRIC_SPEED_DATA_TYPE             int16_t
#define  FRIC_CURRENT_DATA_TYPE           int16_t





typedef enum{
	GM_6020_,
	M_3508_,
	M_2006_,
	DM_J4310_,
	KT_4005,
	
}Dial_Motor_type_e;

typedef struct{
	DIAL_ANGLE_DATA_TYPE  angle;
	DIAL_SPEED_DATA_TYPE  speed;
  DIAL_CURRENT_DATA_TYPE  current;
	DIAL_ANGLE_SUM_DATA_TYPE  angle_sum;
	uint8_t temperature;

}Dial_Rx_Info_t;


typedef struct{
	FRIC_SPEED_DATA_TYPE  speed;
  FRIC_CURRENT_DATA_TYPE  current;
	uint8_t temperature;

}Fric_Rx_Info_t;

typedef struct{
	int16_t speed_max;
	int16_t current_min;
	uint8_t block_time_max;
  uint8_t menage_time_max;

}Motor_Block_Config_t;


typedef struct{
	uint16_t oneshot_angle;
	int16_t reload_speed;
	int16_t reset_speed;
	uint16_t adjust_angle;




}Dial_Config_t;










#endif
