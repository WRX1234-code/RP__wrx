
#ifndef __MOTOR_DEF_H
#define __MOTOR_DEF_H

#include "stm32h7xx_hal.h"

#include "pid.h"


/*----------------------------自定义枚举类型开始--------------------------------*/
typedef enum motor_state_e
{
	M_OFFLINE = 0,	
	
	M_ONLINE,

	M_TYPE_ERR,
	M_ID_ERR,
	M_INIT_ERR,	
	M_DATA_ERR,
	
}motor_state_e;

typedef enum motor_protect_e
{
	
	M_PROTECT_ON = 0,
	M_PROTECT_OFF ,	
	
}motor_protect_e;

typedef enum motor_init_e
{

	M_DEINIT = 0,
	M_INIT,

}motor_init_e;

typedef enum motor_drive_e
{
	M_CAN1,
	M_CAN2,
	M_PWM,
	M_USART1,
	M_USART2,
	M_USART3,	
	M_USART4,
	M_USART5,

}motor_drive_e;

typedef enum motor_type_e
{
	GM6020 = 1,
	RM3508,
	RM2006,
	KT9015 = 4,
	KT9025,
	DM4310,
	DM8009,
}motor_type_e;

typedef enum motor_dir_e 
{
	CLOCK_WISE    = 0x00,    
	N_CLOCK_WISE  = 0x01, 

	MOTOR_B,
	MOTOR_F,
		
}motor_dir_e;

/* Exported function ------------------------------------------------------------*/
void motor_pid_init(motor_pid_t *motor_pid,motor_pid_t extern_motor_pid);
void rm_motor_pid_init(pid_ctrl_t *motor_pid,pid_ctrl_t extern_motor_pid);




#endif


