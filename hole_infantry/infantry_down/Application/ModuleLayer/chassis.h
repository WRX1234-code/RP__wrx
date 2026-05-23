#ifndef __CHASSIS_H
#define __CHASSIS_H

#include  "motor.h" 

#define   CHASSIS_SPEED_MAX     7500
#define   FRONT_SPEED_MAX    3000
#define   RIGHT_SPEED_MAX    3000
#define   CYCLE_SPEED_MAX    1500
#define   TURN_CYCLE_SPEED   1000

#define  ROOM_ENOUGH_GIMBAL   0


typedef struct{
	int16_t  front_speed;
	int16_t  right_speed;
	int16_t  cycle_speed;
	
	int32_t  front_location;
	int32_t  right_location;
	int32_t  cycle_location;
	
	int16_t  motor_speed[WHEEL_CNT];
	int32_t  motor_position[WHEEL_CNT];
	
}Chassis_Target_t;


typedef enum{
	SPEED_MODE,
	POSITION_MODE,
	
}Chassis_Pid_Mode_e;

typedef enum{
  C_SLEEP,
	C_INIT,
	C_BOSS,
	C_SLAVE,

}Chassis_Mode_e;

typedef struct{
	float  wheel_initial_out[WHEEL_CNT];
	float  wheel_powerd_out[WHEEL_CNT];


}Chassis_Out_t;


typedef struct{
	Motor_RM_t*         wheel[WHEEL_CNT];
	Chassis_Pid_Mode_e  pid_mode; 
	Chassis_Mode_e      mode;
  Chassis_Target_t    target;
  Chassis_Out_t       out;

}Chassis_t;


extern Chassis_t chassis;


#endif


