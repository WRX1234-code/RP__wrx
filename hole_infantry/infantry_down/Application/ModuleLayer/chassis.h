#ifndef __CHASSIS_H
#define __CHASSIS_H

#include  "motor.h" 

#define   CHASSIS_MAX_SPEED     4
#define   FRONT_MAX_SPEED       2
#define   LEFT_MAX_SPEED        2
#define   CYCLE_MAX_SPEED       1.5
#define   TURN_CYCLE_SPEED      0.5

#define  ROOM_ENOUGH_GIMBAL   0


typedef struct{
	float  front_speed;
	float  left_speed;
	float  cycle_speed;
	
	float  front_location;
	float  left_location;
	float  cycle_location;
	
	float  motor_speed[WHEEL_CNT];
	float  motor_position[WHEEL_CNT];
	
}Chassis_Target_t;


typedef struct{
	
	float   front_speed;
	float   left_speed;
	float   cycle_speed;
	
	float  front_location;
	float  left_location;
	float  cycle_location;
	
}Chassis_Measure_t;


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
	float  wheel_end_out[WHEEL_CNT];

}Chassis_Out_t;


typedef struct{
	bool  slip_flag;
	bool  is_allot;
	float wheel_speed_max_difference;
  float slip_low_out;
}Chassis_Slip_t;



typedef struct Chassis_Struct_t{
	Motor_RM_Group_t*   wheel;
	Chassis_Pid_Mode_e  pid_mode; 
	Chassis_Mode_e      mode;
  Chassis_Target_t    target;
	Chassis_Measure_t   measure;
	Chassis_Slip_t      slip;
  Chassis_Out_t       out;
	
	void (*work)(struct Chassis_Struct_t* chassis);

}Chassis_t;


extern Chassis_t chassis;


#endif


