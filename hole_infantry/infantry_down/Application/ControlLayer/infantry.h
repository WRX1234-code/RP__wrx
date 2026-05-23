#ifndef __INFANTRY_H
#define __INFANTRY_H

#include "motor.h"

#define  WHEEL_UP_TO_ONCE        rc_info->thumbwheel.step[0] != last_thumbwheel_step[0] || rc_info->thumbwheel.step[2] != last_thumbwheel_step[2]    
#define  WHEEL_DOWN_TO_ONCE        rc_info->thumbwheel.step[1] != last_thumbwheel_step[1] || rc_info->thumbwheel.step[3] != last_thumbwheel_step[3]   


typedef enum{
	RC_CTRL,
	KEY_CTRL,

}Infantry_Ctrl_e;

typedef enum{
	I_SLEEP,
	I_INIT,
	I_MEC,
	I_IMU,
	I_TURN,
  I_HOLE,
}Infantry_Mode_e;


typedef struct{
	bool  value;
	bool  last_value;
	uint16_t  work_time;
	uint16_t  work_time_max;

}Flag_Class_t;


typedef enum{
	NO_VIS,
	AUTO_AIM,
	S_BUFF,
	B_BUFF,
  OUTPOST,
	HERO,
	VISION_CNT,

}Infantry_Vision_Class_e;


typedef struct{
//	Flag_Class_t  U_turn_flag;
//	Flag_Class_t  L_turn_flag;
//	Flag_Class_t  R_turn_flag;
	
	bool    mec_flag;
	bool    imu_flag;
  bool    turn_flag;
	bool    hole_flag;
	bool    vision_flag;
	bool    broken_flag;

  bool    U_turn_flag;
	bool    L_turn_flag;
	bool    R_turn_flag;


}Infantry_Flag_t;








typedef struct Infantry_Struct_t{
	Infantry_Ctrl_e          ctrl;
  Infantry_Mode_e          mode;
	Infantry_Flag_t          flag;
  Infantry_Vision_Class_e  vision;

  void (* work)(struct Infantry_Struct_t* infantry);

}Infantry_t;



extern Infantry_t  infantry;


#endif


