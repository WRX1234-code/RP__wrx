#ifndef __INFANTRY_H
#define __INFANTRY_H

#include "motor.h"


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





typedef struct{
	Flag_Class_t  U_turn_flag;
	Flag_Class_t  L_turn_flag;
	Flag_Class_t  R_turn_flag;
	
	bool    mec_flag;
	bool    imu_flag;
  bool    turn_flag;
	bool    broken_flag;



}Infantry_Flag_t;








typedef struct{
  Infantry_Mode_e    mode;
	Infantry_Flag_t    flag;




}Infantry_t;



extern Infantry_t  infantry;


#endif


