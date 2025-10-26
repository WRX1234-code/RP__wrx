#ifndef __COMMUNICATE_H
#define __COMMUNICATE_H
#include <stdint.h>

typedef struct{
	int16_t x_target_speed;
	int16_t y_target_speed;
	int16_t w_target_speed;
	uint8_t heart_state;
	
}Communicate_Chassis_Target_t;
	
extern Communicate_Chassis_Target_t communicate_chassis_target;


#endif

