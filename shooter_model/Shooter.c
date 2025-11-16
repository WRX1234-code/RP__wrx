#include "Shooter.h"

shoot_t shoot = {
	.dial = {
		.type = DIAL_TYPE,
		.pid = &dial_pid,

	},
	.fric = {
		.type = FRIC_TYPE,
	  .pid = &fric_pid,
		
	}，
	.shoot_safe_state = LOCKED,
	.shoot_work_state = CEASEFIRE,
	.dial_work_state = SLEEP,
	.flag = {
		inited_flag = 0;    
	  fire_flag = 0;      
	  firing_flag = 0;    
		dial_block_flag = 0;
    fric_block_flag = 0;
    load_flag = 1;      
    fric_speed_flag = 1;
    fric_temp_flag = 1; 
		
  },

};