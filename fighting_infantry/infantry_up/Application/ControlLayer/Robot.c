#include "Robot.h"
#include "rc_sensor.h"

Robot_t robot;

void Robot_CU_Update(){


};

void Robot_Mode_Update(){


};

void Robot_Elec_Level_Update(){
	static float roller_step;
	static float last_roller_step;
	
	static float last_value;
	static float roller_value;
	
	static uint8_t key_cnt;
	
	roller_step=rc_sensor.info->thumbwheel.step[2];
	roller_value=rc_sensor.info->thumbwheel.value;
	
	if((robot.CU == RC_CU && (roller_step != last_roller_step && roller_value > 0))
		||(robot.CU == KEY_CU && rc_sensor.info->mouse_btn_l.status == release_to_press))
	{
		robot.elec_level = RISING_EDGE;
	}
		
	else if((robot.CU == RC_CU &&(roller_step == last_roller_step && roller_value>0))
		     ||(robot.CU == KEY_CU &&(rc_sensor.info->mouse_btn_l.status == release_to_press
		     || rc_sensor.info->mouse_btn_l.status == short_press || rc_sensor.info->mouse_btn_l.status == long_press)))
	{
		robot.elec_level = HIGH_LEVEL;		
	}
	else if((robot.CU == RC_CU && (roller_value < last_value && roller_value > 0 && last_value < 660))
		     ||(robot.CU == KEY_CU && rc_sensor.info->mouse_btn_l.status == press_to_release))
	{
		robot.elec_level = FALLING_EDGE;
	}
	else if((robot.CU == RC_CU && roller_value == 0 && last_value == 0) || (robot.CU == KEY_CU && rc_sensor.info->mouse_btn_l.status == release))
	{
		robot.elec_level = LOW_LEVEL;
	}
		 
	last_roller_step=roller_step;
	last_value = roller_value;
};

