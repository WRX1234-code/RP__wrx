#include "infantry.h"
#include "board_protocol.h"
#include "chassis.h"
#include "gimbal.h"
#include "launch.h" 


Infantry_t  infantry;


static void Infantry_Work(void)
{
	
}

static void Infantry_State_Check(void)
{
	
}


static uint8_t last_thumbwheel_step[4];
static void Rc_State_Check(void)
{
	
	rc_sensor_info_t*  rc_info = rc_sensor.info;
	
	switch (rc_info->s1.value)
	{
		case  RC_SW_UP:
			
		  if(rc_info->s2.value == RC_SW_UP)
			{
				if(rc_info->thumbwheel.step[0] != last_thumbwheel_step[0] || rc_info->thumbwheel.step[2] != last_thumbwheel_step[2])
				{
					infantry.flag.turn_flag = !infantry.flag.turn_flag;
					
					if(infantry.flag.turn_flag == true)
					{
						infantry.mode = I_TURN;
					}
				}
			}
		
		
		
		
			break;
		
		case  RC_SW_MID:
			break;
		
		case  RC_SW_DOWN:
			break;
		
		default:
			break;
		
		
	}
}