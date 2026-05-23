#include "launch.h"

static void Launch_Work(Launch_t*  launch);

Launch_t  launch = {
	.state = L_LOCK,
	.mode = SINGLE_SHOT,
	
	.shoot_level = 0,
	
	.work = Launch_Work,

};


static void Launch_Data_Update(Launch_t*  launch)
{
  

}



static void Launch_Work(Launch_t*  launch)
{
  
}