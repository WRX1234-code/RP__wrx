#include "Balance.h"
void Balance_Init(Balance_t* balance);
static void Balance_Init_Judge (Balance_t* balance);
static void Balance_Status_Update(Balance_t* balance);
static void RC_Move_Mode_Update(Balance_t* balance);
void Rescue_Check(void);
uint8_t last_step[4];
Balance_Remote_Ctrl Balance_Rc = 
{
	.sensor = &rc_sensor,
	
	.last_thumbwheel_step = last_step,
};

Balance_t Balance =
{

	.mode = Sleep_Mode,
	
	.init = Balance_Init,
	
	.rc = &Balance_Rc,
};

void Balance_Init(Balance_t* balance)
{
	balance->update = Balance_Status_Update;
	balance->command = command;
}

/**
  * @brief  整车状态更新
  * @param  Balance_t* balance
  * @retval None
  */
static void Balance_Status_Update(Balance_t* balance)
{
	Balance_Init_Judge(balance);//初始化完成判断
	
	if(rc_sensor.work_state == DEV_OFFLINE)
	{
		balance->mode=Sleep_Mode;
		balance->reset_struct.reset_cnt=0;
		//RC_Offline_Flag_Clean
	}
//	else if(balance->mode==Sleep_Mode)//开控但是sleep就初始化
//	{
//		balance->mode=Init_Mode;
//	}
//	else if(balance->mode==Init_Mode&&balance->reset_struct.reset_state==Balance_reset_OK)
//	{
//		Rescue_Check();
//		balance->reset_struct.reset_cnt=0;
//		balance->mode=Mec_Mode;
//	}
	else
	{
		Rescue_Check();
		RC_Move_Mode_Update(balance);
		
	}	
}
/**
 * @brief 初始化完成判断
 */
static void Balance_Init_Judge (Balance_t* balance)
{
	if(Chassis.reset_struct->reset_state==Chassis_reset_OK)
	{
		balance->reset_struct.reset_state=Balance_reset_OK;
	}
	if(balance->mode==Init_Mode)
	{
		balance->reset_struct.reset_cnt++;
	}
	if(balance->reset_struct.reset_cnt>=BALANCE_INIT_CNT_MAX)
	{
		balance->reset_struct.reset_cnt=BALANCE_INIT_CNT_MAX;
		balance->reset_struct.reset_state=Balance_reset_OK;
	}
}


/**
  * @brief  底盘自救判断
  * @param  None
  * @retval None
  */
uint8_t t1_rescue_cnt;
void Rescue_Check(void)
{
	static bool last_Rescue_Flag;
	float R_phi0 = Chassis.Leg_Unit[R_Leg]->Link->info->angle->vir_phi0_ ;
	float L_phi0 = Chassis.Leg_Unit[L_Leg]->Link->info->angle->vir_phi0_ ;
	float thetab	 = Chassis.Posture->info->pitch;
	float roll	 = Chassis.Posture->info->roll;
	/*自救条件判断*/
	if(abs(thetab)>= angle2rad(90.f)||abs(roll)>=angle2rad(25.f))//机体太斜
	{
		Balance.Flag->Rescue_Flag=true;
		Balance.Flag->Unable_Rescue_Flag=true;
		t1_rescue_cnt++;
	}
	else if(abs(R_phi0)>=80||abs(L_phi0)>=80)//机体角度还行但是腿的姿态很离谱，可以自救
	{
		
		Balance.Flag->Rescue_Flag=true;
		Balance.Flag->Unable_Rescue_Flag=false;
		t1_rescue_cnt++;
	}
	else//机体正常控，不自救
	{
		Balance.Flag->Rescue_Flag=false;
		Balance.Flag->Unable_Rescue_Flag=false;
	}
	/*自救标志位上升沿*/
	if(last_Rescue_Flag==false&&Balance.Flag->Rescue_Flag==true)
	{
		Balance.Flag->Rescue_Trigger=true;
	}
	else
	{
		Balance.Flag->Rescue_Trigger=false;
	}
	last_Rescue_Flag=Balance.Flag->Rescue_Flag;
	
}

/**
 * @brief 遥控模式整车移动模式更新
 */

static void RC_Move_Mode_Update(Balance_t* balance)
{
	rc_sensor_info_t*  rc_info=balance->rc->sensor->info;
	
	if(balance->command[JUMP].cmd_value==true)
	{
		balance->Flag->Jumping_Flag = true;
	}
	if(balance->command[KNEE_STRIKE].cmd_value==true)
	{
		balance->Flag->Knee_Strike_Flag = true;
	}
		
	switch (rc_info->s2)
	{
		case RC_SW_UP:
			balance->mode=LEG_TEST_Mode;
			break;
		
		case RC_SW_MID:
			balance->mode=Mec_Mode;
			balance->Flag->Leg_length_ctrl_Flag=false;
			break;
		
		case RC_SW_DOWN:
			balance->mode=Mec_Mode;
			balance->Flag->Leg_length_ctrl_Flag=true;
			break;
		
		default:
			break;
	}
		
	
}
