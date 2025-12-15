#include "Balance.h"
#include "Board_protocol.h"
void Balance_Init(Balance_t* balance);
static void Balance_Init_Judge (Balance_t* balance);
static void Balance_Status_Update(Balance_t* balance);
static void RC_Move_Mode_Update(Balance_t* balance);
static void Key_Move_Mode_Update(Balance_t* balance);
void Rescue_Check(void);
uint8_t last_step[4];
Balance_Remote_Ctrl Balance_Rc = 
{
	.sensor = &rc_sensor,
	
	.last_thumbwheel_step = last_step,
};

Balance_t Balance =
{
  .ctrl = RC_CTRL,
	
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
//	
//	if(balance->command[JUMP].cmd_value==true)
//	{
//		balance->Flag->Jumping_Flag = true;
//	}
//	if(balance->command[KNEE_STRIKE].cmd_value==true)
//	{
//		balance->Flag->Knee_Strike_Flag = true;
//	}
//	if(balance->command[U_TURN].cmd_value == true)
//	{
//		balance->Flag->U_Turn_Flag = true;
//	}		
//	if(balance->command[R_TURN45].cmd_value == true)
//	{
//		balance->Flag->R_Turn_Flag = true;
//	}		
//	if(balance->command[L_TURN45].cmd_value == true)
//	{
//		balance->Flag->L_Turn_Flag = true;
//	}		
//	
	
	
	switch (rc_info->s1)
	{
		case RC_SW_UP:
			if(rc_info->s2 == RC_SW_MID)
			{
				if(rc_info->thumbwheel.step[0] != balance->rc->last_thumbwheel_step[0])
				{
					balance->Flag->Turn_Flag = !balance->Flag->Turn_Flag;
					if(balance->Flag->Turn_Flag == true)
					{
						balance->mode=Turn_Mode;
						balance->Flag->S_Turn_Flag = false;
					}
					
					
				}
				else if(rc_info->thumbwheel.step[2] != balance->rc->last_thumbwheel_step[2])
				{
					balance->Flag->S_Turn_Flag = !balance->Flag->S_Turn_Flag;
					if(balance->Flag->S_Turn_Flag == true)
					{
						balance->mode=Turn_Mode;
//						D_Board_Tx_Pkt.car_base_mode = 
					}
					
					
				}
			}
			
			else if(rc_info->s2 == RC_SW_DOWN)
			{
				if(rc_info->thumbwheel.step[0] != balance->rc->last_thumbwheel_step[0])
			{
//					balance->Flag->Vision_Test_Flag = !balance->Flag->Vision_Test_Flag;
//					if(balance->Flag->Vision_Test_Flag == true)
//					{
//						balance->mode = VISION_TEST_Mode;
//					}
					
					if(balance->Flag->Vision_Test_Flag == true)
					{
						balance->mode = LEG_TEST_Mode;
						balance->Flag->Leg_length_ctrl_Flag = 1;
						
					}
					else
					{
						balance->mode = Imu_Mode;
						balance->Flag->Leg_length_ctrl_Flag = 0;
					}
					
				}
				else if(rc_info->thumbwheel.step[2] != balance->rc->last_thumbwheel_step[2])
				{
					balance->Flag->Key_Flag = !balance->Flag->Key_Flag;
					if(balance->Flag->Key_Flag == true)
					{
						balance->mode = Key_Mode;
						Key_Move_Mode_Update(balance);
						balance->ctrl = KEY_CTRL;
					}
					else{
					  balance->mode = Imu_Mode;
						balance->ctrl = RC_CTRL;
					}
					
				}
			}
			
			else if(rc_info->s2 ==  RC_SW_UP && balance->rc->last_s2 == RC_SW_MID)
			{
				D_Board_Tx_Pkt.Launch_mode = 0;
				if(balance->Flag->Launch_Open_Flag == true)
				{
					D_Board_Tx_Pkt.is_fire = 1;
				}
				
			}
			break;
		
		case RC_SW_MID:
			if(rc_info->thumbwheel.step[0] != balance->rc->last_thumbwheel_step[0])
			{
				balance->Flag->Launch_Open_Flag = !balance->Flag->Launch_Open_Flag;
				if(balance->Flag->Launch_Open_Flag == true)
				{
					D_Board_Tx_Pkt.is_operater_ctrl = 1;
					D_Board_Tx_Pkt.Launch_state = 1;
				}
				else
				{
					D_Board_Tx_Pkt.Launch_state = 0;
				}
			}
			
			if(rc_info->s2 == RC_SW_UP && D_Board_Tx_Pkt.is_operater_ctrl == 1)
			{
				D_Board_Tx_Pkt.Launch_mode = 1;
				if(balance->Flag->Launch_Open_Flag == true)
				{
					D_Board_Tx_Pkt.is_fire = 1;
				}
				else
				{
					D_Board_Tx_Pkt.is_fire = 1;
				}
			}
			else if(rc_info->s2 == RC_SW_DOWN  && balance->rc->last_s2 == RC_SW_MID)
			{
				balance->Flag->Self_Aim_Flag = !balance->Flag->Self_Aim_Flag;
				if(balance->Flag->Self_Aim_Flag == true)
				{
					D_Board_Tx_Pkt.vision_mode = 1;
					D_Board_Tx_Pkt.is_operater_ctrl = 0;
					if(rc_info->s1 == RC_SW_UP && rc_info->s2 == RC_SW_UP  && balance->rc->last_s2 == RC_SW_MID)
					{
						D_Board_Tx_Pkt.is_operater_ctrl = 1;
					}
				
				}
			}
			
			if(rc_info->thumbwheel.step[2] != balance->rc->last_thumbwheel_step[2])
			{
				if(balance->Flag->Self_Aim_Flag == true)
			  {
					balance->Flag->Auto_step ++;
					if(balance->Flag->Auto_step % 4 == 3)
					{
						
					}
					if(balance->Flag->Auto_step % 4 == 2)
					{
						
					}
					else if(balance->Flag->Auto_step % 4 == 1)
					{
						
					}
					else if(balance->Flag->Auto_step % 4 == 0)
					{
						
					}
			  }
					
			}
			
			break;
		
		case RC_SW_DOWN:
			if(rc_info->s2 ==  RC_SW_UP)
			{
				if(rc_info->thumbwheel.step[0] != balance->rc->last_thumbwheel_step[0])
				{
					balance->Flag->Jumping_Flag = 1;
				}
				else if(rc_info->thumbwheel.step[2] != balance->rc->last_thumbwheel_step[2])
				{
					balance->Flag->Knee_Strike_Flag = !balance->Flag->Knee_Strike_Flag;
				}
			}
			
			else if(rc_info->s2 ==  RC_SW_MID)
			{
        if(rc_info->thumbwheel.step[0] != balance->rc->last_thumbwheel_step[0])
				{
					balance->Flag->U_Turn_Flag = 1;
				}
			}
			
			else if(rc_info->s2 ==  RC_SW_DOWN)
			{
				if(rc_info->thumbwheel.step[0] != balance->rc->last_thumbwheel_step[0])
				{
					balance->Flag->Fly_Flag = !balance->Flag->Fly_Flag;
				}
				else if(rc_info->thumbwheel.step[2] != balance->rc->last_thumbwheel_step[2])
				{
					balance->Flag->Reserve_Fly_Flag = !balance->Flag->Reserve_Fly_Flag;
				}
			}
		
			break;
		
		default:
			break;
	}
		
	balance->rc->last_s2 = rc_info->s2;
	balance->rc->last_thumbwheel_step[0] = rc_info->thumbwheel.step[0];
	balance->rc->last_thumbwheel_step[2] = rc_info->thumbwheel.step[2];
}

void Key_Move_Mode_Update(Balance_t* balance)
{
	rc_sensor_info_t*  rc_info=balance->rc->sensor->info;
	
	
	if(rc_info->Z.status == release_to_press)
	{
		if(balance->last_mode != Mec_Mode)
		{
			balance->last_mode = Mec_Mode;
		}
		else
		{
			balance->last_mode = Imu_Mode;
		}
	}
	
	if(rc_info->C.status == release_to_press)
	{
		if(balance->last_mode != Turn_Mode)
		{
			balance->last_mode = Turn_Mode;
		}
		else
		{
			balance->last_mode = Imu_Mode;
		}
	}
	
	if(rc_info->B.status == release_to_press)
	{
		balance->Flag->Launch_Open_Flag = !balance->Flag->Launch_Open_Flag;
		if(balance->Flag->Launch_Open_Flag == true)
		{
			D_Board_Tx_Pkt.Launch_state = 1;
		}
		else 
		{
			D_Board_Tx_Pkt.Launch_state = 0;
			D_Board_Tx_Pkt.is_fire = 0;
		}
	}
	
	if(rc_info->mouse_btn_r.status == long_press)
	{
		balance->Flag->Self_Aim_Flag = 1;
	}
	else
	{
		balance->Flag->Self_Aim_Flag = 0;
	}
	
	if(rc_info->mouse_btn_l.status == release_to_press)
	{
		D_Board_Tx_Pkt.Launch_mode = 0;
		if(balance->Flag->Launch_Open_Flag == true)
	  {
		  if( balance->Flag->Self_Aim_Flag == 0)
		  {
			  D_Board_Tx_Pkt.is_fire = 1;
		  }
			else if(D_Board_Tx_Pkt.is_operater_ctrl == 1 && balance->Flag->Self_Aim_Flag == 1)
			{
				D_Board_Tx_Pkt.is_fire = 1;
			}
			else if(D_Board_Tx_Pkt.is_operater_ctrl == 0 && balance->Flag->Self_Aim_Flag == 1)
			{
				D_Board_Tx_Pkt.is_operater_ctrl = 1;
				D_Board_Tx_Pkt.is_fire = 0;
			}
	  }
		else
		{
			D_Board_Tx_Pkt.is_fire = 0;
		}
	}
	else if(rc_info->mouse_btn_l.status == short_press && rc_info->mouse_btn_l.status == long_press)
	{
	  D_Board_Tx_Pkt.Launch_mode = 1;
		if(balance->Flag->Launch_Open_Flag == true)
	  {
			D_Board_Tx_Pkt.is_fire = 1;
   	}
		else
		{
			D_Board_Tx_Pkt.is_fire = 0;
		}
  }
	
	if(rc_info->V.status == release_to_press)
	{
		balance->Flag->Jumping_Flag = 1;
	}
	
	if(rc_info->X.status == release_to_press)
	{
		balance->Flag->Knee_Strike_Flag = 1;
	}
	
	if((rc_info->Shift.status == release_to_press || rc_info->Shift.status == short_press || rc_info->Shift.status == long_press)
		 && rc_info->X.status == release_to_press)
	{
		
	}
	
	if((rc_info->Shift.status == release_to_press || rc_info->Shift.status == short_press || rc_info->Shift.status == long_press)
		 && rc_info->C.status == release_to_press)
	{
		balance->Flag->Turn_Flag = !balance->Flag->Turn_Flag;
		if(balance->Flag->Turn_Flag == 0)
		{
			balance->mode = Imu_Mode;
		}
	}
	
	if((rc_info->Shift.status == release_to_press || rc_info->Shift.status == short_press || rc_info->Shift.status == long_press)
		 && rc_info->Z.status == release_to_press)
	{
		balance->Flag->Lob_Flag = !balance->Flag->Lob_Flag;
		if(balance->Flag->Lob_Flag == 0)
		{
			balance->mode = Imu_Mode;
		}
	}
	
	if(rc_info->R.status == release_to_press && balance->Flag->Self_Aim_Flag == 0)
	{
		balance->Flag->U_Turn_Flag = 1;
		balance->Flag->R_Turn_Flag = 0;
		balance->Flag->L_Turn_Flag = 0;
	}
	else if(rc_info->R.status == release_to_press && balance->Flag->Self_Aim_Flag == 1)
	{
		D_Board_Tx_Pkt.is_operater_ctrl = 1;
	}
	
	if(rc_info->Q.status == release_to_press)
	{
		balance->Flag->L_Turn_Flag = 1;
		balance->Flag->R_Turn_Flag = 0;
		balance->Flag->U_Turn_Flag = 0;
	}
	
	if(rc_info->E.status == release_to_press)
	{
		balance->Flag->R_Turn_Flag = 1;
		balance->Flag->L_Turn_Flag = 0;
		balance->Flag->U_Turn_Flag = 0;
	}
	
	if(rc_info->G.status == release_to_press)
	{
		
	}
	
	if((rc_info->Shift.status == release_to_press || rc_info->Shift.status == short_press || rc_info->Shift.status == long_press)
		 && rc_info->V.status == release_to_press)
	{
		if(balance->Flag->Self_Aim_Flag == 1)
		{
			balance->Flag->Auto_step ++;
			if(balance->Flag->Auto_step %4 == 3)
			{
				
			}
			else if(balance->Flag->Auto_step %4 == 2)
			{
				
			}
			else if(balance->Flag->Auto_step %4 == 1)
			{
				
			}
			else if(balance->Flag->Auto_step %4 == 0)
			{
				
			}
		}
	}
	
	if(rc_info->F.status == release_to_press)
	{
		balance->Flag->Fly_step ++;
		if(balance->Flag->Fly_step % 3 == 2)
		{
			balance->Flag->Fly_Flag = 1;
			balance->Flag->Reserve_Fly_Flag = 0;
		}
		else if(balance->Flag->Fly_step % 3 == 1)
		{
			balance->Flag->Reserve_Fly_Flag = 1;
			balance->Flag->Fly_Flag = 0;
			
		}
		else if(balance->Flag->Fly_step % 3 == 0)
		{
			balance->Flag->Reserve_Fly_Flag = 0;
			balance->Flag->Fly_Flag = 0;
		}
	
	}

	if((rc_info->Z.status == release_to_press || rc_info->Z.status == short_press) 
		&& (rc_info->X.status == release_to_press || rc_info->X.status == short_press)
	  && (rc_info->C.status == release_to_press || rc_info->C.status == short_press))
  {
	  balance->Flag->Rescue_Flag = !balance->Flag->Rescue_Flag;
	}
  else if(rc_info->Z.status == long_press && rc_info->X.status == long_press && rc_info->C.status == long_press)
	{
		
	}	
	
	
	
	





}

