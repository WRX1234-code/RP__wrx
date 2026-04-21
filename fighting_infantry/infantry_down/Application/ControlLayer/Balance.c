#include "Balance.h"
#include "Board_protocol.h"
#include "judge.h"
#include "rp_config.h" 
void Balance_Init(Balance_t* balance);
static void Balance_Init_Judge (Balance_t* balance);
static void Balance_Status_Update(Balance_t* balance);
static void RC_Move_Mode_Update(Balance_t* balance);
static void Key_Move_Mode_Update(Balance_t* balance);
void Rescue_Check(void);
uint8_t last_step[4] = {1,0,0,0};
uint8_t last_fire = 0;
Balance_Flag_t flag = {0,0,1};
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
	
	.Flag = &flag,
};

void Balance_Init(Balance_t* balance)
{
	balance->update = Balance_Status_Update;
	balance->command = command;
}

void RC_Flag_Clean(Balance_t* balance)
{

//	balance->Flag->Rescue_Flag = false;
	balance->Flag->Last_Rescue_Flag = false;
	balance->Flag->Gimbal_Ctrl_Flag = false;
	balance->Flag->Rescue_OK = false;
	balance->Flag->Unable_Rescue_Flag = false;//无法自救
	balance->Flag->rescue_cnt = 0;
	balance->Flag->Ctrl_Rescue_Flag = false;
	
	balance->Flag->Leg_length_ctrl_Flag = false;
	
	balance->Flag->Turn_Flag = false;
	balance->Flag->S_Turn_Flag = false;
	balance->Flag->U_G_Turn_Flag = false;
	balance->Flag->U_C_Turn_Flag = false;
	balance->Flag->R_Turn_Flag = false;
	balance->Flag->L_Turn_Flag = false;
	
	balance->Flag->Jumping_Flag = false;//跳跃过程中，用于给chassis状态信号量
	balance->Flag->Knee_Strike_Flag = false;
	balance->Flag->Fly_Flag = false;
	balance->Flag->Reserve_Fly_Flag = false;
	balance->Flag->Lob_Flag = false;
	
	balance->Flag->Gimbal_Reset_OK = false;
	balance->Flag->chassis_reset = false;
	balance->Flag->car_reset = false;

	balance->Flag->Auto_step = 0;   //内含自瞄 0，小符 1，大符 2，前哨 3，英雄 4
	balance->Flag->Fly_step =0;    //内含飞坡 0，反向飞坡 1
	
	balance->Flag->Power_Limit_Flag = false;
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
		
		balance->Flag->Chassis_Sleep_Flag = 1;
		
		Chassis.reset_struct->reset_state = Chassis_reset_NO;
		balance->reset_struct.reset_state=Balance_reset_NO;
		
		D_Board_Tx_Pkt.car_state = 0; 
		D_Board_Tx_Pkt.Gimbal_state = 0;
		D_Board_Tx_Pkt.Launch_state = 0;
		
		D_Board_Tx_Pkt.Gimbal_mode = 0;
		D_Board_Tx_Pkt.vision_mode = 0;
		
		balance->Flag->Rescue_Flag = false;
		RC_Flag_Clean(balance);
	}
	else{
		#if CHASSIS_SWITCH == 0
		
		#else
		  #if RESCUE_SWITCH == 0
		
		  #else
    	  Rescue_Check();
		
		  #endif 
		#endif
		
		if(balance->mode ==Sleep_Mode)//开控但是sleep就初始化
	  {
			RC_Flag_Clean(balance);
		  balance->Flag->Chassis_Sleep_Flag = 0;
	
			balance->reset_struct.reset_cnt=0;
		
		  Chassis.reset_struct->reset_state = Chassis_reset_NO;
		  balance->reset_struct.reset_state=Balance_reset_NO;
			
		  balance->Flag->Mec_Flag = true;
			balance->Flag->Imu_Flag = false;
		
		  if(balance->Flag->Rescue_Flag == true)
		  {
			  balance->mode = Sos_Mode;
		  }
		  else
		  {
			  balance->mode=Init_Mode;
		  }
		
		  if(balance->ctrl == RC_CTRL)
		  {
			  D_Board_Tx_Pkt.car_state = 1;
		  }			
		  else if(balance->ctrl == KEY_CTRL)
		  {
			  D_Board_Tx_Pkt.car_state = 2;
		  }
		
			D_Board_Tx_Pkt.Gimbal_mode = 0;
		  D_Board_Tx_Pkt.vision_mode = 0;
	  }
	  else if(balance->mode == Sos_Mode && balance->Flag->Rescue_OK == false)
	  {  
		  if(balance->Flag->Gimbal_Ctrl_Flag == false)
		  {
			  D_Board_Tx_Pkt.Gimbal_state = 0;
		  }
		  else if(balance->Flag->Gimbal_Ctrl_Flag == true)
		  {
			  D_Board_Tx_Pkt.Gimbal_state = 1;
		    D_Board_Tx_Pkt.Gimbal_mode = 0;
		    balance->Flag->Mec_Flag = true;
		  }
		
	  }
	  else if(balance->Flag->Rescue_OK == true)
	  {
		  balance->Flag->Rescue_OK = false;
		  balance->Flag->Gimbal_Ctrl_Flag = false;

		  balance->mode=Init_Mode;
		  balance->Flag->Mec_Flag = true;
	  }
	  else if(balance->mode==Init_Mode && balance->reset_struct.reset_state==Balance_reset_NO)
	  {
		  D_Board_Tx_Pkt.Gimbal_state = 1;
		  D_Board_Tx_Pkt.Gimbal_mode = 0;
		  balance->Flag->Mec_Flag = true;
		
	  }
	  else if(balance->mode==Init_Mode && balance->reset_struct.reset_state==Balance_reset_OK)
	  {
		  balance->reset_struct.reset_cnt = 0;
		
		  balance->mode = Imu_Mode;
		  balance->Flag->Imu_Flag = true;
		  balance->Flag->Mec_Flag = false;
		  D_Board_Tx_Pkt.Gimbal_mode = 1;
		
//		balance->mode = Test_Mode;
//		balance->Flag->Imu_Flag = false;
//		balance->Flag->Mec_Flag = true;
//		balance->Flag->Test_Flag = true;
//		D_Board_Tx_Pkt.Gimbal_mode = 0;
		
		  if(balance->reset_struct.reset_state == Balance_reset_OK && gimbal.cmd.yaw_mec_tar == gimbal.info.cfg_info.head_to[4])
      {
	  	  balance->Flag->U_G_Turn_Flag = true;
		    balance->Flag->U_C_Turn_Flag = true;
	    }
		
	  }
	  else
	  {
		  balance->Flag->Chassis_Sleep_Flag = 0;

		  if(balance->command[U_TURN].cmd_value==true)
	    {
		    balance->Flag->U_G_Turn_Flag = true;
			  balance->Flag->U_C_Turn_Flag = true;
	    }
//		if(balance->command[L_TURN45].cmd_value==true)
//	  {
//		  balance->Flag->L_Turn_Flag = true;
//	  }
//		if(balance->command[R_TURN45].cmd_value==true)
//	  {
//	   	balance->Flag->R_Turn_Flag = true;
//	  }
//	
	    if(balance->Flag->Turn_Flag == false && balance->Flag->S_Turn_Flag == false)
	    {
			  if(balance->command[JUMP].cmd_value==true)
	      {
 	  	    balance->Flag->Jumping_Flag = true;
	      }
			
	   	  if(balance->command[KNEE_STRIKE].cmd_value==true)
  	    {
		      balance->Flag->Knee_Strike_Flag = true;
				  balance->Flag->chassis_reset = true;
	      }	
//	    if(balance->command[FLY].cmd_value == true)
//	    {
//		    balance->Flag->Fly_Flag = true;
//			  balance->Flag->Reserve_Fly_Flag = false;
//	    }		
//	    if(balance->command[RESERVE_FLY].cmd_value == true)
//	    {
//		    balance->Flag->Reserve_Fly_Flag = true;
//		  	balance->Flag->Fly_Flag = false;
//	    }		
//	  }
	
//	  if(balance->command[LOB].cmd_value == true)
//	  {
//		  balance->Flag->Lob_Flag = true;
//	 
//		  balance->Flag->Reserve_Fly_Flag = false;
//		  balance->Flag->Fly_Flag = false;
	    }		
		
		  if(balance->ctrl == RC_CTRL)
		  {
			  RC_Move_Mode_Update(balance);
		  }			
		  else if(balance->ctrl == KEY_CTRL)
		  {
			  Key_Move_Mode_Update(balance);
		  }
		
  	}	
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
	if(balance->mode==Init_Mode && Balance.Flag->Rescue_Flag == false)
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
void Rescue_Check(void)
{
	float R_phi0 = Chassis.Leg_Unit[R_Leg]->Link->info->angle->vir_phi0_ ;
	float L_phi0 = Chassis.Leg_Unit[L_Leg]->Link->info->angle->vir_phi0_ ;
	float thetab	 = Chassis.Posture->info->pitch;
	float roll	 = Chassis.Posture->info->roll;
	/*自救条件判断*/
	if(fabs(thetab)>= angle2rad(60.f))//机体太斜
	{
		Balance.Flag->Rescue_Flag=true;
		Balance.Flag->Unable_Rescue_Flag=false;
	}
	else if(fabs(roll) >= angle2rad(20.f) && fabs(roll) <= angle2rad(160.f))
	{
		Balance.Flag->Rescue_Flag=true;
		Balance.Flag->Unable_Rescue_Flag=false;
	}
	else if(R_phi0>=60||R_phi0<=-45||L_phi0>=60||L_phi0<=-45)
	{
		
		Balance.Flag->Rescue_Flag=true;
		Balance.Flag->Unable_Rescue_Flag=false;
	}
//	else if(abs(R_phi0)<=15||abs(L_phi0)<=15)//机体正常控，不自救
//	{
//		Balance.Flag->Rescue_Flag=false;
//		Balance.Flag->Unable_Rescue_Flag=false;
//	}
	
//	if(Balance.Flag->Rescue_Flag==true)
//	{
//		Balance.mode = Sos_Mode;
//	}
	
}

/**
 * @brief 遥控模式整车移动模式更新
 */
//static void RC_Move_Mode_Update(Balance_t* balance)
//{
//	
//	rc_sensor_info_t*  rc_info=balance->rc->sensor->info;
//	
//	switch (rc_info->s1)
//	{
//		case RC_SW_UP:
//			if(rc_info->s2 == RC_SW_MID)
//			{
//				if(rc_info->thumbwheel.step[0] != balance->rc->last_thumbwheel_step[0])
//				{
//					balance->Flag->Turn_Flag = !balance->Flag->Turn_Flag;
//					if(balance->Flag->Turn_Flag == true)
//	        {
//	        	balance->mode=Turn_Mode;
//		        balance->Flag->Mec_Flag = false;
//		        balance->Flag->Imu_Flag = false;
//		        balance->Flag->S_Turn_Flag = false;
//		        balance->Flag->Knee_Strike_Flag = false;
//		        balance->Flag->Fly_Flag = false;
//		        balance->Flag->Reserve_Fly_Flag = false;
//						
//						D_Board_Tx_Pkt.Gimbal_mode = 1;
//         	}
//					else
//					{
//						balance->Flag->Imu_Flag = true;
//						balance->mode=Imu_Mode;
//						D_Board_Tx_Pkt.Gimbal_mode = 1;
//					}
//				}
//				else if(rc_info->thumbwheel.step[2] != balance->rc->last_thumbwheel_step[2])
//				{
//					balance->Flag->Fly_Flag = !balance->Flag->Fly_Flag;
//					
//			  }
//			}
//			else if(rc_info->s2 == RC_SW_DOWN)
//			{
//        if(rc_info->thumbwheel.step[0] != balance->rc->last_thumbwheel_step[0])  
//        {
//					balance->Flag->Imu_Flag = true;
//		      balance->mode = Imu_Mode;

//		      balance->Flag->Mec_Flag = false;
//		      balance->Flag->Turn_Flag = false;
//		      balance->Flag->S_Turn_Flag = false;
////		    balance->Flag->Jumping_Flag = false;
//		      balance->Flag->Knee_Strike_Flag = false;
//		      balance->Flag->Fly_Flag = false;
//		      balance->Flag->Ctrl_Rescue_Flag = false;
//		
//		      balance->Flag->chassis_reset = true;
//		
//		      if(D_Board_Tx_Pkt.vision_mode >= 2)
//		      {
//			      D_Board_Tx_Pkt.vision_mode = 1;
//	        }
			
//			  }
//
//				if(rc_info->thumbwheel.step[2] != balance->rc->last_thumbwheel_step[2])
//		   	{
//			  	balance->Flag->Test_Flag = !balance->Flag->Test_Flag;
//					if(balance->Flag->Test_Flag == true)
//	        {
//		        balance->mode = Test_Mode;
//		        #if CHASSIS_SWITCH == 0
//			         balance->Flag->Chassis_Sleep_Flag = true;
//						   balance->Flag->imu_Flag = true;
//						   
//						   D_Board_Tx_Pkt.Gimbal_mode = 1;
//					
//	         	#else			
//			         balance->Flag->Mec_Flag = true;
//		           balance->Flag->Imu_Flag = false;
//		           balance->Flag->Turn_Flag = false;
//		           balance->Flag->S_Turn_Flag = false;
//						
//						   D_Board_Tx_Pkt.Gimbal_mode = 0;
//		        #endif
//			
//         	}
//					else
//					{
//						balance->Flag->Imu_Flag = true;
//						balance->mode=Imu_Mode;
//						D_Board_Tx_Pkt.Gimbal_mode = 1;
//					}
//				}
//			}

//		
//			break;
//		
//		case RC_SW_MID:
//			if(rc_info->thumbwheel.step[0] != balance->rc->last_thumbwheel_step[0])
//			{
//				D_Board_Tx_Pkt.Launch_state = 1 - D_Board_Tx_Pkt.Launch_state;
//				
//			}
//		
//      #if WHEEL_SHOOT_SWITCH == 0
//			  if(rc_info->thumbwheel.step[2] != balance->rc->last_thumbwheel_step[2])
//			  {
//				  if(D_Board_Rx_Info.vision_state == 0)
//				  {
//					  D_Board_Tx_Pkt.vision_mode = 0;
//				  }
//				  else{
//				    if(D_Board_Tx_Pkt.vision_mode == 0)
//				    {
//					    D_Board_Tx_Pkt.vision_mode = 1;
//						
//						  if(balance->Flag->Mec_Flag == true)
//						  {
//							  balance->Flag->Mec_Flag = false;
//							
//							  balance->Flag->Imu_Flag = true;
//                balance->mode = Imu_Mode;
//						  }
//				    }
//				    else if(D_Board_Tx_Pkt.vision_mode != 0)
//				    {
//					    D_Board_Tx_Pkt.vision_mode = 0;
//				    }
//				  } 
//			  }
//      #else
//        if(rc_info->s2 ==  RC_SW_MID)
//        {
//          D_Board_Tx_Pkt.vision_mode = 0;
//        }
//        else if(rc_info->s2 ==  RC_SW_UP)
//        {
//          D_Board_Tx_Pkt.vision_mode = 1;
//        }
//        else if(rc_info->s2 ==  RC_SW_DOWN)
//        {
//          D_Board_Tx_Pkt.vision_mode = 3;
//        }
//      #endif
//
//			
//			break;
//		
//		case RC_SW_DOWN:

//		    if(rc_info->s2 ==  RC_SW_UP)
//				{
//					if(rc_info->thumbwheel.step[2] != balance->rc->last_thumbwheel_step[2])
//					{
//						D_Board_Tx_Pkt.dial_reset = !D_Board_Tx_Pkt.dial_reset;
//					}
//				}
//		
//			break;
//		
//		default:
//			break;
//		
//	}	
//	
//	
//	if(rc_info->s1 == RC_SW_DOWN && rc_info->s2 == RC_SW_DOWN)
//	{
//		balance->Flag->Key_Flag = true;
//		balance->ctrl = KEY_CTRL;
//		D_Board_Tx_Pkt.car_state = 2;
//	}
//	else{
//		balance->Flag->Key_Flag = false;
//		balance->ctrl = RC_CTRL;
//		D_Board_Tx_Pkt.car_state = 1;
//	}
//	
//	
//	
//	if(balance->Flag->Imu_Flag == true)
//	{
//		balance->Flag->Mec_Flag = false;
//		balance->Flag->Turn_Flag = false;
//		balance->Flag->S_Turn_Flag = false;
//		balance->Flag->Test_Flag = false;
//		
//		D_Board_Tx_Pkt.Gimbal_mode = 1;
//	}
//	
//	if(balance->Flag->Mec_Flag == true)
//	{
//		balance->Flag->Imu_Flag = false;
//		balance->Flag->Turn_Flag = false;
//		balance->Flag->S_Turn_Flag = false;
//		
//		D_Board_Tx_Pkt.Gimbal_mode = 0;
//	}
//	
//  #if WHEEL_SHOOT_SWITCH == 0
//	  if(rc_info->s1 == RC_SW_MID && rc_info->s2 ==  RC_SW_UP)
//	  {
//		  D_Board_Tx_Pkt.Launch_mode = 0;
//		
//		  shoot_statistics.shoot_mode = 0;
//		  shoot_statistics.shooting_flag=0;
//		  if(D_Board_Tx_Pkt.Launch_state == 1)
//		  {
//			  D_Board_Tx_Pkt.is_fire = 1;
//			
//			  if(last_fire == 0)
//			  {
//				  Shooting_Cmd_Excute_Tick_Calculating(0);
//			  }
//			  
//		  }
//	  }
//	  else if(rc_info->s1 == RC_SW_MID && rc_info->s2 ==  RC_SW_MID)
//	  {
//		  D_Board_Tx_Pkt.Launch_mode = 0;
//		  D_Board_Tx_Pkt.is_fire = 0;
//		
//		  shoot_statistics.shoot_mode = 0;
//	    shoot_statistics.shooting_flag = 0;
//	  }

//	  else if(rc_info->s1 == RC_SW_MID && rc_info->s2 == RC_SW_DOWN)
//	  {
//		  D_Board_Tx_Pkt.Launch_mode = 1;
//		  if(D_Board_Tx_Pkt.Launch_state == 1)
//		  {
//			  D_Board_Tx_Pkt.is_fire = 1;
//			
//			  shoot_statistics.shoot_mode = 1;
//			  if(shoot_statistics.shooting_flag == 0)
//			  {
//				  Shooting_Cmd_Excute_Tick_Calculating(0);
//			    shoot_statistics.shooting_flag = 1;
//			  }
//		
//		  }
//	  }
//	  else
//	  {
//		  D_Board_Tx_Pkt.is_fire = 0;
//		
//		  shoot_statistics.shooting_flag=0;
//	  }	
//	#else
//    {
//      if(rc_info->thumbwheel.value <= 0 && rc_info->thumbwheel.value >= -200)
//      {
//        D_Board_Tx_Pkt.is_fire = 0;
//      }
//      else if(rc_info->thumbwheel.value < -200 && rc_info->thumbwheel.value >= -660) 
//      {
//        D_Board_Tx_Pkt.is_fire = 1;
//      }

//    }
//
//  #endif
//	
//	if(D_Board_Tx_Pkt.vision_mode != 0)
//	{
//		balance->Flag->Auto_step = 0;
////		if(rc_info->thumbwheel.step[2] != balance->rc->last_thumbwheel_step[2])
////		{
////			balance->Flag->Auto_step ++;
////			if(balance->Flag->Auto_step >= 4)
////			{
////				balance->Flag->Auto_step = 0;
////			}
////			
////	  }
//	  D_Board_Tx_Pkt.vision_mode = balance->Flag->Auto_step + 1;
//	}
//	else if(D_Board_Tx_Pkt.vision_mode == 0)
//	{
//	  balance->Flag->Auto_step = 0;
//	}

//	
//	if(rc_info->s1 ==  RC_SW_UP && balance->Flag->Jumping_Flag == false 
//		     && balance->Flag->Knee_Strike_Flag == false && balance->Flag->Fly_Flag ==false && balance->Flag->Rescue_Flag == false
//					 && Chassis.Leg_Unit[R_Leg]->off_ground == false && Chassis.Leg_Unit[L_Leg]->off_ground == false)
//	{
//		
//		balance->Flag->Leg_length_ctrl_Flag = true;
//	}
//	else{
//		 balance->Flag->Leg_length_ctrl_Flag = false;
//	} 
//					

//	balance->rc->last_s2 = rc_info->s2;
//	balance->rc->last_thumbwheel_step[0] = rc_info->thumbwheel.step[0];
//	balance->rc->last_thumbwheel_step[2] = rc_info->thumbwheel.step[2];
//	last_fire = D_Board_Tx_Pkt.is_fire;
//	if(balance->last_mode != balance->mode)
//	{
//		balance->last_mode = balance->mode;
//	}

//}


static void RC_Move_Mode_Update(Balance_t* balance)
{
	
	rc_sensor_info_t*  rc_info=balance->rc->sensor->info;
	
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
		        balance->Flag->Mec_Flag = false;
		        balance->Flag->Imu_Flag = false;
		        balance->Flag->S_Turn_Flag = false;
		        balance->Flag->Knee_Strike_Flag = false;
		        balance->Flag->Fly_Flag = false;
		        balance->Flag->Reserve_Fly_Flag = false;
						
						D_Board_Tx_Pkt.Gimbal_mode = 1;
         	}
					else
					{
						balance->Flag->Imu_Flag = true;
						balance->mode=Imu_Mode;
						D_Board_Tx_Pkt.Gimbal_mode = 1;
					}
				}
				else if(rc_info->thumbwheel.step[2] != balance->rc->last_thumbwheel_step[2])
				{
					balance->Flag->S_Turn_Flag = !balance->Flag->S_Turn_Flag;
					if(balance->Flag->S_Turn_Flag == true)
	        {
	         	balance->mode=Turn_Mode;
		        balance->Flag->Mec_Flag = false;
		        balance->Flag->Imu_Flag = false;
		        balance->Flag->Turn_Flag = false;
		        balance->Flag->Knee_Strike_Flag = false;
		        balance->Flag->Fly_Flag = false;
		        balance->Flag->Reserve_Fly_Flag = false;
						
						D_Board_Tx_Pkt.Gimbal_mode = 1;
	        }			
          else
					{
						balance->Flag->Imu_Flag = true;
						balance->mode=Imu_Mode;
						D_Board_Tx_Pkt.Gimbal_mode = 1;
					}
			  }
			}
//			else if(rc_info->s2 == RC_SW_DOWN)
//			{
//				if(rc_info->thumbwheel.step[0] != balance->rc->last_thumbwheel_step[0])
//		   	{
//			  	balance->Flag->Test_Flag = !balance->Flag->Test_Flag;
//					if(balance->Flag->Test_Flag == true)
//	        {
//		        balance->mode = Test_Mode;
//		        #ifdef VISION_TEST
//			         balance->Flag->Chassis_Sleep_Flag = true;
//						   balance->Flag->imu_Flag = true;
//						   
//						   D_Board_Tx_Pkt.Gimbal_mode = 1;
//					
//	         	#else			
//			         balance->Flag->Mec_Flag = true;
//		           balance->Flag->Imu_Flag = false;
//		           balance->Flag->Turn_Flag = false;
//		           balance->Flag->S_Turn_Flag = false;
//						
//						   D_Board_Tx_Pkt.Gimbal_mode = 0;
//		        #endif
//			
//         	}
//					else
//					{
//						balance->Flag->Imu_Flag = true;
//						balance->mode=Imu_Mode;
//						D_Board_Tx_Pkt.Gimbal_mode = 1;
//					}
//				}
//				
//				else if(rc_info->thumbwheel.step[2] != balance->rc->last_thumbwheel_step[2])
//				{
//					balance->Flag->Key_Flag = !balance->Flag->Key_Flag;
//					if(balance->Flag->Key_Flag == true)
//          {
////		        balance->mode = Key_Mode;

//            balance->ctrl = KEY_CTRL;
//  	        D_Board_Tx_Pkt.car_state = 2;
//	        }
//	        else{
////	          balance->mode = balance->last_mode;
//	          balance->ctrl = RC_CTRL;
//			   
//          	D_Board_Tx_Pkt.car_state = 1;
//      	  }
//				}
//			}
		
			break;
		
		case RC_SW_MID:
			if(rc_info->thumbwheel.step[0] != balance->rc->last_thumbwheel_step[0])
			{
				D_Board_Tx_Pkt.Launch_state = 1 - D_Board_Tx_Pkt.Launch_state;
				
			}
		
			if(rc_info->s2 == RC_SW_DOWN  && balance->rc->last_s2 == RC_SW_MID)
			{
				if(D_Board_Rx_Info.vision_state == 0)
				{
					D_Board_Tx_Pkt.vision_mode = 0;
				}
				else{
				  if(D_Board_Tx_Pkt.vision_mode == 0)
				  {
					  D_Board_Tx_Pkt.vision_mode = 1;
						
						if(balance->Flag->Mec_Flag == true)
						{
							balance->Flag->Mec_Flag = false;
							
							balance->Flag->Imu_Flag = true;
						}
				  }
				  else if(D_Board_Tx_Pkt.vision_mode != 0)
				  {
					  D_Board_Tx_Pkt.vision_mode = 0;
				  }
				} 
			}
			
			break;
		
		case RC_SW_DOWN:
//			if(rc_info->s2 ==  RC_SW_MID)
//			{
//				if(rc_info->thumbwheel.step[0] != balance->rc->last_thumbwheel_step[0])
//				{
//					balance->Flag->U_Turn_Flag = !balance->Flag->U_Turn_Flag;
//				}
//				
//			}
		
		  #if DIAL_RESET_SWITCH == 0
		
		  #else
		    if(rc_info->s2 ==  RC_SW_MID)
				{
					if(rc_info->thumbwheel.step[2] != balance->rc->last_thumbwheel_step[2])
					{
						D_Board_Tx_Pkt.dial_reset = !D_Board_Tx_Pkt.dial_reset;
					}
				}
		
		  #endif
		    
//			else if(rc_info->s2 ==  RC_SW_UP)
//			{
//				if(rc_info->thumbwheel.step[0] != balance->rc->last_thumbwheel_step[0])
//				{
//					balance->Flag->Jumping_Flag = !balance->Flag->Jumping_Flag;
//				}
//				else if(rc_info->thumbwheel.step[2] != balance->rc->last_thumbwheel_step[2])
//				{
//					balance->Flag->Knee_Strike_Flag = !balance->Flag->Knee_Strike_Flag;
//				}
//			}
			if(rc_info->s2 ==  RC_SW_DOWN)
			{
				if(rc_info->thumbwheel.step[0] != balance->rc->last_thumbwheel_step[0])
				{
					balance->Flag->Fly_Flag = !balance->Flag->Fly_Flag;
				}
//				else if(rc_info->thumbwheel.step[2] != balance->rc->last_thumbwheel_step[2])
//				{
//					balance->Flag->Reserve_Fly_Flag = !balance->Flag->Reserve_Fly_Flag;
//				}
			}
		
			break;
		
		default:
			break;
		
	}	
	
	
	if(rc_info->s1 == RC_SW_UP && rc_info->s2 == RC_SW_DOWN)
	{
		balance->Flag->Key_Flag = true;
		balance->ctrl = KEY_CTRL;
		D_Board_Tx_Pkt.car_state = 2;
	}
	else{
		balance->Flag->Key_Flag = false;
		balance->ctrl = RC_CTRL;
		D_Board_Tx_Pkt.car_state = 1;
	}
	
	
	
	if(balance->Flag->Imu_Flag == true)
	{
		balance->Flag->Mec_Flag = false;
		balance->Flag->Turn_Flag = false;
		balance->Flag->S_Turn_Flag = false;
		balance->Flag->Test_Flag = false;
		
		D_Board_Tx_Pkt.Gimbal_mode = 1;
	}
	
	if(balance->Flag->Mec_Flag == true)
	{
		balance->Flag->Imu_Flag = false;
		balance->Flag->Turn_Flag = false;
		balance->Flag->S_Turn_Flag = false;
		
		D_Board_Tx_Pkt.Gimbal_mode = 0;
	}
	
					
//	if(rc_info->s1 == RC_SW_UP && rc_info->s2 ==  RC_SW_UP && balance->rc->last_s2 == RC_SW_MID)
//	{
//		D_Board_Tx_Pkt.Launch_mode = 0;
//		if(D_Board_Tx_Pkt.Launch_state == 1)
//		{
//			D_Board_Tx_Pkt.is_fire = 1;
//			Shooting_Cmd_Excute_Tick_Calculating(0);
//		}
//	}
	
	if(rc_info->s1 == RC_SW_UP && rc_info->s2 ==  RC_SW_UP)
	{
		D_Board_Tx_Pkt.Launch_mode = 0;
		
		shoot_statistics.shoot_mode = 0;
		shoot_statistics.shooting_flag=0;
		if(D_Board_Tx_Pkt.Launch_state == 1)
		{
			D_Board_Tx_Pkt.is_fire = 1;
			
			if(last_fire == 0)
			{
				Shooting_Cmd_Excute_Tick_Calculating(0);
			}
			  
		}
	}
	else if(rc_info->s1 == RC_SW_UP && rc_info->s2 ==  RC_SW_MID)
	{
		D_Board_Tx_Pkt.Launch_mode = 0;
		D_Board_Tx_Pkt.is_fire = 0;
		
		shoot_statistics.shoot_mode = 0;
	  shoot_statistics.shooting_flag = 0;
	}

	else if(rc_info->s1 == RC_SW_MID && rc_info->s2 == RC_SW_UP)
	{
		D_Board_Tx_Pkt.Launch_mode = 1;
		if(D_Board_Tx_Pkt.Launch_state == 1)
		{
			D_Board_Tx_Pkt.is_fire = 1;
			
			shoot_statistics.shoot_mode = 1;
			if(shoot_statistics.shooting_flag == 0)
			{
				Shooting_Cmd_Excute_Tick_Calculating(0);
			  shoot_statistics.shooting_flag = 1;
			}
		
		}
	}
	else if(rc_info->s1 == RC_SW_MID && rc_info->s2 == RC_SW_MID)
	{
		D_Board_Tx_Pkt.Launch_mode = 1;
		D_Board_Tx_Pkt.is_fire = 0;
		
		shoot_statistics.shoot_mode = 1;
	  shoot_statistics.shooting_flag = 0;
	}
	
	else
	{
		D_Board_Tx_Pkt.is_fire = 0;
		
		shoot_statistics.shooting_flag=0;
	}	
	
	
	if(D_Board_Tx_Pkt.vision_mode != 0)
	{
		if(rc_info->thumbwheel.step[2] != balance->rc->last_thumbwheel_step[2])
		{
			D_Board_Tx_Pkt.vision_mode ++;
			if(D_Board_Tx_Pkt.vision_mode > 5)
			{
				D_Board_Tx_Pkt.vision_mode = 1;
			}
			
	  }
	}

	
	if(rc_info->s1 ==  RC_SW_DOWN && rc_info->s2 ==  RC_SW_DOWN && balance->Flag->Jumping_Flag == false 
		     && balance->Flag->Knee_Strike_Flag == false && balance->Flag->Fly_Flag ==false && balance->Flag->Rescue_Flag == false
					 )//&& Chassis.Leg_Unit[R_Leg]->off_ground == false && Chassis.Leg_Unit[L_Leg]->off_ground == false
	{
		
		balance->Flag->Leg_length_ctrl_Flag = true;
	}
	else{
		 balance->Flag->Leg_length_ctrl_Flag = false;
	} 
					
	
	
	
	balance->rc->last_s2 = rc_info->s2;
	balance->rc->last_thumbwheel_step[0] = rc_info->thumbwheel.step[0];
	balance->rc->last_thumbwheel_step[2] = rc_info->thumbwheel.step[2];
	last_fire = D_Board_Tx_Pkt.is_fire;
	if(balance->last_mode != balance->mode)
	{
		balance->last_mode = balance->mode;
	}
	
}


static void Key_Move_Mode_Update(Balance_t* balance)
{
	static uint32_t last_rescue_time = 0;
	static uint8_t last_rescue_cnt = 0;
	rc_sensor_info_t*  rc_info=balance->rc->sensor->info;
	
//  if(rc_info->s1 == RC_SW_UP && rc_info->s2 == RC_SW_DOWN && balance->rc->last_thumbwheel_step[2] != rc_info->thumbwheel.step[2])
//  {
//    if(balance->Flag->Key_Flag == true)
//    {
//      balance->Flag->Key_Flag = false;
//      balance->ctrl = RC_CTRL;
//			D_Board_Tx_Pkt.car_state = 1;

//    }
//  }
	
	
	if(rc_info->s1 == RC_SW_UP && rc_info->s2 == RC_SW_DOWN)
	{
		balance->Flag->Key_Flag = true;
		balance->ctrl = KEY_CTRL;
		D_Board_Tx_Pkt.car_state = 2;
	}
	else{
		balance->Flag->Key_Flag = false;
		balance->ctrl = RC_CTRL;
		D_Board_Tx_Pkt.car_state = 1;
	}

	
	if(rc_info->Z.status == release_to_press)
	{
		if(balance->Flag->rescue_cnt == 0)
		{
			balance->Flag->rescue_cnt = 1;
		}
		else if(balance->Flag->rescue_cnt > 0 && last_rescue_cnt != 0)
		{
			if(HAL_GetTick() - last_rescue_time <= 250)
			{
				balance->Flag->rescue_cnt ++;
			}
		}
		if(balance->Flag->rescue_cnt >= 5)
	  {
		  balance->Flag->Ctrl_Rescue_Flag = true;
	 	  balance->Flag->rescue_cnt = 0;
	  }		
		
		last_rescue_time = HAL_GetTick();
		last_rescue_cnt = balance->Flag->rescue_cnt;
	}	
	
	if(HAL_GetTick() - last_rescue_time > 250)
	{
		balance->Flag->rescue_cnt = 0;
		last_rescue_cnt = 0;
	}
//		balance->Flag->Mec_Flag = !balance->Flag->Mec_Flag;
//		if(balance->Flag->Mec_Flag == true)
//		{
//			balance->mode = Mec_Mode;
//			
//			balance->Flag->Imu_Flag = false;
//		  balance->Flag->Turn_Flag = false;
//			balance->Flag->S_Turn_Flag = false;
//		  balance->Flag->Knee_Strike_Flag = false;
//		  balance->Flag->Fly_Flag = false;
//		  balance->Flag->Reserve_Fly_Flag = false;
//			
//			D_Board_Tx_Pkt.Gimbal_mode = 0;
//		}
//		else
//		{
//			balance->mode = Imu_Mode;
//			
//			balance->Flag->Imu_Flag = true;
//			balance->Flag->Mec_Flag = false;
//		  balance->Flag->Turn_Flag = false;
//			balance->Flag->S_Turn_Flag = false;
//		  balance->Flag->Knee_Strike_Flag = false;
//		  balance->Flag->Fly_Flag = false;
//		  balance->Flag->Reserve_Fly_Flag = false;
//			
//			D_Board_Tx_Pkt.Gimbal_mode = 1;
//		}
//	}
	
	if(rc_info->Shift.status == release_to_press)
	{
		balance->Flag->Turn_Flag = true;
		balance->mode = Turn_Mode;
		
		balance->Flag->Mec_Flag = false;
    balance->Flag->Imu_Flag = false;
    balance->Flag->S_Turn_Flag = false;
		balance->Flag->Jumping_Flag = false;
    balance->Flag->Knee_Strike_Flag = false;
    balance->Flag->Fly_Flag = false;
    balance->Flag->Reserve_Fly_Flag = false;
		
		D_Board_Tx_Pkt.Gimbal_mode = 1;

//		balance->Flag->Turn_Flag = !balance->Flag->Turn_Flag;
//		if(balance->Flag->Turn_Flag == true)
//		{
//			balance->mode = Turn_Mode;

//      balance->Flag->Mec_Flag = false;
//		  balance->Flag->Imu_Flag = false;
//		  balance->Flag->S_Turn_Flag = false;
//		  balance->Flag->Knee_Strike_Flag = false;
//		  balance->Flag->Fly_Flag = false;
//		  balance->Flag->Reserve_Fly_Flag = false;
//			
//			D_Board_Tx_Pkt.Gimbal_mode = 1;
//		}
//		else
//		{
//			balance->mode = Imu_Mode;

//      balance->Flag->Imu_Flag = true;
//      balance->Flag->Mec_Flag = false;
//		  balance->Flag->S_Turn_Flag = false;
//			
//			D_Board_Tx_Pkt.Gimbal_mode = 1;
//		}
	}
	
	if(rc_info->B.status == release_to_press)
	{
		D_Board_Tx_Pkt.Launch_state = 1 - D_Board_Tx_Pkt.Launch_state;
	}
	
	
	if(D_Board_Rx_Info.vision_state == 0)
	{
		D_Board_Tx_Pkt.vision_mode = 0;
	}	
	else{
	  if(rc_info->mouse_btn_r.status != long_press && D_Board_Tx_Pkt.vision_mode <= 1)
	  {
		  D_Board_Tx_Pkt.vision_mode = 0;

	  }
	  else if(rc_info->mouse_btn_r.status == long_press && D_Board_Tx_Pkt.vision_mode <= 1)
	  {
		  if(balance->Flag->Mec_Flag == true)
		  {
			  balance->mode = Imu_Mode;
			
			  balance->Flag->Imu_Flag = true;
			  balance->Flag->Mec_Flag = false;
		  }
		
		  D_Board_Tx_Pkt.vision_mode = 1;
	  }
	
	  if(rc_info->X.status == release_to_press)
	  {
		  D_Board_Tx_Pkt.vision_mode = 2;
		
		  if(balance->Flag->Mec_Flag == true)
		  {
			  balance->mode = Imu_Mode;
			
			  balance->Flag->Imu_Flag = true;
			  balance->Flag->Mec_Flag = false;
		  }
	  }
	  else if(rc_info->X.status == long_press)
	  {
		  D_Board_Tx_Pkt.vision_mode = 3;

		  if(balance->Flag->Mec_Flag == true)
		  {
			  balance->mode = Imu_Mode;
			
			  balance->Flag->Imu_Flag = true;
			  balance->Flag->Mec_Flag = false;
		  }
	  }
		
		if(rc_info->V.status == release_to_press)
	  {
		  D_Board_Tx_Pkt.vision_mode = 4;

		  if(balance->Flag->Mec_Flag == true)
		  {
			  balance->mode = Imu_Mode;
			
			  balance->Flag->Imu_Flag = true;
			  balance->Flag->Mec_Flag = false;
		  }
	  }
	
	} 
	
	
//	if((rc_info->mouse_btn_r.status != long_press && D_Board_Tx_Pkt.vision_mode <= 1)|| D_Board_Rx_Info.vision_state == 0)
//	{
//		D_Board_Tx_Pkt.vision_mode = 0;

//	}
//	else if(rc_info->mouse_btn_r.status == long_press && D_Board_Rx_Info.vision_state == 1 && D_Board_Tx_Pkt.vision_mode <= 1)
//	{
//		if(balance->Flag->Mec_Flag == true)
//		{
//			balance->mode = Imu_Mode;
//			
//			balance->Flag->Imu_Flag = true;
//			balance->Flag->Mec_Flag = false;
//		}
//		
//		D_Board_Tx_Pkt.vision_mode = 1;
//	}
//	
//	if(rc_info->X.status == release_to_press && D_Board_Rx_Info.vision_state == 1)
//	{
//		D_Board_Tx_Pkt.vision_mode = 2;
//		
//		if(balance->Flag->Mec_Flag == true)
//		{
//			balance->mode = Imu_Mode;
//			
//			balance->Flag->Imu_Flag = true;
//			balance->Flag->Mec_Flag = false;
//		}
//	}
//	else if(rc_info->X.status == long_press && D_Board_Rx_Info.vision_state == 1)
//	{
//		D_Board_Tx_Pkt.vision_mode = 3;

//		if(balance->Flag->Mec_Flag == true)
//		{
//			balance->mode = Imu_Mode;
//			
//			balance->Flag->Imu_Flag = true;
//			balance->Flag->Mec_Flag = false;
//		}
//	}
//	
//	if(rc_info->V.status == release_to_press && D_Board_Rx_Info.vision_state == 1)
//	{
//		D_Board_Tx_Pkt.vision_mode = 4;
//		
//		if(balance->Flag->Mec_Flag == true)
//		{
//			balance->mode = Imu_Mode;
//			
//			balance->Flag->Imu_Flag = true;
//			balance->Flag->Mec_Flag = false;
//		}
//	}

	
//	if(rc_info->R.status == release_to_press)
//	{
//		balance->Flag->U_G_Turn_Flag = true;
//    balance->Flag->U_C_Turn_Flag = true;	
//		balance->Flag->R_Turn_Flag = false;
//		balance->Flag->L_Turn_Flag = false;
//	}

//	if(rc_info->Q.status == release_to_press)
//	{
//		balance->Flag->L_Turn_Flag = true;
//		balance->Flag->R_Turn_Flag = false;
//		balance->Flag->U_G_Turn_Flag = false;
//    balance->Flag->U_C_Turn_Flag = false;	
//	}
//	
//	if(rc_info->E.status == release_to_press)
//	{
//		balance->Flag->R_Turn_Flag = true;
//		balance->Flag->L_Turn_Flag = false;
//		balance->Flag->U_G_Turn_Flag = false`;
//    balance->Flag->U_C_Turn_Flag = false;	
//	}
	
	if(rc_info->Ctrl.status == release_to_press) 
	{
		balance->Flag->Imu_Flag = true;
		balance->mode = Imu_Mode;
		
		balance->Flag->Mec_Flag = false;
		balance->Flag->Turn_Flag = false;
		balance->Flag->S_Turn_Flag = false;
		balance->Flag->U_G_Turn_Flag = false;
		balance->Flag->U_C_Turn_Flag = false;
//		balance->Flag->Jumping_Flag = false;
		balance->Flag->Knee_Strike_Flag = false;
		balance->Flag->Fly_Flag = false;
		balance->Flag->Ctrl_Rescue_Flag = false;
		
		balance->Flag->chassis_reset = true;
		
		if(D_Board_Tx_Pkt.vision_mode >= 2 && rc_info->mouse_btn_r.status == release)
		{
			D_Board_Tx_Pkt.vision_mode = 0;
		}
		else if(D_Board_Tx_Pkt.vision_mode >= 2 && rc_info->mouse_btn_r.status == long_press)
		{
			D_Board_Tx_Pkt.vision_mode = 1;
		}
		
	}
	
	if(D_Board_Tx_Pkt.vision_mode == 0)
	{
	}

	if(rc_info->mouse_btn_l.status == release_to_press)
  {
	  D_Board_Tx_Pkt.Launch_mode = 0;
		if(D_Board_Tx_Pkt.Launch_state == 1)
	  {
	   D_Board_Tx_Pkt.is_fire = 1;
	  }
		else
		{
			D_Board_Tx_Pkt.is_fire = 0;
		}
	}
  else if(rc_info->mouse_btn_l.status == long_press)
	{
	  D_Board_Tx_Pkt.Launch_mode = 1;
		if(D_Board_Tx_Pkt.Launch_state == 1)
	  {
      D_Board_Tx_Pkt.is_fire = 1;		
    }
		else
		{
			D_Board_Tx_Pkt.is_fire = 0;
		}
  }
	else{
	  D_Board_Tx_Pkt.is_fire = 0;
  }
		 
		 
//	if(rc_info->V.status == release_to_press)
//	{
//		balance->Flag->Jumping_Flag = !balance->Flag->Jumping_Flag;
//	}
//	
//	if(rc_info->X.status == release_to_press)
//	{
//		balance->Flag->Knee_Strike_Flag = !balance->Flag->Knee_Strike_Flag;
//	}
	
//	if((rc_info->Shift.status == release_to_press || rc_info->Shift.status == short_press || rc_info->Shift.status == long_press)
//		 && rc_info->X.status == release_to_press)
//	{
//		
//	}
	
//	if((rc_info->Shift.status == release_to_press || rc_info->Shift.status == short_press || rc_info->Shift.status == long_press)
//		 && rc_info->C.status == release_to_press)
//	{
//		balance->Flag->S_Turn_Flag = !balance->Flag->S_Turn_Flag;
//		
//		D_Board_Tx_Pkt.Gimbal_mode = 1;
//		if(balance->Flag->S_Turn_Flag == true)
//		{
//			balance->mode = Turn_Mode;

//      balance->Flag->Mec_Flag = false;
//		  balance->Flag->Imu_Flag = false;
//		  balance->Flag->Turn_Flag = false;
//		  balance->Flag->Knee_Strike_Flag = false;
//		  balance->Flag->Fly_Flag = false;
//		  balance->Flag->Reserve_Fly_Flag = false;
//			
//			D_Board_Tx_Pkt.Gimbal_mode = 1;
//		}
//    else
//    {
//			balance->mode = Imu_Mode;

//      balance->Flag->Mec_Flag = false;
//		  balance->Flag->Imu_Flag = true;
//		  balance->Flag->Turn_Flag = false;
//			
//			D_Board_Tx_Pkt.Gimbal_mode = 1;     
//    }
//	}
	
//	if((rc_info->Shift.status == release_to_press || rc_info->Shift.status == short_press || rc_info->Shift.status == long_press)
//		 && rc_info->Z.status == release_to_press)
//	{
//		balance->Flag->Lob_Flag = !balance->Flag->Lob_Flag;
//		if(balance->Flag->Lob_Flag == true)
//		{
//			balance->mode = Mec_Mode;
//		}
//		else
//		{
//			balance->mode = Imu_Mode;
//		}
//	}
	
	
//	#ifdef DIAL_RESET_CTRL
//	if(rc_info->G.status == release_to_press)
//	{
//		D_Board_Tx_Pkt.is_dial_self_reset = 1;
//  }
//  else
//  { 
//    D_Board_Tx_Pkt.is_dial_self_reset = 0;
//  }
//	
//	#else
//	 D_Board_Tx_Pkt.dial_reset = 1;
//	
//	#endif
	
	
//	if(rc_info->F.status == release_to_press)
//	{
//		balance->Flag->Fly_step ++;
//		
////		if(balance->Flag->Fly_step % 3 == 2)
////		{
////			balance->Flag->Reserve_Fly_Flag = 1;
////			balance->Flag->Fly_Flag = 0;
////		}
////		else if(balance->Flag->Fly_step % 3 == 1)
////		{
////			balance->Flag->Fly_Flag = 1;
////			balance->Flag->Reserve_Fly_Flag = 0;
////		}
////		else if(balance->Flag->Fly_step % 3 == 0)
////		{
////			balance->Flag->Reserve_Fly_Flag = 0;
////			balance->Flag->Fly_Flag = 0;
////		}
//	}
   
	if((rc_info->Z.status == release_to_press || rc_info->Z.status == short_press) 
		&& (rc_info->X.status == release_to_press || rc_info->X.status == short_press)
	  && (rc_info->C.status == release_to_press || rc_info->C.status == short_press))
  {
	  balance->Flag->Ctrl_Rescue_Flag = !balance->Flag->Ctrl_Rescue_Flag;
	}
  else if(rc_info->Z.status == long_press && rc_info->X.status == long_press && rc_info->C.status == long_press)
	{
		balance->Flag->Power_Limit_Flag = !balance->Flag->Power_Limit_Flag;
	}	

//  if(D_Board_Tx_Pkt.vision_mode != 0)
//	{
//		if(rc_info->thumbwheel.step[2] != balance->rc->last_thumbwheel_step[2])
//		{
//			balance->Flag->Auto_step ++;
//			if(balance->Flag->Auto_step >= 4)
//			{
//				balance->Flag->Auto_step = 0;
//			}
//			D_Board_Tx_Pkt.vision_mode = balance->Flag->Auto_step + 1;
//	  }
//	  else if(D_Board_Tx_Pkt.vision_mode == 0)
//	  {
//		  balance->Flag->Auto_step = 0;
//	  }
//	}

   balance->rc->last_thumbwheel_step[2] = rc_info->thumbwheel.step[2];  

}

