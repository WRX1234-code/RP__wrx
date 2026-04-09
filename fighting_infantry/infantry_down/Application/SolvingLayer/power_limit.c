#include "power_limit.h"
#include "chassis.h"
#include "car_info.h"
#include "chassis_motor.h"
#include "balance.h"
#include "judge.h"

///*功率限制*/
//float a = 0,Nf = 0,kk;
//void Chassis_Power_Limit(void)
//{
//	static float Power_limit = 60.f;
//	float Tw_Enable = 2.2f,Tp_Big;//阈值得调
//	
//	Tw_Enable = My_Judge.org_info->power_heat_data.buffer_energy / 24.f * _3508_TORQUE_CONSTANT;				
//	
//	Link_Var_t *Link_Var;
//	if(abs(Straight_Leg.r_leg->Tp) >= abs(Straight_Leg.l_leg->Tp))
//	{
//		Tp_Big = abs(Straight_Leg.r_leg->Tp);
//		Link_Var = &My_Link_Var[R_Link];
//	}
//	else
//	{
//		Tp_Big = abs(Straight_Leg.l_leg->Tp);
//		Link_Var = &My_Link_Var[L_Link];
//	}
//	
//	Nf = (float)(Tw_Enable / WHEEL_RADIUS) - (float)((2*Tp_Big / My_Chassis.target->leg_length) * abs(arm_cos_f32(Link_Var->angle->vir_phi0))) \
//		                                    - (float)(mb*g*(My_Chassis.target->leg_length)*abs(arm_sin_f32(Link_Var->angle->vir_phi0))) \
//																				- (float)(mb*g*abs(arm_cos_f32(Link_Var->angle->vir_phi0))*abs(arm_sin_f32(Link_Var->angle->vir_phi0)));
//	
//	kk = arm_sin_f32(Link_Var->angle->vir_phi0);
//	
//	a = (float)Nf/mb;//质量得换整车的
//	
//	My_Chassis.target->velocity_limit = My_Chassis.target->velocity_limit + a*TIME_STEP;
//	
//	if(My_Chassis.target->velocity_limit >= MAX_STRAIGHT_SPEED)
//	{
//		My_Chassis.target->velocity_limit = MAX_STRAIGHT_SPEED;
//	}
//	
//	if(My_Chassis.target->velocity_limit <= 0.4)//阈值得调
//	{
//		My_Chassis.target->velocity_limit = 0.4;
//	}
//}

//void Chassis_Power_Save(void)
//{
//	if(My_Balance.command->chassis->Over_Power_Flag == true)
//	{
//		My_Balance.command->chassis->Top_Flag = false;//防止起来后转动功率太大
//		//My_Chassis.save_power_cnt++;
//		if(My_Chassis.save_power_cnt <= DAMPING_DELAY_MAX_CNT)
//		{
//			Chassis_Stop_Damping();
//			My_Chassis.save_power_cnt++;
//		}
//		else
//		{
//            Motor_DM_Group.motor[R_F_Sd_Motor]->tx_info->torque = 0;
//	
//            Motor_DM_Group.motor[R_B_Sd_Motor]->tx_info->torque = 0;
//		
//            Motor_DM_Group.motor[L_F_Sd_Motor]->tx_info->torque = 0;
//		
//            Motor_DM_Group.motor[L_B_Sd_Motor]->tx_info->torque = 0;
//		
//            Wheel_Group[R_Wheel_Motor].tx_info->torque = 0;
//            Wheel_Group[L_Wheel_Motor].tx_info->torque = 0;

//		}
//	}
//	else
//	{
//		My_Chassis.save_power_cnt = 0;
//	}
//}