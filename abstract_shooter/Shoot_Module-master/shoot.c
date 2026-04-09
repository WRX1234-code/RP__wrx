#include "shoot.h"
static void Motor_Set_AngleSum(Motor_RM_t *motor);
/*--------------------------------------------------*/
/*
个人用于测试的一份代码
此文件仅供参考
- 注意其中is_ready_flag标志位在初始化时为0
- elec_level_flag为操作标志位，单发状态且为上升沿时开火，连发且为高电平时开火，例如定义鼠标左键按下为1，单发状态按下鼠标触发开火操作，具体关联自行定义
- 数据配置请根据实际情况谨慎配置

*/
//#define REMOTE_CTRL//无遥控器debug时使用
//#define ABSOLUTE_ANGLE//有无绝对角度


void Shoot_Update(Shoot_t *shoot){
	//外部拨盘电机数据传入shoot模块
	Dial_Rt_Rx_Info_t *dial=&shoot->info.rt_rx_info.dial_info;
	Motor_RM_Rx_Info_t *motor_info=Dial.rx_info;
//	Motor_DM_Rx_Info_t *motor_info=DM_Motor.rx_info;
	
//	dial->angle=motor_info->motor_angle;
//	dial->current=motor_info->torque;
//	dial->speed=motor_info->speed;
//	
	dial->angle=motor_info->encoder;
	dial->current=motor_info->torque_current_raw;
	dial->speed=motor_info->speed;
	//dial->angle_sum=motor_info->encoder_sum ;
	//外部指令传入
	#ifdef REMOTE_CTRL 
	rc_sensor_info_t *rc=&rc_sensor_info;
	Flag_Rt_Rx_Info_t *flag=&shoot->info.rt_rx_info.flag_Info;
	switch (rc->s1.status){
		case mid_R:{
			flag->fire_mode_flag=0;
			flag->is_sleep_flag=0;
			break;
		}
		case down_R:{
			flag->fire_mode_flag=1;
			flag->is_sleep_flag=0;
			break;
		}
		default :
			flag->is_sleep_flag=1;
		break;
	}
	
	#else
	//没遥控器手动debug看情况

	#endif
}

//接收电机数据
void Get_data(Shoot_t *shoot){
	pid_struct_t* pid=&DM_pid;
	Motor_RM_Ctrl_Info_t *motor_info=Dial.ctrl;
	Dial_Tx_Cmd_t *dial_tx= &shoot->cmd.dial_tx_cmd ;
	
	if(shoot->cmd.fric_tx_cmd.work_state==RUN){
	}
	else{
	}
	
	#ifdef ABSOLUTE_ANGLE
	if(shoot->cmd.dial_tx_cmd.work_state==SLEEP){
		DM_Motor.single_sleep(&DM_Motor);
	}
	else{
		switch(dial_tx->mode){
			case DIAL_SPEED:{
				pid->single_pid->speed.target=dial_tx->speed_target;
				pid->single_pid->speed.measure=DM_Motor.rx_info->speed;
				DM_ctrl_speed(&DM_Motor);
				break ;
			}
			case DIAL_ANGLE:{
				pid->double_pid->angle.target=dial_tx->angle_target;
				pid->double_pid->angle.measure=DM_Motor.rx_info->motor_angle;
				pid->double_pid->speed.measure=DM_Motor.rx_info->speed;
				DM_ctrl_angle(&DM_Motor);
				break;
			}
			default:
				DM_Motor.single_sleep(&DM_Motor);
			break;
		}
	}
	#else
	if(shoot->cmd.dial_tx_cmd.work_state==SLEEP){
		Dial.single_sleep(&Dial);
	}
	else{
		switch(dial_tx->mode){
			case DIAL_SPEED:{
				motor_info->speed_ctrl->target=dial_tx->speed_target;
				Dial.single_set_speed(&Dial);
				break ;
			}
			case DIAL_ANGLE:{
				motor_info->angle_ctrl_outer->target=dial_tx->angle_sum_target;
				Motor_Set_AngleSum(&Dial);
				break;
			}
			default:
				Dial.single_sleep(&Dial);
			break;
		}
	}
	#endif
}

void Shoot_Sendata(){
	Dial.single_set_torque(&Dial);
	//DM_Motor.single_set_torque(&DM_Motor);
}
/*-----------------------函数工具--------------------------------------*/
/**
  * @brief          单电机控制角度(不含发送函数，需要再控制扭矩才可以控制)
  * @param[in]      Motor_RM_t *motor     
  * @retval         none
  */
static void Motor_Set_AngleSum(Motor_RM_t *motor)
{
	pid_ctrl_t* my_angle_ctrl = motor->ctrl->angle_ctrl_outer;
	pid_ctrl_t* my_speed_ctrl = motor->ctrl->angle_ctrl_inner;
	/*外环计算*/
	if(motor->ctrl->Angle_Input_Flag == false)
	{
	my_angle_ctrl->measure = motor->rx_info->encoder_sum;
	}
	my_angle_ctrl->err=my_angle_ctrl->target-my_angle_ctrl->measure;
	if(motor->ctrl->Nearest_Return == true)
	{
		if(motor->ctrl->Angle_Input_Flag == false)
		{
			if(my_angle_ctrl->err < 0)
			{
				my_angle_ctrl->err += 8192;
			}
			if(my_angle_ctrl->err > 4096)
			{
				my_angle_ctrl->err -= 8192;
			}
		}
		else
		{
			if(my_angle_ctrl->err < -180.f)
			{
				my_angle_ctrl->err += 360.f;
			}
			if(my_angle_ctrl->err > 180.f)
			{
				my_angle_ctrl->err -= 360.f;
			}
		}
	}
	single_pid_ctrl(my_angle_ctrl);
	
	my_speed_ctrl->target = my_angle_ctrl->out;
	if(motor->ctrl->Speed_Input_Flag == false)
	my_speed_ctrl->measure = motor->rx_info->encoder_speed;
	my_speed_ctrl->err=my_speed_ctrl->target-my_speed_ctrl->measure;
	single_pid_ctrl(my_speed_ctrl);
	motor->tx_info->torque = my_speed_ctrl->out;
}
