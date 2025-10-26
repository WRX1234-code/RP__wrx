/**
  ******************************************************************************
  * @file    RM_motor.c
  * @brief   �������
  * @version 
  * @date    
  ******************************************************************************
  * @attention
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "rm_motor.h"
#include "pid.h"
#include "arm_math.h"
#include "rp_math.h"
static uint16_t CAN_01_GetMotorAngle(uint8_t *rxData);
static int16_t CAN_23_GetMotorSpeed(uint8_t *rxData);
static int16_t CAN_45_GetMotorCurrent(uint8_t *rxData);
static int16_t CAN_23_GetMotorTorque(uint8_t *rxData);
static int16_t CAN_45_GetMotorTorque(uint8_t *rxData);
static uint8_t CAN_6_GetMotorTemperature(uint8_t *rxData);
static void Torque_to_Raw_Current(Motor_RM_t *motor);
static void Angle_Sum_Cal(Motor_RM_t *motor);
static void Encoder_to_Motor_Angle(Motor_RM_t *motor);
static float RPM_to_Rads(Motor_RM_t *motor);
static void Raw_Current_to_Torque(Motor_RM_t* motor);
static void Encoder_Sum_Cal(Motor_RM_t *motor);
/*..........................................�����..........................................*/
/**
  * @brief          ������������ת��,����CAN���Ͳ���
  * @param[in]      Motor_RM_t *motor     �������
  * @retval         none
  */
static void Motor_Set_Torque(Motor_RM_t *motor)
{
	uint8_t Id = motor->born_info->rxId*2;
	Torque_to_Raw_Current(motor);
	motor->tx_info->tx_buff[Id] = (uint8_t)(motor->tx_info->torque_current_raw >> 8);
	motor->tx_info->tx_buff[Id+1] = (uint8_t)(motor->tx_info->torque_current_raw);
	CAN_SendData(motor->born_info->hcan, motor->born_info->stdId, motor->tx_info->tx_buff);
}

/**
  * @brief          ���������,����CAN���Ͳ���
  * @param[in]      Motor_RM_t *motor     �������
  * @retval         none
  */
static void Motor_Ctrl(Motor_RM_t *motor)
{
	uint8_t Id = motor->born_info->rxId*2;
	motor->tx_info->tx_buff[Id] = (uint8_t)(motor->tx_info->torque_current_raw >> 8);
	motor->tx_info->tx_buff[Id+1] = (uint8_t)(motor->tx_info->torque_current_raw);
	CAN_SendData(motor->born_info->hcan, motor->born_info->stdId, motor->tx_info->tx_buff);
}

/**
  * @brief          �����ж��
  * @param[in]      Motor_RM_t *motor     �������
  * @retval         none
  */
static void Single_Motor_Sleep(Motor_RM_t *motor)
{
	motor->tx_info->torque = 0;
	motor->single_set_torque(motor);
}

/**
  * @brief          ����������ٶ�(�������ͺ�������Ҫ�ٿ���Ť�زſ��Կ���)
  * @param[in]      Motor_RM_t *motor     
  * @retval         none
  */
static void Motor_Set_Speed(Motor_RM_t *motor)
{
	pid_ctrl_t* my_speed_ctrl = motor->ctrl->speed_ctrl;
	my_speed_ctrl->measure = motor->rx_info->encoder_speed;
	my_speed_ctrl->err=my_speed_ctrl->target-my_speed_ctrl->measure;
	single_pid_ctrl(my_speed_ctrl);
	motor->tx_info->torque = my_speed_ctrl->out;
}


/**
  * @brief          ��������ƽǶ�(�������ͺ�������Ҫ�ٿ���Ť�زſ��Կ���)
  * @param[in]      Motor_RM_t *motor     
  * @retval         none
  */
static void Motor_Set_Angle(Motor_RM_t *motor)
{
	pid_ctrl_t* my_angle_ctrl = motor->ctrl->angle_ctrl_outer;
	pid_ctrl_t* my_speed_ctrl = motor->ctrl->angle_ctrl_inner;
	/*�⻷����*/
	if(motor->ctrl->Angle_Input_Flag == false)
	{
	my_angle_ctrl->measure = motor->rx_info->encoder;
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


/**
 * @brief  �������ʧ�����
 * @param  motor: ����ṹ��
 * @retval ��
 */
static void rm_motor_heart_beat(Motor_RM_t *motor)
{
    Motor_RM_State_t *motor_state = motor->state;
    motor_state->offline_cnt++;
    if(motor_state->offline_cnt > motor_state->offline_cnt_max) 
	{
        motor_state->offline_cnt = motor_state->offline_cnt_max;
        motor_state->status = DEV_OFFLINE;
    }
    else 
	{
        if(motor_state->status == DEV_OFFLINE)
            motor_state->status = DEV_ONLINE;
    }
}

/**
 *	@brief	����RM��׼����ĽǶȡ��ٶȡ�ת�ص������¶�
 */
static void rm_motor_update(Motor_RM_t *rm_motor, uint8_t *rxBuf)
{
    Motor_RM_Rx_Info_t *motor_info = rm_motor->rx_info;
    
    motor_info->encoder = CAN_01_GetMotorAngle(rxBuf);
		Encoder_Sum_Cal(rm_motor);
		Encoder_to_Motor_Angle(rm_motor);
		motor_info->encoder_speed = CAN_23_GetMotorSpeed(rxBuf);
		motor_info->speed = RPM_to_Rads(rm_motor);
		motor_info->torque_current_raw = CAN_45_GetMotorCurrent(rxBuf);
		Raw_Current_to_Torque(rm_motor);
    motor_info->temperature = CAN_6_GetMotorTemperature(rxBuf);
    rm_motor->state->offline_cnt = 0;
}

/**
 * @brief  �����ʼ��
 * @param  motor: ����ṹ��
 * @retval ��
 */
void RM_Motor_Init(Motor_RM_t *motor)
{
	motor->single_set_torque = Motor_Set_Torque;
	motor->single_heart_beat = rm_motor_heart_beat;
	motor->single_sleep = Single_Motor_Sleep;
	motor->single_set_speed = Motor_Set_Speed;
	motor->single_set_angle = Motor_Set_Angle;
	motor->single_ctrl = Motor_Ctrl;
	motor->rx = rm_motor_update;
	motor->state->offline_cnt_max = 100;
}

/*..........................................����..........................................*/
/**
  * @brief          ������1~4��������������ת��(��ת��),4���������Ϊͬһ���ͣ�����CAN���Ͳ���
  * @param[in]      Motor_RM_Group_t *group     �����
  * @retval         none
  */
static void Group_Motor_Set_Torque(Motor_RM_Group_t *group)
{	
		int16_t torque_current = 0;
		uint8_t Id;
		for(uint8_t i = 0; i < 4; i ++)
		{
			if(group->motor[i] != NULL)
			{
				Id = group->motor[i]->born_info->rxId*2;
				Torque_to_Raw_Current(group->motor[i]);
				torque_current = group->motor[i]->tx_info->torque_current_raw;
				group->tx_buff[Id] = (uint8_t)(torque_current >> 8);
				group->tx_buff[Id+1] = (uint8_t)(torque_current);
//				group->motor[i]->tx_info->torque = 0;
			}
		}
		
		CAN_SendData(group->hcan, group->stdId, group->tx_buff);
		
		memset(group->tx_buff, 0, 8);
}

/**
  * @brief          ������1~4��������������,4���������Ϊͬһ���ͣ�����CAN���Ͳ���
  * @param[in]      Motor_RM_Group_t *group     �����
  * @retval         none
  */
static void Group_Motor_Ctrl(Motor_RM_Group_t *group)
{	
		int16_t torque_current = 0;
		uint8_t Id;
		for(uint8_t i = 0; i < 4; i ++)
		{
			if(group->motor[i] != NULL)
			{
				Id = group->motor[i]->born_info->rxId*2;
				torque_current = group->motor[i]->tx_info->torque_current_raw;
				group->tx_buff[Id] = (uint8_t)(torque_current >> 8);
				group->tx_buff[Id+1] = (uint8_t)(torque_current);
//				group->motor[i]->tx_info->torque = 0;
			}
		}
		
		CAN_SendData(group->hcan, group->stdId, group->tx_buff);
		
		memset(group->tx_buff, 0, 8);
}

/**
  * @brief          �����ж��
  * @param[in]      Motor_RM_Group_t *group     �����
  * @retval         none
  */
static void Group_Motor_Sleep(Motor_RM_Group_t *group)
{		
		
	for(uint8_t i = 0; i < 4; i ++)
		{
			if(group->motor[i] != NULL)
			{
				group->motor[i]->tx_info->torque = 0;
			}
		}
		
}


/**
  * @brief          �����������
  * @param[in]      Motor_RM_Group_t *group     �����
  * @retval         none
  */
static void Group_Motor_Heartbeat(Motor_RM_Group_t *group)
{
	for(uint8_t i = 0; i < 4; i ++)
	{
		if(group->motor[i] != NULL)
		{
			group->motor[i]->single_heart_beat(group->motor[i]);
		}
	}
}

/**
  * @brief          ������ʼ��
  * @param[in]      Motor_Ktech_Group_t *group     �����
  * @retval         none
  */
void RM_Group_Motor_Init(Motor_RM_Group_t *group)
{
		for(uint8_t i = 0; i < 4; i ++)
		{
			if(group->motor[i] != NULL)
			{
				group->motor[i]->single_init = RM_Motor_Init;
				group->motor[i]->single_init(group->motor[i]);
			}
		}
	
	  group->group_set_torque = Group_Motor_Set_Torque;
		group->group_heartbeat = Group_Motor_Heartbeat;
	  group->group_sleep = Group_Motor_Sleep;
		group->group_ctrl = Group_Motor_Ctrl;
}

/*..........................................���ߺ���..........................................*/
/**
 *	@brief	��CAN����[0][1]�ж�ȡ�����λ�÷���
 */
static uint16_t CAN_01_GetMotorAngle(uint8_t *rxData)
{
	uint16_t angle;
	angle = (uint16_t)(rxData[0] << 8| rxData[1]);
	return angle;
}

/**
 *	@brief	��CAN����[2][3]�ж�ȡ�����ת��ת�ٷ���
 */
static int16_t CAN_23_GetMotorSpeed(uint8_t *rxData)
{
	int16_t speed;
	speed = (int16_t)(rxData[2] << 8| rxData[3]);
	return speed;
}

/**
 *	@brief	��CAN����[4][5]�ж�ȡ�����ʵ��ת�ص�������
 */
static int16_t CAN_45_GetMotorCurrent(uint8_t *rxData)
{
	int16_t current;
	current = (int16_t)(rxData[4] << 8 | rxData[5]);
	return current;
}

/**
 *	@brief	��CAN����[2][3]�ж�ȡ�����ʵ�����ת��
 */
static int16_t CAN_23_GetMotorTorque(uint8_t *rxData)
{
	int16_t torque;
	torque = ((uint16_t)rxData[2] << 8 | rxData[3]);
	return torque;
}

/**
 *	@brief	��CAN����[4][5]�ж�ȡ�����ʵ�����ת��
 */
static int16_t CAN_45_GetMotorTorque(uint8_t *rxData)
{
	int16_t torque;
	torque = ((uint16_t)rxData[4] << 8 | rxData[5]);
	return torque;
}

/**
 *	@brief	��CAN����[6]�ж�ȡ�����ʵ���¶�
 */
static uint8_t CAN_6_GetMotorTemperature(uint8_t *rxData)
{
	uint8_t temp;
	temp = rxData[6];
	return temp;
}

/**
  * @brief          ������������Ť��תΪԭʼ��������,���ڷ������ݵĴ���
  * @param[in]      Motor_RM_t *motor     �������
  * @retval         none
  */
static void Torque_to_Raw_Current(Motor_RM_t *motor)
{
		switch(motor->born_info->type)
		{
			//��3508���û���ȶ���ת�س�����ʵ�����Ť�ز�����motor->tx_info->torque
			case _3508_Single:
			motor->tx_info->torque_current = motor->tx_info->torque;
			motor->tx_info->torque_current = constrain(motor->tx_info->torque_current, -16384, 16384);//�������޷�
			/*��3508ת�ص���ת��Ϊ������ֵ*/
			motor->tx_info->torque_current_raw = (int16_t)((motor->tx_info->torque_current));
			break;
			case _3508_Reduction:
			motor->tx_info->torque_current = motor->tx_info->torque / _3508_TORQUE_CONSTANT;
			motor->tx_info->torque_current = constrain(motor->tx_info->torque_current, -_3508_MAX_CURRENT*0.9f, _3508_MAX_CURRENT*0.9f);//�������޷�
			/*3508������ת�ص���ת��Ϊ������ֵ*/
			motor->tx_info->torque_current_raw = (int16_t)((motor->tx_info->torque_current / _3508_MAX_CURRENT) * 16384.f);
			break;
			case _2006_Single:
			motor->tx_info->torque_current = motor->tx_info->torque;
			motor->tx_info->torque_current = constrain(motor->tx_info->torque_current, -10000, 10000);//�������޷�
			/*2006ת�ص���ת��Ϊ������ֵ*/
			motor->tx_info->torque_current_raw = (int16_t)((motor->tx_info->torque_current));
			break;
			case _6020_Single:
			motor->tx_info->torque_current = motor->tx_info->torque / _6020_TORQUE_CONSTANT;
			motor->tx_info->torque_current = constrain(motor->tx_info->torque_current, -_6020_MAX_CURRENT, _6020_MAX_CURRENT);//�������޷�
			/*6020ת�ص���ת��Ϊ������ֵ*/
			motor->tx_info->torque_current_raw = (int16_t)((motor->tx_info->torque_current));
			break;
		}
		

}


/**
 *	@brief	У��RM��׼���������(��Ϊ����ת���ĽǶ�)
 */
static void Encoder_Sum_Cal(Motor_RM_t *motor)
{
	int16_t err;
	Motor_RM_Rx_Info_t *motor_info = motor->rx_info;
	
	/* δ��ʼ�� */
	if(motor_info->motor_angle_last == 0 && motor_info->encoder_sum == 0)
	{
		err = 0;
	}
	else
	{
		err = motor_info->encoder - motor_info->encoder_last;
	}
	
	/* ����� */
	if(abs(err) > 4095)
	{
		/* 0�� -> 8191 */
		if(err >= 0)
			motor_info->encoder_sum += -8191 + err;
		/* 8191�� -> 0 */
		else
			motor_info->encoder_sum += 8191 + err;
	}
	/* δ����� */
	else
	{
		motor_info->encoder_sum += err;
	}
	
	motor_info->encoder_last = motor_info->encoder;
}


/**
  * @brief          ��������ֵת��Ϊ�����ƣ����������ǶȺ�,��9025��8016
  * @param[in]      Motor_RM_t *motor     �������
  * @retval         none
  */
static void Encoder_to_Motor_Angle(Motor_RM_t *motor)
{
	if(motor->born_info->type == _3508_Reduction)
	motor->rx_info->motor_angle = ((float)motor->rx_info->encoder / 8191.f) * (float)PI * 2.f / _3508_REDUCT_RATIO;
	else if(motor->born_info->type == _2006_Single)
	motor->rx_info->motor_angle = ((float)motor->rx_info->encoder / 8191.f) * (float)PI * 2.f / _2006_REDUCT_RATIO;
	else
	motor->rx_info->motor_angle = ((float)motor->rx_info->encoder / 8191.f) * (float)PI * 2.f;
	
	Angle_Sum_Cal(motor);
}

/**
  * @brief          ��������ת�ǶȺ�
  * @param[in]      Motor_RM_t *motor     �������
  * @retval         none
  */
static void Angle_Sum_Cal(Motor_RM_t *motor)
{
	float err = 0.f;
	
	float order_correction = 0.f;
	
	if(motor->born_info->order_correction == 1 || motor->born_info->order_correction == -1)
	{
		order_correction = (float)motor->born_info->order_correction;
	}
	else
	{
		order_correction = 1.f;
	}
	
	if(!motor->rx_info->motor_angle_last && !motor->rx_info->motor_angle_sum)//��һ�Ƕ�ֵΪ0�ҽǶȺ�Ϊ��ʱ����������������������
	{
		err = 0.f;
	}
	else
	{
		err = motor->rx_info->motor_angle - motor->rx_info->motor_angle_last;
	}
	
	if((abs(err) > ((float)PI)) || (abs(err) > ((float)PI/_3508_REDUCT_RATIO) && motor->born_info->type == _3508_Reduction))//�����
	{
		if(err > 0.f)
		{
			if(motor->born_info->type == _3508_Reduction)
			motor->rx_info->motor_angle_sum += (-(float)PI * 2.f / _3508_REDUCT_RATIO + err) * order_correction;
			else
			motor->rx_info->motor_angle_sum += (-(float)PI * 2.f + err) * order_correction;
		}
		else
		{
			if(motor->born_info->type == _3508_Reduction)
			motor->rx_info->motor_angle_sum += ((float)PI * 2.f / _3508_REDUCT_RATIO + err) * order_correction;
			else
			motor->rx_info->motor_angle_sum += ((float)PI * 2.f + err) * order_correction;
		}
	}
	else
	{
		motor->rx_info->motor_angle_sum += err * order_correction;
	}
	
	motor->rx_info->motor_angle_last = motor->rx_info->motor_angle;
}

/**
  * @brief          ת�������ת�ٶ�Ϊrad/s
  * @param[in]      int16_t rpm     r/min
  * @retval         rad/s
  */
static float RPM_to_Rads(Motor_RM_t *motor)
{
	float ret;
	if(motor->born_info->type == _3508_Reduction)
	ret= motor->rx_info->encoder_speed/60.f*2*PI/_3508_REDUCT_RATIO;
	else if(motor->born_info->type == _2006_Single)
	ret= motor->rx_info->encoder_speed/60.f*2*PI/_2006_REDUCT_RATIO;
	else
	ret= motor->rx_info->encoder_speed/60.f*2*PI;
	return ret;
}


/**
  * @brief          ���������ԭʼ��������תΪʵ��ת��,���ڽ������ݵĴ���
  * @param[in]      Motor_Ktech_t *motor     �������
  * @retval         none(Ŀǰδ����)
  */
static void Raw_Current_to_Torque(Motor_RM_t* motor)
{
		motor->rx_info->torque_current = (motor->rx_info->torque_current_raw / 16384.f)*20.f;
		motor->rx_info->torque = motor->rx_info->torque_current * _3508_TORQUE_CONSTANT;
}
