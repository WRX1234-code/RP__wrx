/**
 * @file        Ht_Motor.c
 * @author      2025_YZJ
 * @Version     V1.0
 * @date        8-Febraruary-2025
 * @brief       ��̩�����(�����ͺ�HT-03��
 * @brief       ��δ���ƣ�����ֻ��Ϊ��챵��ʹ�ã�ֻд�˲��ֹ���
 */
 
/* Includes ------------------------------------------------------------------*/
#include "HT_Motor.h"

static uint8_t Motor_Command[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
static void Motor_Send_Data(Motor_HT_t *motor, uint8_t* buf);
static void Motor_Send_Command(Motor_HT_t *motor, Motor_MIT_Command_e Command);
static void HT_Motor_SetControlPara(Motor_HT_t *motor);
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits);
static void Encoder_to_Motor_Angle(Motor_HT_t *motor);
uint8_t flag_rx;
/**
  * @brief          �����ж��
  * @param[in]      Motor_HT_t *motor     �������
  * @retval         none
  */
void HT_Single_Motor_Sleep(Motor_HT_t *motor)
{
	if(motor != NULL)
	{
//		if(motor->state->mode == Motor_UnControl)
//		{
//			Motor_Send_Command(motor, Enter_Motor_Mode);//�����������
//			motor->state->mode = Motor_Control;
//		}
		
		Motor_HT_Tx_Info_t* motor_tx_info = motor->tx_info;
		motor_tx_info->torque = 0;
		motor_tx_info->Kd = 0;
		motor_tx_info->Kp = 0;
	}
}

/**
  * @brief          ��������������
  * @param[in]      Motor_HT_t *motor     �������
  * @retval         none
  */
void HT_Single_Motor_ZeroPosSensor(Motor_HT_t *motor)
{
	if(motor != NULL)
	{
		motor->single_sleep(motor);//�ȶԵ��ж��
		
		Motor_Send_Command(motor, Zero_Position_Sensor);
	}
}

/**
  * @brief          ������������ת��,����CAN���Ͳ���
  * @param[in]      Motor_HT_t *motor     �������
  * @retval         none
  */
void HT_Single_Motor_Set_Torque(Motor_HT_t *motor)
{
	  if(motor != NULL)
		{
			if(motor->state->mode == Motor_UnControl)
			{
				Motor_Send_Command(motor, Enter_Motor_Mode);//�����������
				motor->state->mode = Motor_Control;
			}
			else
			{
				Motor_HT_Tx_Info_t* motor_tx_info = motor->tx_info;
				motor_tx_info->target_angle = 0;
				motor_tx_info->target_speed = 0;
				motor_tx_info->Kp = 0;
				motor_tx_info->Kd = 0;
				HT_Motor_SetControlPara(motor);
				Motor_Send_Data(motor, motor_tx_info->single_tx_buff);
			}
		}
}

/**
  * @brief          ����������ٶ�,��Ҫ��������kd\target_speed\torque,����CAN���Ͳ���
  * @param[in]      Motor_HT_t *motor     �������
  * @retval         none
  */
void HT_Single_Motor_Set_Speed(Motor_HT_t *motor)
{
	  if(motor != NULL)
		{
			if(motor->state->mode == Motor_UnControl)
			{
				Motor_Send_Command(motor, Enter_Motor_Mode);//�����������
				motor->state->mode = Motor_Control;
			}
			
			Motor_HT_Tx_Info_t* motor_tx_info = motor->tx_info;
			motor_tx_info->target_angle = 0;
			motor_tx_info->Kp = 0;
			HT_Motor_SetControlPara(motor);
		
			Motor_Send_Data(motor, motor_tx_info->single_tx_buff);
		}
}

/**
  * @brief          ��������ƽǶ�,��Ҫ��������kp\target_angle\kd\target_speed\torque,����CAN���Ͳ���
  * @param[in]      Motor_HT_t *motor     �������
  * @retval         none
  */
void HT_Single_Motor_Set_Angle(Motor_HT_t *motor)
{
	  if(motor != NULL)
		{
			if(motor->state->mode == Motor_UnControl)
			{
				Motor_Send_Command(motor, Enter_Motor_Mode);//�����������
				motor->state->mode = Motor_Control;
			}
			
			Motor_HT_Tx_Info_t* motor_tx_info = motor->tx_info;
			HT_Motor_SetControlPara(motor);
			Motor_Send_Data(motor, motor_tx_info->single_tx_buff);
		}
}

/**
  * @brief          ���CAN�жϽ������ݴ���
  * @param[in]      Motor_HT_t *motor      �������
  * @param[in]      uint8_t *rxBuf						CAN�������ݰ�
  * @retval         none
  */
void static Motor_ReceiveData(Motor_HT_t *motor, uint8_t *rxBuf)
{
	Motor_HT_Rx_Info_t* motor_rx_info = motor->rx_info;
	motor_rx_info->encoder = uint_to_float((uint16_t)((rxBuf[1] << 8) | rxBuf[2]), HT_P_MIN, HT_P_MAX, 16);
	motor_rx_info->speed = uint_to_float((uint16_t)((rxBuf[3] << 4) | (rxBuf[4] >> 4)), HT_V_MIN, HT_V_MAX, 12) * motor->born_info->order_correction;
	motor_rx_info->torque_current = uint_to_float((uint16_t)(((rxBuf[4]&0x0F) << 8) | rxBuf[5]), HT_C_MIN, HT_C_MAX, 12);
	motor_rx_info->torque = motor_rx_info->torque_current * HT_TORQUE_CONSTANT;
	Encoder_to_Motor_Angle(motor);
	motor_rx_info->motor_angle_sum_vi += motor_rx_info->speed * TIME_STEP;
	motor->state->offline_cnt = 0;
}

/**
  * @brief          �����������
  * @param[in]      Motor_HT_t *motor    �������
  * @retval         none
  */
void HT_Motor_Hearbeat(Motor_HT_t *motor)
{
	motor->state->offline_cnt++;
	
	if(motor->state->offline_cnt > motor->state->offline_cnt_max) 
	{
//		motor->state->offline_cnt = motor->state->offline_cnt_max;
		motor->state->status = DEV_OFFLINE;
		motor->state->mode = Motor_UnControl;
	}
	else 
	{
		if(motor->state->status == DEV_OFFLINE)
			motor->state->status = DEV_ONLINE;
	}
}

/**
  * @brief          �������ʼ��
  * @param[in]      Motor_HT_t *motor     �������
  * @retval         none
  */
void HT_Single_Motor_Init(Motor_HT_t *motor)
{
	motor->state->mode = Motor_UnControl;
	motor->single_sleep = HT_Single_Motor_Sleep;
	motor->zero_position = HT_Single_Motor_ZeroPosSensor;
	motor->single_set_torque = HT_Single_Motor_Set_Torque;
	motor->single_set_speed  = HT_Single_Motor_Set_Speed;
	motor->single_set_angle  = HT_Single_Motor_Set_Angle;
	motor->rx = Motor_ReceiveData;
	motor->single_heart_beat = HT_Motor_Hearbeat;
	/*�����������*/
	motor->single_sleep(motor);
	motor->state->offline_cnt_max = 500;
	motor->rx_info->motor_angle_sum = 0;
}

/*..........................................����..........................................*/

/**
  * @brief          ������1~4��������������ת��,4���������Ϊͬһ���ͣ�����CAN���Ͳ���
  * @param[in]      Motor_HT_Group_t *group     �����
  * @retval         none
  */
static void Group_Motor_Set_Torque(Motor_HT_Group_t *group)
{	
	
		if(group->motor[0] != NULL)
		{
			group->motor[0]->single_set_angle(group->motor[0]);
		}
		if(group->motor[1] != NULL)
		{
			group->motor[1]->single_set_angle(group->motor[1]);
		}
		if(group->motor[2] != NULL)
		{
			group->motor[2]->single_set_angle(group->motor[2]);
		}
		if(group->motor[3] != NULL)
		{
			group->motor[3]->single_set_angle(group->motor[3]);
		}
}

/**
  * @brief          ������1~4�����������ж����������CAN���Ͳ���
  * @param[in]      Motor_HT_Group_t *group     �����
  * @retval         none
  */
static void Group_Motor_Sleep(Motor_HT_Group_t *group)
{	
		if(group->motor[0] != NULL)
		{
			group->motor[0]->single_sleep(group->motor[0]);
		}
		if(group->motor[1] != NULL)
		{
			group->motor[1]->single_sleep(group->motor[1]);
		}
		if(group->motor[2] != NULL)
		{
			group->motor[2]->single_sleep(group->motor[2]);
		}
		if(group->motor[3] != NULL)
		{
			group->motor[3]->single_sleep(group->motor[3]);
		}
}

/**
  * @brief          �����������
  * @param[in]      Motor_HT_Group_t *group     �����
  * @retval         none
  */
static void Group_Motor_Heartbeat(Motor_HT_Group_t *group)
{
	if(group->motor[0] != NULL)
	{
		group->motor[0]->single_heart_beat(group->motor[0]);
	}
	  
	if(group->motor[1] != NULL)
	{
		group->motor[1]->single_heart_beat(group->motor[1]);
	}

  if(group->motor[2] != NULL)
	{
		group->motor[2]->single_heart_beat(group->motor[2]);
	}
	  
	if(group->motor[3] != NULL)
	{
		group->motor[3]->single_heart_beat(group->motor[3]);
	}
}


/**
  * @brief          ������ʼ��
  * @param[in]      Motor_HT_Group_t *group     �����
  * @retval         none
  */
void HT_Group_Motor_Init(Motor_HT_Group_t *group)
{
	  if(group->motor[0] != NULL)
		{
			group->motor[0]->single_init = HT_Single_Motor_Init;
	    group->motor[0]->single_init(group->motor[0]);
		}
	  
		if(group->motor[1] != NULL)
		{
			group->motor[1]->single_init = HT_Single_Motor_Init;
	    group->motor[1]->single_init(group->motor[1]);
		}

    if(group->motor[2] != NULL)
		{
			group->motor[2]->single_init = HT_Single_Motor_Init;
	    group->motor[2]->single_init(group->motor[2]);
		}
	  
		if(group->motor[3] != NULL)
		{
			group->motor[3]->single_init = HT_Single_Motor_Init;
	    group->motor[3]->single_init(group->motor[3]);
		}
	  
	
	  group->group_set_torque = Group_Motor_Set_Torque;
		group->group_heartbeat = Group_Motor_Heartbeat;
	  group->group_sleep = Group_Motor_Sleep;
}

/*..........................................���ߺ���..........................................*/
/**
  * @brief          ���ϲ����͵�������
  * @param          Motor_HT_t *motor
  * @param[in]      Motor_HT_Command_e Command
  * @retval         none
  */
static void Motor_Send_Command(Motor_HT_t *motor, Motor_MIT_Command_e Command)
{
	switch(Command)
	{
		case Enter_Motor_Mode:
		Motor_Command[7] = 0xFC;
		break;
		case Exit_Motor_Mode:
		Motor_Command[7] = 0xFD;
		break;
		case Zero_Position_Sensor:
		Motor_Command[7] = 0xFE;
		break;
		default:
		break;
	}
	Motor_Send_Data(motor, Motor_Command);
}

/**
  * @brief          ���ݽṹ����Ϣ���ͱ���
  * @param          Motor_HT_t *motor
  * @param          uint8_t* buf Ҫ���͵ı�����Ϣ
  * @retval         none
  */
static void Motor_Send_Data(Motor_HT_t *motor, uint8_t* buf)
{
	Motor_HT_Born_Info_t* motor_born_info = motor->born_info;
	
	CAN_SendData(motor_born_info->hcan, motor_born_info->stdId, buf);
}

/**
  * @brief          ���ݷ��͵ı�����Ϣ���ñ���
  * @param          Motor_HT_t *motor
  * @retval         none
  */
static void HT_Motor_SetControlPara(Motor_HT_t *motor)
{
	Motor_HT_Tx_Info_t* motor_tx_info = motor->tx_info;
	uint16_t p, v, kp, kd, t;
  uint8_t* buf = motor_tx_info->single_tx_buff;
	
	/* ��������Ĳ����ڶ���ķ�Χ�� */
	motor_tx_info->target_angle = constrain(motor_tx_info->target_angle, HT_P_MIN, HT_P_MAX);
	motor_tx_info->target_speed = constrain(motor_tx_info->target_speed, HT_V_MIN, HT_V_MAX);
	motor_tx_info->Kp = constrain(motor_tx_info->Kp, HT_KP_MIN, HT_KP_MAX);
	motor_tx_info->Kd = constrain(motor_tx_info->Kd, HT_KD_MIN, HT_KD_MAX);
	motor_tx_info->torque = constrain(motor_tx_info->torque, -8, 8);
	
	/* ����Э�飬��float��������ת�� */
	p = float_to_uint(motor_tx_info->target_angle,      HT_P_MIN,  HT_P_MAX,  16);            
	v = float_to_uint(motor_tx_info->target_speed,      HT_V_MIN,  HT_V_MAX,  12);
	kp = float_to_uint(motor_tx_info->Kp,    HT_KP_MIN, HT_KP_MAX, 12);
	kd = float_to_uint(motor_tx_info->Kd,    HT_KD_MIN, HT_KD_MAX, 12);
	t = float_to_uint(motor_tx_info->torque,      HT_T_MIN,  HT_T_MAX,  12);
	
	/* ���ݴ���Э�飬������ת��ΪCAN���������ֶ� */
	buf[0] = p>>8;
	buf[1] = p&0xFF;
	buf[2] = v>>4;
	buf[3] = ((v&0xF)<<4)|(kp>>8);
	buf[4] = kp&0xFF;
	buf[5] = kd>>4;
	buf[6] = ((kd&0xF)<<4)|(t>>8);
	buf[7] = t&0xff;
}

/**
  * @brief  ��floatתΪuint����������������,��ͨ��Э�鱣��һ��
  * @param
  * @retval 
  */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief  ��uintתΪfloat����������������
  * @param
  * @retval 
  */
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


/**
  * @brief          ��������ֵת��Ϊ������(0~2PI)
  * @param[in]      Motor_Ktech_t *motor     �������
  * @retval         none
  */
static void Encoder_to_Motor_Angle(Motor_HT_t *motor)
{
	float order_correction = 0.f;
	
	if(motor->born_info->order_correction == 1 || motor->born_info->order_correction == -1)
	{
		order_correction = (float)motor->born_info->order_correction;
	}
	else
	{
		order_correction = 1.f;
	}
	
	//������������ж�
	if(!motor->rx_info->encoder_last && !motor->rx_info->motor_angle_sum)//��һ�Ƕ�ֵΪ0�ҽǶȺ�Ϊ��ʱ����������������������
	{
		motor->rx_info->encoder_err = 0.f;
	}
	else
	{
		motor->rx_info->encoder_err = -motor->rx_info->encoder_last + motor->rx_info->encoder;
	}
	 
	if(motor->rx_info->encoder_err > 180.f)
	{
		motor->rx_info->encoder_err -= 2*HT_P_MAX;
	}
	else if(motor->rx_info->encoder_err < -180.f)
	{
		motor->rx_info->encoder_err += 2*HT_P_MAX;
	}
	
	motor->rx_info->motor_angle_sum += motor->rx_info->encoder_err * order_correction;
	motor->rx_info->motor_angle = motor->rx_info->motor_angle_sum - (int16_t)(motor->rx_info->motor_angle_sum / (2.f*PI)) * 2.f * PI;
	
	motor->rx_info->encoder_last = motor->rx_info->encoder;
	
}

/*ʾ������*/
/*-------------�����������-------------*/
/*

Motor_HT_Born_Info_t L_Wheel_Born_Info = 
{	
	.stdId = 0x009,//������Ʊ���ID
	
	.hcan = &hcan2,//ʹ�õ�Can����
	
	.order_correction = 0,//����ܽǶȵ�������Ϊ˳ʱ��
	
};

Motor_HT_Rx_Info_t L_Wheel_Rx_Info_t;

Motor_HT_Tx_Info_t L_Wheel_Tx_Info_t;

Motor_HT_State_t L_Wheel_State_t;

Motor_HT_t L_Wheel = 
{
	.born_info = &L_Wheel_Born_Info,
	
	.rx_info = &L_Wheel_Rx_Info_t,
	
	.tx_info = &L_Wheel_Tx_Info_t,
	
	.state = &L_Wheel_State_t,
	
	.single_init = &HT_Single_Motor_Init,
*/

/*-------------��ʼ��-------------*/
/*
L_Wheel.single_init(&L_Wheel);
*/

/*-------------���պ���-------------*/
/*
void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case 0x000://����ID
		L_Wheel.rx(&L_Wheel, rxBuf);
		break;
		default:
			break;
	}
}
*/

/*-------------����ִ��-------------*/
/*
  * @file    monitor_task.c
  * @brief   �������
  *          1. ��ģ������ʧ�����
  *          2. ���ң����״̬�������λ
void StartMonitorTask(void const * argument)//
{
	
	for(;;)
	{
		L_Wheel.heartbeat(&L_Wheel);
		
		osDelay(1);
	}
}

  * @file    monitor_task.c
  * @brief   �����������
  *          1. ��������Ϳ��Ʊ���
  *          2. ��״̬��־λ������Ӧ
void StartMonitorTask(void const * argument)//
{
	
	for(;;)
	{
		//���Ϳ��Ʊ��ģ����Ƶ�����Ť��Ϊ0.5N*m
		L_Wheel.tx_info->torque = 0.5f;
		L_Wheel.single_set_torque(&L_Wheel);
		
		osDelay(1);
	}
}

*/
