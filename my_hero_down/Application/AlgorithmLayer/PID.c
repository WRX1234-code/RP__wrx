#include "pid.h"
#include "rp_math.h"
/**
 *  @name   single_pid_ctrl
 */
void single_pid_ctrl(pid_ctrl_t *pid)
{
    // �������ֵ(��Ҫ���������м������)
	//pid->err = pid->target-pid->measure;
	pid->integral += pid->err;  
    pid->integral = constrain(pid->integral, -pid->integral_max, +pid->integral_max);
    // p i d ��������
    pid->pout = pid->kp * pid->err;
    pid->iout = pid->ki * pid->integral;
	pid->last_dout=pid->dout;
    // �ۼ�pid���ֵ
    pid->out = pid->pout + pid->iout + pid->dout;
    pid->out = constrain(pid->out, -pid->out_max, pid->out_max);
    // ��¼�ϴ����ֵ
    pid->last_err = pid->err;
}


/**
 *	@brief	pid�ܿ��� �������⻷ �ڻ�  �⻷���ڻ�Ŀ��ֵ �⻷�۲�ֵ �ڻ��۲�ֵ  �ڻ��۲�ֵkp��һ����� err����ʽ
 *          err_cal_mode��err����ʽ ��Ȧ�����ķ�֮һȦ 0��1��2 �ٶȻ�ʹ��0 yaw��ʹ��1 
						�����ǽǶȻ� 3
 *         �ڻ�����ΪNULL
 *	@note   ʹ��ʾ����
			pid_ctrl_t *out	  = ;
			pid_ctrl_t *inn	  = ;
			float target   	  = ;
			float mea_out       = ;
			float mea_in        = ;
			float inner_kp      = ;
			uint8_t err_cal_mode= ;
			=all_pid_calc (out,inn,target,mea_out,mea_in,inner_kp,err_cal_mode);
 *  @author HERMIT_PURPLE
 *
 *  @return ���ؼ�����
 */

float  all_pid_calc (pid_ctrl_t *out,pid_ctrl_t *inn,float target,float mea_out,float mea_in,float inner_kp,uint8_t err_cal_mode)
{
	if(inn == NULL)return 0;  //û���ڻ���Ϊ0
	
	 else if(out == NULL&&inn!=NULL)  //ֻ���ٶȻ�
	{
		inn->target=target;
		inn->measure=mea_in;
		inn->err=inn->target-mea_in;
		single_pid_ctrl(inn);
		return inn->out;
	}
	
	else if(out != NULL&&inn!=NULL)  //˫��PID
	{
		
		out->target=target;
		out->measure=mea_out;
		out->err=out->target-out->measure; //����ǶȻ��������ٽ�������
		switch(err_cal_mode)
		{
			
			case 0:			
				break;
			
			case 1:
				out->err = motor_half_cycle(out->err, 8191);
				break;		
			
			case 2:
				out->err = motor_half_cycle(out->err, 8191);
				out->err = motor_half_cycle(out->err, 4095);
				break;
			
			case 3:
				out->err = motor_half_cycle(out->err, 360);
				break;
			
			case 4:
				out->err = motor_half_cycle(out->err, 65535);
				break;
			
			case 5:
				out->err = motor_half_cycle(out->err, 191);
				break;
			default:
				break;
		}
		
		single_pid_ctrl(out);  //�������������ĽǶȻ���ֵ
		inn->target=out->out; //�ǶȻ������Ϊ�ٶȻ�Ŀ��ֵ
		inn->measure=mea_in*inner_kp;//�ڻ�����kp�����Ե��������ʹ�С
		inn->err=inn->target+inn->measure;  //�ٶȻ�������
		single_pid_ctrl(inn);
		return inn->out;  //����ڻ�����ֵ
	}
	else  //ֻ�нǶȻ�
	{
		return 0;
	}
}
