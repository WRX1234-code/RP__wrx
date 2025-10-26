/**
 * @file        rp_math.c
 * @author      RobotPilots
 * @Version     v1.1
 * @brief       RobotPilots Robots' Math Libaray.
 * @update
 *              v1.0(11-September-2020)
 *              v1.1(13-November-2021)
 *                  1.����λ��������
 */

/* Includes ------------------------------------------------------------------*/
#include "rp_math.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 * @brief  ��ͨ�˲�,K��(0,1)��K Խ���˲�Ч��Խ��
 * @param  �ϴε��˲����X_last  ���µ�����X_new ���˲�ϵ��K
 * @return  �˲�����ֵ
 */
float Lowpass(float X_last, float X_new, float K)
{
	return (X_last + (X_new - X_last) * K);
}

/**
 *	@brief	����Ȧ���� angle��Դ���� cycle:���ݷ�Χ
 */
float motor_half_cycle(float angle, float max)
{
	if (abs(angle) > (max / 2))
	{
		if (angle >= 0)
			angle += -max;
		else
			angle += max;
	}
	return angle;
}

int16_t RampInt(int16_t final, int16_t now, int16_t ramp)
{
	int32_t buffer = 0;

	buffer = final - now;
	if (buffer > 0)
	{
		if (buffer > ramp)
			now += ramp;
		else
			now += buffer;
	}
	else
	{
		if (buffer < -ramp)
			now += -ramp;
		else
			now += buffer;
	}

	return now;
}

float RampFloat(float final, float now, float ramp)
{
	float buffer = 0;

	buffer = final - now;
	if (buffer > 0)
	{
		if (buffer > ramp)
			now += ramp;
		else
			now += buffer;
	}
	else
	{
		if (buffer < -ramp)
			now += -ramp;
		else
			now += buffer;
	}

	return now;
}




float DeathZoom(float input, float center, float death)
{
	if (abs(input - center) < death)
		return center;
	return input;
}
/**
  * @name   Time_Trigger_inloop
  * @brief  ��ѭ���ﶨʱ����
  * @note   ����Ҫ�ⲿ���������ʱ�䡢��һ�β�ֱ�Ӵ�����־λ,��ֹ�ദ���ù���ʱ����־λ���´���
  *         ��Ҫô�Լ��ⲿ���־λ��Ҫôִ�е�����Ҫ��������������棬��Ȼ���״������ʱ��
  *         ��*ignore_first_trigger_flag��Ҫ��ʼΪ0
			 ��*ignore_first_trigger_flag �˳�������Ҫ����
  * @param  private_flag: ��־λָ�룬����ָʾ�Ƿ񴥷���1Ϊ������
  * @param  last_trigger_tick: ���ڴ洢�ϴδ�����ʱ�䣨�ⲿ���������ʼ��Ϊ0��
  * @param  ignore_first_trigger_flag: �Ƿ���Ե�һ�δ����ı�־λ���ⲿ���������ʼ��Ϊ0,�˳�������Ҫ���㣩
  * @param  delay_tick: ������ʱ��������λ�����룩
  * @param  if_ignore_first: �Ƿ���Ե�һ�δ�����1Ϊ���ԣ�0Ϊ�����ԣ�
  * @author HERMIT_PURPLE
  */
void Time_Trigger_inloop(Time_trigger_t *Time_trigger_struct)
{
	if (Time_trigger_struct->private_flag == NULL || Time_trigger_struct->last_trigger_tick == NULL || Time_trigger_struct->ignore_first_trigger_flag == NULL)
	{
		return;
	}

	// ����������������ֱ�Ӵ���һ��,���ȸ�ֵһ���ϴε�ʱ��
	if (Time_trigger_struct->if_ignore_first == 1 && Time_trigger_struct->ignore_first_trigger_flag == 0)
	{
		*Time_trigger_struct->ignore_first_trigger_flag = 1;
		*Time_trigger_struct->last_trigger_tick = HAL_GetTick();
	}

	if ((HAL_GetTick() - *Time_trigger_struct->last_trigger_tick > Time_trigger_struct->delay_tick) && (Time_trigger_struct->ignore_first_trigger_flag != 0 || Time_trigger_struct->if_ignore_first == 0))
	{
		*Time_trigger_struct->private_flag = 1;
		*Time_trigger_struct->last_trigger_tick = HAL_GetTick();
	}
	/*�ڲ����־λ*/
	else
	{
		*Time_trigger_struct->private_flag = Time_trigger_struct->flag_before_trigger;
	}
}
