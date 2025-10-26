#include "cap.h"
//#include "judge_protocol.h"
//#include "communicate_protocol.h"
/*
电容使用：
1、使用cap.setinfo函数进行赋值
2、用can将Buff0x2E和Buff0x2F两个包发送出去
其中一个是数据包一个是控制包，控制的频率自己斟酌
*/

static void CAP_setMessage(cap_t *self, uint16_t powerBuff, uint16_t powerLimit);
static void CAP_rxMessage(cap_t *self, uint32_t can_Id, uint8_t *rxBuf);
static void CAP_heart_beat(cap_t *self);

cap_t cap =
	{
		.info.can_rx_Id = MASTER_CAN_RX_ID,
		.info.can_tx_Id = CAPBOARD_CAN_TX_ID,
		.info.wireless_can_rx_Id = WIRELESS_CHARGE_CAN_RX_ID,
		.info.offline_max_cnt = 100,
		.info.cap_offline_cnt = 100,
		.info.wireless_offline_cnt = 100,
		.state = CAP_OFFLINE,

		.update = CAP_rxMessage,
		.setdata = CAP_setMessage,
		.heart_beat = CAP_heart_beat,
};

/**
 * @brief 电容心跳
 *
 * @param self
 */
static void CAP_heart_beat(cap_t *self)
{
	cap_info_t *info = &self->info;

	info->cap_offline_cnt++;
	info->wireless_offline_cnt++;
	if (info->cap_offline_cnt > info->offline_max_cnt) // 每次等待一段时间后自动离线
	{
		info->cap_offline_cnt = info->offline_max_cnt;
		self->state = CAP_OFFLINE;
	}
	else // 每次接收成功就清空计数
	{
		/* 离线->在线 */
		if (self->state == CAP_OFFLINE)
		{
			self->state = CAP_ONLINE;
		}
	}

	if (info->wireless_offline_cnt > info->offline_max_cnt)
	{
		info->wireless_offline_cnt = info->offline_max_cnt;
		self->wireless_charge_state = WIRELESS_CHARGE_OFFLINE;
	}
}

/**
 * @brief 设置电容数据
 *
 * @param self cap
 * @param powerBuff 底盘功率缓冲
 * @param powerLimit 机器人底盘功率限制上限
 * @param volt 底盘输出电压
 * @param current 底盘输出电流
 */
uint16_t input = 300;
int16_t output = -300;
uint16_t cap_switch = 1;
uint16_t turbo_mode = 0;
void CAP_setMessage(cap_t *self, uint16_t powerBuff, uint16_t powerLimit)
{
//	if(communicate.car_data0_rx_info->pre_charge_flag==1)
//	{
//		self->info.cap_tx_data.bit_control.pre_charge_mode_en=1;
//	}
//	else
//	{
//		self->info.cap_tx_data.bit_control.pre_charge_mode_en=0;
//	}
	self->info.cap_tx_data.chassis_power_buffer = powerBuff;
	self->info.cap_tx_data.chassis_power_limit = powerLimit;

	//	if(RC_ONLINE)
	//	{
	//		cap_switch=1;
	//	}
	//	else
	//	{
	//		cap_switch=0;
	//	}
	cap_switch = 1;
	self->info.cap_tx_data.bit_control.cap_switch = cap_switch; // 0开1关
	self->info.cap_tx_data.bit_control.turbo_mode = turbo_mode; // 充电时要不要用缓冲能量
	// 除了电压够而且开超电的情况下，其他情况都是少放电；
	cap.info.cap_tx_data.cap_power_in_limit = input;
	cap.info.cap_tx_data.cap_power_out_limit = output;
}

float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min)
{
	int32_t a_32 = a, a_max_32 = a_max, a_min_32 = a_min;
	int32_t diff_a = a_max_32 - a_min_32;

	if (diff_a == 0)
		return (b_max + b_min) / 2.0f; // 处理除零

	float ratio = (float)(a_32 - a_min_32) / (float)diff_a;
	return ratio * (b_max - b_min) + b_min;
}

/**
 * @brief 超电相关接收数据
 * @param self cap
 * @param canId CAN ID
 * @param rxBuf 数据帧
 */
void CAP_rxMessage(cap_t *self, uint32_t can_Id, uint8_t *rxBuf)
{
	if (can_Id == self->info.can_rx_Id)
	{
		memcpy(&self->info.cap_rx_data, rxBuf, sizeof(capboard_rx_info_t));
		self->info.cap_offline_cnt = 0;
		self->cap_U = int16_to_float(self->info.cap_rx_data.now_cap_V, 32000, -32000, 25, 0);
		self->cap_I = int16_to_float(self->info.cap_rx_data.now_cap_I, 32000, -32000, 16, -16);
	}
	else if (can_Id == self->info.wireless_can_rx_Id)
	{
		memcpy(&self->info.wireless_rx_data, rxBuf, sizeof(wireless_rx_info_t));
		self->info.wireless_offline_cnt = 0;
		self->cap_U = int16_to_float(self->info.wireless_rx_data.charging_power, 32000, -32000, 150, 0);
	}
}
