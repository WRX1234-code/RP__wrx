 
/* Includes ------------------------------------------------------------------*/
#include "judge_protocol.h"
//#include "communicate_protocol.h"
#include "string.h"
#include "usart.h"
//#include "cap.h"
#include "rp_config.h"

///**
// * @brief 判断自己是什么颜色->更新本方的ID->更新自己的ID
// * 
// */
//void Determine_ID(void)
//{
//	if(judge.game_robot_status.robot_id < 10)//本机器人的ID，红方
//	{
//    judge.ids.teammate_hero      = 1;
//    judge.ids.teammate_engineer  = 2;
//    judge.ids.teammate_infantry3 = 3;
//    judge.ids.teammate_infantry4 = 4;
//    judge.ids.teammate_infantry5 = 5;
//    judge.ids.teammate_plane     = 6;
//    judge.ids.teammate_sentry    = 7;

//    judge.ids.client_hero      = 0x0101;
//    judge.ids.client_engineer  = 0x0102;
//    judge.ids.client_infantry3 = 0x0103;
//    judge.ids.client_infantry4 = 0x0104;
//    judge.ids.client_infantry5 = 0x0105;
//    judge.ids.client_plane     = 0x0106;

//    if     (judge.game_robot_status.robot_id == hero_red)//不断刷新放置在比赛中更改颜色
//			judge.self_client = judge.ids.client_hero;
//		else if(judge.game_robot_status.robot_id == engineer_red)
//			judge.self_client = judge.ids.client_engineer;
//		else if(judge.game_robot_status.robot_id == infantry3_red)
//			judge.self_client = judge.ids.client_infantry3;
//		else if(judge.game_robot_status.robot_id == infantry4_red)
//			judge.self_client = judge.ids.client_infantry4;
//		else if(judge.game_robot_status.robot_id == infantry5_red)
//			judge.self_client = judge.ids.client_infantry5;
//		else if(judge.game_robot_status.robot_id == plane_red)
//			judge.self_client = judge.ids.client_plane;
//	}
//	else //蓝方
//	{
//    judge.ids.teammate_hero      = 101;
//    judge.ids.teammate_engineer  = 102;
//    judge.ids.teammate_infantry3 = 103;
//    judge.ids.teammate_infantry4 = 104;
//    judge.ids.teammate_infantry5 = 105;
//    judge.ids.teammate_plane     = 106;
//    judge.ids.teammate_sentry    = 107;

//    judge.ids.client_hero      = 0x0165;
//    judge.ids.client_engineer  = 0x0166;
//    judge.ids.client_infantry3 = 0x0167;
//    judge.ids.client_infantry4 = 0x0168;
//    judge.ids.client_infantry5 = 0x0169;
//    judge.ids.client_plane     = 0x016A;

//    if     (judge.game_robot_status.robot_id == hero_blue)
//			judge.self_client = judge.ids.client_hero;
//		else if(judge.game_robot_status.robot_id == engineer_blue)
//			judge.self_client = judge.ids.client_engineer;
//		else if(judge.game_robot_status.robot_id == infantry3_blue)
//			judge.self_client = judge.ids.client_infantry3;
//		else if(judge.game_robot_status.robot_id == infantry4_blue)
//			judge.self_client = judge.ids.client_infantry4;
//		else if(judge.game_robot_status.robot_id == infantry5_blue)
//			judge.self_client = judge.ids.client_infantry5;
//		else if(judge.game_robot_status.robot_id == plane_blue)
//			judge.self_client = judge.ids.client_plane;
//	}
//}

judge_t judge = {
	.judge_offline_max_cnt = 30,//失联计数最大值
};
uint16_t frame_length;
uint16_t cnt_test = 0;
//头CRC8校验->读取祯头-》计算帧长->检验SOF为0xA5->帧尾CRC校验->读取命令ID->清除失联计数
void judge_update(judge_t *judge,uint8_t *rxBuf)
{
	if(Verify_CRC8_Check_Sum(rxBuf, LEN_FRAME_HEAD) == true)//帧头CRC校验
	{
		memcpy(&judge->fream_header, rxBuf, LEN_FRAME_HEAD);//读取帧头信息
		frame_length = LEN_FRAME_HEAD + LEN_CMD_ID + judge->fream_header.data_length + LEN_FRAME_TAIL;//计算帧长
		if(judge->fream_header.sof == JUDGE_FRAME_HEADER)//SOF为0xA5
		{
			if(Verify_CRC16_Check_Sum(rxBuf, frame_length) == true) //帧尾CRC校验
			{
				uint32_t cmd_id = rxBuf[5] | (rxBuf[6]<<8);//读取命令ID,低位优先
				judge->judge_offline_cnt = 0;//清除失联计数		
				switch(cmd_id)
				{
//					case ID_game_state:					
//						memcpy(&judge->game_status,rxBuf+7, judge->fream_header.data_length);					
//						break;
//					case ID_game_result:
//						memcpy(&judge->game_result,rxBuf+7, judge->fream_header.data_length);
//						break;
//					case ID_game_robot_HP:
//						memcpy(&judge->game_robot_HP,rxBuf+7, judge->fream_header.data_length);
//						break;
//					case ID_supply_projectile_action:
//						memcpy(&judge->supply_projectile_action,rxBuf+7, judge->fream_header.data_length);
//						break;
//					case ID_referee_warning:
//						memcpy(&judge->referee_warning,rxBuf+7, judge->fream_header.data_length);
//						break;

//					case ID_game_robot_status:
//						memcpy(&judge->game_robot_status,rxBuf+7, judge->fream_header.data_length);		
//						Determine_ID();//更新机间交互的ID	
//						Game_Robot_Status_Tx();//发送机器人状态	
//					
//						break;
//					case ID_power_heat_data:
//						memcpy(&judge->power_heat_data,rxBuf+7, judge->fream_header.data_length);
//						Power_Heat_Data_Tx();//发送功率热量数据
//						cap.setdata(&cap,
//							judge->power_heat_data.chassis_power_buffer,
//							judge->game_robot_status.chassis_power_limit);

//						cap.judge_pack_state = PACK_HAVE_UPDATED;

//						break;
//					case ID_game_robot_pos:
//						memcpy(&judge->game_robot_pos,rxBuf+7, judge->fream_header.data_length);
//						Game_robot_pos_Tx();
//						break;
//					case ID_buff_musk:
//						memcpy(&judge->buff,rxBuf+7, judge->fream_header.data_length);
//						break;
//					case ID_aerial_robot_energy:
//						memcpy(&judge->aerial_robot_energy,rxBuf+7, judge->fream_header.data_length);
//						break;
//					case ID_robot_hurt:
//						memcpy(&judge->robot_hurt,rxBuf+7, judge->fream_header.data_length);
//						if (judge->robot_hurt.hurt_type == 0)
//						{
//							judge->armor_hurt_state.rx_flag = 1;
//							judge->armor_hurt_state.armor_id = judge->robot_hurt.armor_id + 1;
//						}
//						break;
					case ID_shoot_data:
						memcpy(&judge->shoot_data,rxBuf+7, judge->fream_header.data_length);
					  Shooting_Cmd_Excute_Tick_Calculating(1);
					  Speed_Statistic();
					  if(judge->shoot_mode==1)
					  {
							judge->start_burst_flag=0;
							Shooting_Cmd_Excute_Tick_Calculating(0);
				
					  }
	          
//						Shoot_Data_Tx();//发送射击数据
						break;
//					case ID_bullet_remaining:
//						memcpy(&judge->bullet_remaining,rxBuf+7, judge->fream_header.data_length);
//						break;
//					case ID_rfid_status:
//						memcpy(&judge->rfid_status,rxBuf+7, judge->fream_header.data_length);
//						break;
//					case ID_dart_client_cmd:
//						memcpy(&judge->dart_client_cmd,rxBuf+7, judge->fream_header.data_length);
//						break;
//					case ID_keyboard_information:
//						memcpy(&judge->remote_control,rxBuf+7, judge->fream_header.data_length);
//						judge->user_client_state.offline_cnt = 0;
					
					default:
						break;
				}
			}
		}
		if(rxBuf[frame_length] == JUDGE_FRAME_HEADER)//出现多帧再次读取
			judge_update(judge,&rxBuf[frame_length]);
	}
}

//检查裁判系统失联
void check_judge_offline(judge_t *judge)
{
	judge->judge_offline_cnt++;
	if(judge->judge_offline_cnt > judge->judge_offline_max_cnt)
	{
		judge->judge_offline_cnt = judge->judge_offline_max_cnt;
		judge->judge_work_state = DEV_OFFLINE;
	}
	else if(judge->judge_work_state == DEV_OFFLINE)
	{
		judge->judge_work_state = DEV_ONLINE;
	}
	//键盘
//	judge->user_client_state.offline_cnt++;
//	if (judge->user_client_state.offline_cnt > 200)
//	{
//		judge->user_client_state.offline_cnt = 200;
//		judge->user_client_state.user_client_state = DEV_OFFLINE;
//	}
//	else if (judge->user_client_state.user_client_state == DEV_OFFLINE)
//	{
//		judge->judge_work_state = DEV_ONLINE;
//	}
//	
}	

//检查自己是否复活
//uint8_t check_hero_revive(judge_t *judge)
//{
//	static uint16_t last_robot_HP;
//	uint16_t robot_HP_now;
//	if(judge->game_robot_status.robot_id <= 10)//红方
//	{
//		robot_HP_now=judge->game_robot_HP.red_1_robot_HP;
//	}
//	else{
//		robot_HP_now=judge->game_robot_HP.blue_1_robot_HP;
//	}
//	
//	if(robot_HP_now!=0&&last_robot_HP==0)//上升沿
//	{
//		last_robot_HP=robot_HP_now;
//		return 1;
//	}
//	else
//	{
//		last_robot_HP=robot_HP_now;
//		return 0;
//	}
//	
//}

/**
* @brief 统计弹速
*
*/
shoot_data_t shoot_statistics;
uint8_t cali_flag = 1; //是否计算平均值和平方差
void Speed_Statistic(void)
{
//	shoot_statistics.temperature_L=shoot.fric.thr_fric[FRIC_L]->rx_info->temperature;
	shoot_statistics.speed_now=judge.shoot_data.bullet_speed;
  float s_speed =judge.shoot_data.bullet_speed;
  shoot_statistics.num++;
	
    // 统计速度区间
    if (s_speed <= 23.7f)
    {
        shoot_statistics.lower_237++;
        shoot_statistics. num--; //弹速太离谱不统计
    }
    else if (s_speed >= 23.7f && s_speed <= 23.8f)
    {
        shoot_statistics.speed_237++;
    }
    else if (s_speed > 23.8f && s_speed <= 23.9f)
    {
        shoot_statistics.speed_238++;
    }
    else if (s_speed > 23.9f && s_speed <= 24.0f)
    {
        shoot_statistics.speed_239++;
    }
    else if (s_speed > 24.0f && s_speed <= 24.1f)
    {
        shoot_statistics.speed_240++;
    }
    else if (s_speed > 24.1f && s_speed <= 24.2f)
    {
        shoot_statistics.speed_241++;
    }
    else if (s_speed > 24.2f && s_speed <= 24.3f)
    {
        shoot_statistics.speed_242++;
    }
    else if (s_speed > 24.3f && s_speed <= 24.4f)
    {
        shoot_statistics.speed_243++;
    }
    else if (s_speed > 24.4f && s_speed <= 24.5f)
    {
        shoot_statistics.speed_244++;
    }
    else if (s_speed > 24.5f && s_speed <= 24.6f)
    {
        shoot_statistics.speed_245++;
    }
    else if (s_speed > 24.6f && s_speed <= 24.7f)
    {
        shoot_statistics.speed_246++;
    }
	  else if (s_speed > 24.7f && s_speed <= 24.8f)
    {
        shoot_statistics.speed_247++;
    }
	  else if (s_speed > 24.8f && s_speed <= 24.9f)
    {
        shoot_statistics.speed_248++;
    }
	  else if (s_speed > 24.9f && s_speed <= 25.0f)
    {
        shoot_statistics.speed_249++;
    }
    else if (s_speed > 25.0f)
    {
        shoot_statistics.higher_250++;
        shoot_statistics. num--; //弹速太离谱不统计
    }

    // 统计弹速平均值和方差
    if (cali_flag == 1)
    {
        // 计算平均值
        shoot_statistics.mean = (
            shoot_statistics.speed_237 * 23.7f +
            shoot_statistics.speed_238 * 23.8f +
            shoot_statistics.speed_239 * 23.9f +
            shoot_statistics.speed_240 * 24.0f +
            shoot_statistics.speed_241 * 24.1f +
            shoot_statistics.speed_242 * 24.2f +
            shoot_statistics.speed_243 * 24.3f +
            shoot_statistics.speed_244 * 24.4f +
            shoot_statistics.speed_245 * 24.5f +
            shoot_statistics.speed_246 * 24.6f +
			      shoot_statistics.speed_247 * 24.7f +
			      shoot_statistics.speed_248 * 24.8f +
			      shoot_statistics.speed_249 * 24.9f ) / (float)shoot_statistics.num;
                                         
        // 计算方差
        float sum_of_squares = (
            shoot_statistics.speed_237 * ((23.7f - shoot_statistics.mean) * (23.7f - shoot_statistics.mean)) +
            shoot_statistics.speed_238 * ((23.8f - shoot_statistics.mean) * (23.8f - shoot_statistics.mean)) +
            shoot_statistics.speed_239 * ((23.9f - shoot_statistics.mean) * (23.9f - shoot_statistics.mean)) +
            shoot_statistics.speed_240 * ((24.0f - shoot_statistics.mean) * (24.0f - shoot_statistics.mean)) +
            shoot_statistics.speed_241 * ((24.1f - shoot_statistics.mean) * (24.1f - shoot_statistics.mean)) +
            shoot_statistics.speed_242 * ((24.2f - shoot_statistics.mean) * (24.2f - shoot_statistics.mean)) +
            shoot_statistics.speed_243 * ((24.3f - shoot_statistics.mean) * (24.3f - shoot_statistics.mean)) +
            shoot_statistics.speed_244 * ((24.4f - shoot_statistics.mean) * (24.4f - shoot_statistics.mean)) +
            shoot_statistics.speed_245 * ((24.5f - shoot_statistics.mean) * (24.5f - shoot_statistics.mean)) +
            shoot_statistics.speed_246 * ((24.6f - shoot_statistics.mean) * (24.6f - shoot_statistics.mean)) +
			      shoot_statistics.speed_247 * ((24.7f - shoot_statistics.mean) * (24.7f - shoot_statistics.mean)) +
			      shoot_statistics.speed_248 * ((24.8f - shoot_statistics.mean) * (24.8f - shoot_statistics.mean)) +
			      shoot_statistics.speed_249 * ((24.9f - shoot_statistics.mean) * (24.9f - shoot_statistics.mean)));
                                           
            shoot_statistics.variance = sum_of_squares / (float)shoot_statistics.num;
						
    }
}


/**
 * @brief 打弹命令执行时间计算
 * 
 * @param flag 0；命令开始执行  1：接收到弹速
 */

void Shooting_Cmd_Excute_Tick_Calculating(uint8_t flag)
{
	static uint32_t cmd_start_tick = 0;
	static uint32_t rx_bullet_tick = 0;
	static uint8_t rx_bullet_cnt = 0;
	static uint8_t reset_cnt_flag = 0;
	

	const uint8_t buf_length = 100;
	if (flag == 0)//命令开始执行
	{
		cmd_start_tick = HAL_GetTick();
	}
	else if (flag == 1)//接收到弹速
	{
		rx_bullet_tick = HAL_GetTick();
		shoot_statistics.shooting_cmd_excute_tick=rx_bullet_tick - cmd_start_tick;
		shoot_statistics.shooting_cmd_excute_tick_buf[rx_bullet_cnt]=shoot_statistics.shooting_cmd_excute_tick;
		
		//移动指针
		rx_bullet_cnt++;
		//回归零点
		if(rx_bullet_cnt>buf_length-1)
		{
			rx_bullet_cnt=0;
			reset_cnt_flag=1;
		}
		//计算平均数
		float shooting_cmd_excute_tick_sum;
		float sum_of_squares;
		
		if(reset_cnt_flag==1)//如果回到原点过，直接遍历
		{
			
			for(uint8_t i=0;i<buf_length;i++)
			{
				shooting_cmd_excute_tick_sum+=shoot_statistics.shooting_cmd_excute_tick_buf[i];
			}
			shoot_statistics.shooting_cmd_excute_tick_mean=shooting_cmd_excute_tick_sum/buf_length;
			
			for(uint8_t i=0;i<buf_length;i++)
			{
			  sum_of_squares+=(shoot_statistics.shooting_cmd_excute_tick_buf[i]-shoot_statistics.shooting_cmd_excute_tick_mean)*(shoot_statistics.shooting_cmd_excute_tick_buf[i]-shoot_statistics.shooting_cmd_excute_tick_mean);
			}
			shoot_statistics.shooting_cmd_excute_tick_variance=sum_of_squares/buf_length;
		}
		else//多少个就多少个
		{
			for(uint8_t i=0;i<rx_bullet_cnt;i++)
			{
				shooting_cmd_excute_tick_sum+=shoot_statistics.shooting_cmd_excute_tick_buf[i];
			}
			shoot_statistics.shooting_cmd_excute_tick_mean=shooting_cmd_excute_tick_sum/rx_bullet_cnt;
			
			for(uint8_t i=0;i<rx_bullet_cnt;i++)
			{
			  sum_of_squares+=(shoot_statistics.shooting_cmd_excute_tick_buf[i]-shoot_statistics.shooting_cmd_excute_tick_mean)*(shoot_statistics.shooting_cmd_excute_tick_buf[i]-shoot_statistics.shooting_cmd_excute_tick_mean);
			}
			shoot_statistics.shooting_cmd_excute_tick_variance=sum_of_squares/rx_bullet_cnt;
		}
		
		
	}
}

