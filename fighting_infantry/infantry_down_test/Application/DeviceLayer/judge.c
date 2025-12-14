/* Includes ------------------------------------------------------------------*/
#include "judge.h"
#include "imu_sensor.h"
#include "cap.h"
#include "drv_tick.h"

//Judge_Info_t Judge_Info;
//Judge_Org_Info_t Judge_Org_Info;
//Judge_Status_t Judge_Status = 
//{
//	.offline_cnt_max = JUDGE_OFFLINE_CNT_MAX,
//};
//My_Judge_t My_Judge = 
//{
//	.org_info = &Judge_Org_Info,
//	.info = &Judge_Info,
//	.status = &Judge_Status,
//};


///**
//  * @brief  裁判系统初始化
//  * @param  My_Judge_t * my_judge
//  * @retval None
//  */
//void My_Judge_Init()
//{
//	My_Judge.status->offline_cnt = My_Judge.status->offline_cnt_max;
//	My_Judge.status->status = DEV_OFFLINE;
//}

///**
//  * @brief  裁判系统实时任务，检测离线与状态更新
//  * @param  My_Judge_t * my_judge
//  * @retval None
//  */
//void My_Judge_Realtime_Task(My_Judge_t* my_judge)
//{
//	my_judge->status->offline_cnt++;
//	if(my_judge->status->offline_cnt >= my_judge->status->offline_cnt_max)
//	{
//		My_Judge_Init();
//	}
//	
//	My_Judge_Update(my_judge);
//}

///**
//  * @brief  裁判系统数据分析后更新状态
//  * @param  My_Judge_t * my_judge
//  * @retval None
//  */
//void My_Judge_Update(My_Judge_t * my_judge)
//{
//	my_judge->info->remain_HP_now = my_judge->info->remain_HP;
//	
//	my_judge->info->car_color = my_judge->org_info->game_robot_status.robot_id;
//	
//	my_judge->info->chassis_power_buffer = my_judge->org_info->power_heat_data.chassis_power_buffer;
//	
//	my_judge->info->chassis_power_limit = my_judge->org_info->game_robot_status.chassis_power_limit;
//	
//	my_judge->info->shooter_cooling_limit = my_judge->org_info->game_robot_status.shooter_barrel_heat_limit;
//	
//	my_judge->info->shooting_speed = my_judge->org_info->shoot_data.bullet_speed;
//	
//	my_judge->info->shooter_cooling_heat = my_judge->org_info->power_heat_data.shooter_id1_17mm_cooling_heat;
//	
//	
//}
//uint16_t shoot_cnt11 = 0;
//uint8_t warning;
//void judge_update(uint16_t id, uint8_t *rxBuf)
//{
//	switch(id)
//	{
//		case ID_rfid_status:
//			memcpy(&My_Judge.org_info->rfid_status, rxBuf, LEN_rfid_status);
//		  My_Judge.status->offline_cnt = 0;
//		  My_Judge.status->status = DEV_ONLINE;
//			break;
//		case ID_game_state:
//			memcpy(&My_Judge.org_info->game_status, rxBuf, LEN_game_state);
//		  My_Judge.status->offline_cnt = 0;
//		  My_Judge.status->status = DEV_ONLINE;
//			break;
//		case ID_power_heat_data:
//			memcpy(&My_Judge.org_info->power_heat_data, rxBuf, LEN_power_heat_data);
//		  My_Judge.status->offline_cnt = 0;
//		  My_Judge.status->status = DEV_ONLINE;
//			cap_send_2E();
//			break;
//		case ID_game_robot_state:	
//			memcpy(&My_Judge.org_info->game_robot_status, rxBuf, LEN_game_robot_state);
//		  My_Judge.status->offline_cnt = 0;
//		  My_Judge.status->status = DEV_ONLINE;
//			break;
//		case ID_shoot_data:
//			memcpy(&My_Judge.org_info->shoot_data, rxBuf, LEN_shoot_data);
//		  My_Judge.status->offline_cnt = 0;
//		  My_Judge.status->status = DEV_ONLINE;
//		
//			break;
//		case ID_game_robot_HP:
//			memcpy(&My_Judge.org_info->ext_game_robot_HP, rxBuf, LEN_game_robot_HP);
//		  My_Judge.status->offline_cnt = 0;
//		  My_Judge.status->status = DEV_ONLINE;
//			break;
//		case ID_robot_hurt:
////			memcpy(&judge.info->ext_robot_hurt, rxBuf, LEN_robot_hurt);
//			break;
//		case ID_bullet_remaining:
//			memcpy(&My_Judge.org_info->ext_bullet_remaining, rxBuf, LEN_bullet_remaining);
//		  My_Judge.status->offline_cnt = 0;
//		  My_Judge.status->status = DEV_ONLINE;
//			break;
//		default:
//			break;
//	}

//}
Judge_t judge;
void Judge_Update(uint16_t id, uint8_t *rxBuf)
{
    switch (id)
    {
    case ID_game_status:
        memcpy(&judge.game_status, rxBuf, LEN_game_status);
        break;

    case ID_game_result:
        memcpy(&judge.game_result, rxBuf, LEN_game_result);
        break;

    case ID_game_robot_HP:
        memcpy(&judge.game_robot_HP, rxBuf, LEN_game_robot_HP);
        break;

    case ID_event_data:
        memcpy(&judge.event_data, rxBuf, LEN_event_data);
        break;

    case ID_referee_warning:
        memcpy(&judge.referee_warning, rxBuf, LEN_referee_warning);
        break;

    case ID_dart_info:
        memcpy(&judge.dart_info, rxBuf, LEN_dart_info);
        break;

    case ID_robot_status:
        memcpy(&judge.robot_status, rxBuf, LEN_robot_status);
        break;

    case ID_power_heat_data:
        memcpy(&judge.power_heat_data, rxBuf, LEN_power_heat_data);
        break;

    case ID_robot_pos:
        memcpy(&judge.robot_pos, rxBuf, LEN_robot_pos);
        break;

    case ID_buff:
        memcpy(&judge.buff, rxBuf, LEN_buff);
        break;

    case ID_hurt_data:
        memcpy(&judge.hurt_data, rxBuf, LEN_hurt_data);
        break;

    case ID_shoot_data:
        memcpy(&judge.shoot_data, rxBuf, LEN_shoot_data);
        break;

    case ID_projectile_allowance:
        memcpy(&judge.projectile_allowance, rxBuf, LEN_projectile_allowance);
        break;

    case ID_rfid_status:
        memcpy(&judge.rfid_status, rxBuf, LEN_rfid_status);
        break;

    case ID_robot_interaction_data:
        memcpy(&judge.robot_interaction_data, rxBuf, LEN_robot_interaction_data);
        break;

    case ID_map_command:
        memcpy(&judge.map_command, rxBuf, LEN_map_command);
        break;

    case ID_map_robot_data:
        memcpy(&judge.map_robot_data, rxBuf, LEN_map_robot_data);
        break;

    case ID_map_data:
        memcpy(&judge.map_data, rxBuf, LEN_map_data);
        break;

    case ID_custom_info:
        memcpy(&judge.custom_info, rxBuf, LEN_custom_info);
        break;

    case ID_set_video_channel:
        memcpy(&judge.set_video_channel, rxBuf, LEN_set_video_channel);
        break;

    case ID_query_video_channel:
        memcpy(&judge.query_video_channel, rxBuf, LEN_query_video_channel);
        break;

    default:
        break;
    }
}
