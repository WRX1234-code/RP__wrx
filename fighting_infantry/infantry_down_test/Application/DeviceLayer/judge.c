/* Includes ------------------------------------------------------------------*/
#include "judge.h"
#include "imu_sensor.h"
#include "cap.h"
#include "drv_tick.h"

Judge_Info_t Judge_Info;
Judge_Status_t Judge_Status = 
{
	.offline_cnt_max = JUDGE_OFFLINE_CNT_MAX,
};
Judge_t judge = 
{
	.info = &Judge_Info,
	.status = &Judge_Status,
};


/**
  * @brief  裁判系统初始化
  * @param  My_Judge_t * my_judge
  * @retval None
  */
void Judge_Init(Judge_t* judge)
{
	judge->status->offline_cnt = judge->status->offline_cnt_max;
	judge->status->status = DEV_OFFLINE;
}

/**
  * @brief  裁判系统实时任务，检测离线与状态更新
  * @param  My_Judge_t * my_judge
  * @retval None
  */
void Judge_Heart_Beat(Judge_t* judge)
{
	judge->status->offline_cnt++;
	if(judge->status->offline_cnt >= judge->status->offline_cnt_max)
	{
		Judge_Init(judge);
	}
}

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

void Judge_Update(uint16_t id, uint8_t *rxBuf)
{
    switch (id)
    {
    case ID_game_status:
        memcpy(&judge.info->game_status, rxBuf, LEN_game_status);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;    
		    break;

    case ID_game_result:
        memcpy(&judge.info->game_result, rxBuf, LEN_game_result);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;    
        break;

    case ID_game_robot_HP:
        memcpy(&judge.info->game_robot_HP, rxBuf, LEN_game_robot_HP);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;    
        break;

    case ID_event_data:
        memcpy(&judge.info->event_data, rxBuf, LEN_event_data);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;     
        break;

    case ID_referee_warning:
        memcpy(&judge.info->referee_warning, rxBuf, LEN_referee_warning);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;    
        break;

    case ID_dart_info:
        memcpy(&judge.info->dart_info, rxBuf, LEN_dart_info);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;    
        break;

    case ID_robot_status:
        memcpy(&judge.info->robot_status, rxBuf, LEN_robot_status);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;    
        break;

    case ID_power_heat_data:
        memcpy(&judge.info->power_heat_data, rxBuf, LEN_power_heat_data);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;    
        break;

    case ID_robot_pos:
        memcpy(&judge.info->robot_pos, rxBuf, LEN_robot_pos);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;    
        break;

    case ID_buff:
        memcpy(&judge.info->buff, rxBuf, LEN_buff);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;    
        break;

    case ID_hurt_data:
        memcpy(&judge.info->hurt_data, rxBuf, LEN_hurt_data);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;    
        break;

    case ID_shoot_data:
        memcpy(&judge.info->shoot_data, rxBuf, LEN_shoot_data);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;    
        break;

    case ID_projectile_allowance:
        memcpy(&judge.info->projectile_allowance, rxBuf, LEN_projectile_allowance);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;    
        break;

    case ID_rfid_status:
        memcpy(&judge.info->rfid_status, rxBuf, LEN_rfid_status);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;    
        break;

    case ID_robot_interaction_data:
        memcpy(&judge.info->robot_interaction_data, rxBuf, LEN_robot_interaction_data);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;    
        break;

    case ID_map_command:
        memcpy(&judge.info->map_command, rxBuf, LEN_map_command);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;    
        break;

    case ID_map_robot_data:
        memcpy(&judge.info->map_robot_data, rxBuf, LEN_map_robot_data);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;    
        break;

    case ID_map_data:
        memcpy(&judge.info->map_data, rxBuf, LEN_map_data);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;    
        break;

    case ID_custom_info:
        memcpy(&judge.info->custom_info, rxBuf, LEN_custom_info);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;    
        break;

    case ID_set_video_channel:
        memcpy(&judge.info->set_video_channel, rxBuf, LEN_set_video_channel);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;    
        break;

    case ID_query_video_channel:
        memcpy(&judge.info->query_video_channel, rxBuf, LEN_query_video_channel);
		    judge.status->offline_cnt = 0;
        judge.status->status = DEV_ONLINE;    
        break;

    default:
        break;
    }
}
