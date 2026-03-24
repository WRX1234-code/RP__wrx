#ifndef __MY_UI_H
#define __MY_UI_H

#include "stm32h7xx_hal.h"

typedef enum{
	TOP_FRAME,
	FLY_FRAME,
	UPSTEP_FRAME,
	UPSTEP_NUM,//上台阶数字
	BUFF_FRAME,//发现buff
	BUFF_NUM,//buff序号
//	POWER,
//	POWER_FRAME,
	BULLET_NUM,
	CHAS_HEAD_LINE,//车头线
	CHAS_SIDE_LINE,//车侧线
	PITCH_LINE,//机体线
	THETA_LINE,//摆角线
	R_LEG_LENGTH,//右腿长
	L_LEG_LENGTH,//左腿长
//	VISION_FRAME,//发现目标
	CAP_LINE,//超电条
	VISION_AIM,//视觉目标位置
	AUTO_CATCH_FRAME,//自瞄框
	CAR_SPEED,//车体速度
	LENGTH_FRAME,//腿长模式框
	
	
	DYNAMIC_NUM,
}dynamic_ui_cnt_e;

typedef enum{
	TOP_CHAR,
	UPSTEP_CHAR,
	FLY_CHAR,
	BUFF_CHAR,
//	VISION_CHAR,
//	AUTO_CATCH_FRAME,//自瞄框
	CHAS_CIRCLE,//底盘圆盘
	STANDARD_LINE,//腿长默认高度
	HIGH_LINE,//二阶腿长模式水平线
	MINIMUM_LINE,//最低腿长基准线
	CAP_FRAME,//超电框
	MOVE_L_LINE,//左行车线
	MOVE_R_LINE,//右行车线
	LOW_CHAR,//低腿长
	MID_CHAR,//中腿长
	HIGH_CHAR,//高腿长
	CAP_DIVISION_1,//超电分割线
	CAP_DIVISION_2,
	
	CONST_NUM,
}const_ui_cnt_e;

typedef struct UI_Dynamic_Info_struct_t
{
	uint32_t l_leg_length;
	
	uint32_t r_leg_length;
	
}UI_Dynamic_Info_t;

void My_Ui_Init(void);
void Ui_Info_Update(void);
#endif