#ifndef __MY_UI_H
#define __MY_UI_H

#include "stm32h7xx_hal.h"

typedef enum{
	TOP_FRAME,
	FRIC_FRAME,
	HOLE_FRAME,
	MEC_FRAME,
	VISION_FRAME,//发现buff
	VISION_NUM,//buff序号
	BULLET_NUM,
	CHAS_HEAD_LINE,//车头线
	CHAS_SIDE_LINE,//车侧线
	CAP_LINE,//超电条
	VISION_AIM,//视觉目标位置
	AUTO_CATCH_FRAME,//自瞄框
	CAR_SPEED,//车体速度
	CHASSIS_FRAME,//底盘电机状态框
	CHASSIS_NUM,
	
	DYNAMIC_NUM,
}dynamic_ui_cnt_e;

typedef enum{
	TOP_CHAR,
	FRIC_CHAR,
	HOLE_CHAR,
	MEC_CHAR,
	VISION_CHAR,
	CHAS_CIRCLE,//底盘圆盘
	CAP_FRAME,//超电框
	MOVE_L_LINE,//左行车线
	MOVE_R_LINE,//右行车线
	CAP_DIVISION_1,//超电分割线
	CAP_DIVISION_2,
	CHAS_CHAR,
	
	CONST_NUM,
}const_ui_cnt_e;


void My_Ui_Init(void);
void Ui_Info_Update(void);
#endif
