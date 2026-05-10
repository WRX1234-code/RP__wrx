#include "ui.h"
#include "priority_ui.h"
#include "arm_math.h"
#include "ui_protocol.h"
#include "Balance.h"
#include "chassis.h"
#include "judge.h"
#include "rp_math.h"
#include "car_info.h"
#include "gimbal.h"
#include "board_protocol.h"
#include "cap.h"
#include "gimbal_motor.h"
#include "cap.h"
#include "Straight_Leg_Calc.h"
#include "Filter.h"


//UI_Dynamic_Info_t My_UI_Dynamic_Info;
#define LEFT_UP_X (Client_mid_position_x - 820)
#define LEFT_UP_Y (Client_mid_position_y + 320)
#define RIGHT_UP_X (Client_mid_position_x + 820)
#define RIGHT_UP_Y (Client_mid_position_y + 320)

#define CHAS_CIRCLE_X     (Client_mid_position_x)
#define CHAS_CIRCLE_Y     (Client_mid_position_y + 250 - 5)
#define CHAS_CIRCLE_R     (65)

#define PITCH_CENTER_X    260
#define PITCH_CENTER_Y    690

#define LEG_LENGTH_X      (Client_mid_position_x + 220)   //腿长线起始，以最下面的线为准,高度120，0.24:120
#define LEG_LENGTH_Y      (Client_mid_position_y - 475)

UI_Dynamic_Info_t My_UI_Dynamic_Info;

void UI_Info_Update_Leg_length(void);
void rotate_point(__packed uint16_t *x, __packed uint16_t *y, uint16_t raw_x, uint16_t raw_y, float mid_x, float mid_y, float angle);
void My_Chas_Pitch_Update(float angle);
void My_Chas_Theta_Update(float angle);
void My_Chas_Circle_Update(float angle);

ui_info_t dynamic_ui_info [DYNAMIC_NUM] = 
{
	
	[TOP_FRAME] = {
		/*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d1",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 630 + 120 ,              // 起点 x 坐标,110
    .ui_config.start_y = Client_mid_position_y + 130+ 90 ,              // 起点 y 坐标
		.ui_config.end_x = Client_mid_position_x + 730 + 120 ,
		.ui_config.end_y = Client_mid_position_y + 70 + 90,
                  
	},
	
	[FLY_FRAME] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d2",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 630 ,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 220 ,              // 起点 y 坐标
		.ui_config.end_x = Client_mid_position_x + 730 ,
		.ui_config.end_y = Client_mid_position_y + 160 ,
	},
	
	[UPSTEP_FRAME] = {
		/*******不变配置*********/
    .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d3",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
     .ui_config.start_x = Client_mid_position_x + 630 ,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 310 ,              // 起点 y 坐标
		.ui_config.end_x = Client_mid_position_x + 870 ,
		.ui_config.end_y = Client_mid_position_y + 250 ,
		
	},
	
//	[UPSTEP_NUM] = {
//		/*******不变配置*********/
//		.ui_config.priority = HIGH_PRIORITY,
//    .ui_config.ui_type = INT,           // UI内容类型
//    .ui_config.name = "d4",              // 图形名称
//    /*******可变配置*********/
//		.ui_config.operate_type = MODIFY,    // 操作类型
//    .ui_config.layer = 1,                // 图层数，0~9
//    .ui_config.color = CYAN_BLUE,            // 颜色
//    .ui_config.size = 30,                // 字体大小
//    .ui_config.width = 2,                // 线条宽度
//    .ui_config.start_x = Client_mid_position_x + 830,              // 起点 x 坐标
//    .ui_config.start_y = Client_mid_position_y + 297,              // 起点 y 坐标
//    .ui_config.int_num = 0,
//	},
//	[POWER] = {
//		/*******不变配置*********/
//		.ui_config.priority = MID_PRIORITY,
//    .ui_config.ui_type = CHAR,           // UI内容类型
//    .ui_config.name = "d3",              // 图形名称
//    /*******可变配置*********/
//    .ui_config.layer = 1,                // 图层数，0~9
//    .ui_config.color = WHITE,            // 颜色
//    .ui_config.size = 30,                // 字体大小
//    .ui_config.width = 2,                // 线条宽度
//    .ui_config.start_x = RIGHT_UP_X - 165,              // 起点 x 坐标
//    .ui_config.start_y = RIGHT_UP_Y - 30,              // 起点 y 坐标
//    .ui_config.text = "POWER",            // 显示的文字
//		
//	},
//	
//	[POWER_FRAME] = {
//		/*******不变配置*********/
//    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
//    .ui_config.ui_type = RECTANGEL,         // UI内容类型
//    .ui_config.name = "d4",              // 图形名称
//    /*******可变配置*********/
//    .ui_config.operate_type = MODIFY,    // 操作类型
//    .ui_config.layer = 1,                // 图层数，0~9
//    .ui_config.color = WHITE,            // 颜色
//    .ui_config.width = 5,                // 线条宽度
//    .ui_config.start_x = RIGHT_UP_X - 190,              // 起点 x 坐标
//    .ui_config.start_y = RIGHT_UP_Y - 15 ,              // 起点 y 坐标
//		.ui_config.end_x = RIGHT_UP_X ,
//		.ui_config.end_y = RIGHT_UP_Y - 90 ,
//		
//	},
	
	[BUFF_FRAME] = {
		/*******不变配置*********/
    .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d5",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
     .ui_config.start_x = 170 ,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 310 ,              // 起点 y 坐标
		.ui_config.end_x = 350 ,
		.ui_config.end_y = Client_mid_position_y + 250 ,
		
	},
	
	[BUFF_NUM] = {
		/*******不变配置*********/
		.ui_config.priority = HIGH_PRIORITY,
    .ui_config.ui_type = INT,           // UI内容类型
    .ui_config.name = "d6",              // 图形名称
    /*******可变配置*********/
		.ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = CYAN_BLUE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = 310,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 297,              // 起点 y 坐标
    .ui_config.int_num = 0,
		
	},
	
	[BULLET_NUM] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = INT,            // UI内容类型
    .ui_config.name = "d7",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = CYAN_BLUE,            // 颜色
    .ui_config.size = 60,                // 字体大小
    .ui_config.width = 4,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 310,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y - 90,              // 起点 y 坐标
    .ui_config.int_num = 0,              // 显示的数字
	},
	
	[CHAS_HEAD_LINE] = {
		/*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "d8",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 2,                // 图层数，0~9
    .ui_config.color = CYAN_BLUE,            // 颜色
    .ui_config.width = 4,                // 线条宽度
    .ui_config.start_x = CHAS_CIRCLE_X,              // 起点 x 坐标
    .ui_config.start_y = CHAS_CIRCLE_Y,              // 起点 y 坐标
    .ui_config.end_x = CHAS_CIRCLE_X,                // 终点 x 坐标
    .ui_config.end_y = CHAS_CIRCLE_Y + CHAS_CIRCLE_R,                // 终点 y 坐标
  },
	
	[CHAS_SIDE_LINE] = {
		/*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "d9",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 2,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = CHAS_CIRCLE_X - CHAS_CIRCLE_R,              // 起点 x 坐标
    .ui_config.start_y = CHAS_CIRCLE_Y,              // 起点 y 坐标
    .ui_config.end_x = CHAS_CIRCLE_X + CHAS_CIRCLE_R,                // 终点 x 坐标
    .ui_config.end_y = CHAS_CIRCLE_Y,                // 终点 y 坐标		
	},
		
	[PITCH_LINE] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "d10",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 2,                // 图层数，0~9
    .ui_config.color = CYAN_BLUE,            // 颜色
    .ui_config.width = 7,                // 线条宽度
    .ui_config.start_x = PITCH_CENTER_X - 55,              // 起点 x 坐标
    .ui_config.start_y = PITCH_CENTER_Y,              // 起点 y 坐标
    .ui_config.end_x = PITCH_CENTER_X + 55,                // 终点 x 坐标
    .ui_config.end_y = PITCH_CENTER_Y,                // 终点 y 坐标	
	},
	
	[THETA_LINE] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "d11",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 2,                // 图层数，0~9
    .ui_config.color = PINK,            // 颜色
    .ui_config.width = 7,                // 线条宽度
    .ui_config.start_x = PITCH_CENTER_X,              // 起点 x 坐标
    .ui_config.start_y = PITCH_CENTER_Y,              // 起点 y 坐标
    .ui_config.end_x = PITCH_CENTER_X,                // 终点 x 坐标
    .ui_config.end_y = PITCH_CENTER_Y - 80,  
	},
	
	[R_LEG_LENGTH] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "d12",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = CYAN_BLUE,            // 颜色
    .ui_config.width = 30,                // 线条宽度
    .ui_config.start_x = LEG_LENGTH_X + 60,              // 起点 x 坐标
    .ui_config.start_y = LEG_LENGTH_Y,              // 起点 y 坐标
    .ui_config.end_x = LEG_LENGTH_X + 60,                // 终点 x 坐标
    .ui_config.end_y = LEG_LENGTH_Y + 120,                // 终点 y 坐标
	},
	
	[L_LEG_LENGTH] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "d13",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = PINK,            // 颜色
    .ui_config.width = 30,                // 线条宽度
    .ui_config.start_x = LEG_LENGTH_X + 20,              // 起点 x 坐标
    .ui_config.start_y = LEG_LENGTH_Y,              // 起点 y 坐标
    .ui_config.end_x = LEG_LENGTH_X + 20,                // 终点 x 坐标
    .ui_config.end_y = LEG_LENGTH_Y + 120,                // 终点 y 坐标
	},
	
	[CAP_LINE] = {
		 /*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "d14",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = GREEN,            // 颜色
    .ui_config.width = 25,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x - 250,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 332,              // 起点 y 坐标
    .ui_config.end_x = Client_mid_position_x + 250,                // 终点 x 坐标
    .ui_config.end_y = Client_mid_position_y + 332,                // 终点 y 坐标
	},
	
	[VISION_AIM] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = CIRCLE,         // UI内容类型
    .ui_config.name = "d15",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 1,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x ,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y ,              // 起点 y 坐标
		.ui_config.radius = 3,
	},
	
	[AUTO_CATCH_FRAME] = {
		 /*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d16",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
     .ui_config.start_x = Client_mid_position_x - 280,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 170 ,              // 起点 y 坐标
		.ui_config.end_x = Client_mid_position_x + 280 ,
		.ui_config.end_y = Client_mid_position_y - 180 ,
	},
	
	[CAR_SPEED] = {
		/*不变配置*/
  .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
  .ui_config.ui_type = FLOAT, // UI内容类型
	.ui_config.name = "d17",
  /*可变配置*/
  .ui_config.operate_type = MODIFY, // 操作类型
  .ui_config.layer = 1, // 图层数，0~9
  .ui_config.color = CYAN_BLUE, // 颜色
  .ui_config.size = 30, // 字体大小
  .ui_config.width = 2, // 线条宽度
  .ui_config.start_x = Client_mid_position_x - 60, // 起点 x 坐标
  .ui_config.start_y = Client_mid_position_y - 400, // 起点 y 坐标
  .ui_config.float_num = 0, // 显示的数字
  .ui_config.decimal = 2, // 小数位有效个数
	},
	
	[LENGTH_FRAME] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d18",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = YELLOW,            // 颜色
    .ui_config.width = 3,                // 线条宽度
     .ui_config.start_x = Client_mid_position_x - 350,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y - 125 ,              // 起点 y 坐标
		.ui_config.end_x = Client_mid_position_x - 310 ,
		.ui_config.end_y = Client_mid_position_y - 165 ,
	},
	
	[FRIC_FRAME] = {
		/*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d19",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 630 ,              // 起点 x 坐标,110
    .ui_config.start_y = Client_mid_position_y + 40 ,              // 起点 y 坐标
		.ui_config.end_x = Client_mid_position_x + 770 ,
		.ui_config.end_y = Client_mid_position_y + -20 ,
                  
	},
	[RESCUE_FRAME] = {
		/*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d20",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 630 ,              // 起点 x 坐标,140
    .ui_config.start_y = Client_mid_position_y + 130 ,              // 起点 y 坐标,60
		.ui_config.end_x = Client_mid_position_x + 870 ,
		.ui_config.end_y = Client_mid_position_y + 70 ,
                  
	},
	
	
};

ui_info_t const_ui_info [CONST_NUM] = 
{
	[TOP_CHAR] = {
		/*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g1",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度,没用
    .ui_config.start_x = Client_mid_position_x + 640 + 120,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 117 + 90,              // 起点 y 坐标
    .ui_config.text = "TOP",            // 显示的文字
	},
	
	[UPSTEP_CHAR] = {
		/*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g2",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 640,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 297,              // 起点 y 坐标
    .ui_config.text = "UPSTEP",            // 显示的文字
		
	},
	
	[FLY_CHAR] = {
		/*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g3",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 640,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 207,              // 起点 y 坐标
    .ui_config.text = "FLY",            // 显示的文字
	},
	
//	[AUTO_CATCH_FRAME] = {
//		/*******不变配置*********/
//    .ui_config.ui_type = RECTANGEL,         // UI内容类型
//    .ui_config.name = "g4",              // 图形名称
//    /*******可变配置*********/
// 
//    .ui_config.layer = 1,                // 图层数，0~9
//    .ui_config.color = WHITE,            // 颜色
//    .ui_config.width = 1,                // 线条宽度
//    .ui_config.start_x = Client_mid_position_x - 260,              // 起点 x 坐标
//    .ui_config.start_y = Client_mid_position_y + 180 ,              // 起点 y 坐标
//		.ui_config.end_x = Client_mid_position_x + 260 ,
//		.ui_config.end_y = Client_mid_position_y - 180 ,
//	},
	
//	[VISION_CHAR] = {
//		/*******不变配置*********/
//    .ui_config.ui_type = CHAR,           // UI内容类型
//    .ui_config.name = "g4",              // 图形名称
//    /*******可变配置*********/
//    .ui_config.layer = 1,                // 图层数，0~9
//    .ui_config.color = WHITE,            // 颜色
//    .ui_config.size = 20,                // 字体大小
//    .ui_config.width = 2,                // 线条宽度
//    .ui_config.start_x = RIGHT_UP_X - 40,              // 起点 x 坐标
//    .ui_config.start_y = RIGHT_UP_Y - 60,              // 起点 y 坐标
//    .ui_config.text = "BUFF",            // 显示的文字
//	},
	 [BUFF_CHAR] = {
		 /*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g5",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = 180,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 297,              // 起点 y 坐标
    .ui_config.text = "BUFF",            // 显示的文字
	 },
	 
	 [CHAS_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.ui_type = CIRCLE,         // UI内容类型
    .ui_config.name = "g6",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = GREEN,            // 颜色
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = CHAS_CIRCLE_X ,              // 圆心 x 坐标
    .ui_config.start_y = CHAS_CIRCLE_Y,              // 圆心 y 坐标
    .ui_config.radius = 65, 
	 },
	 
	 
	 [STANDARD_LINE] = {
		 /*******不变配置*********/
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "g8",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = LEG_LENGTH_X,              // 起点 x 坐标
    .ui_config.start_y = LEG_LENGTH_Y + 40,              // 起点 y 坐标
    .ui_config.end_x = LEG_LENGTH_X + 100,                // 终点 x 坐标
    .ui_config.end_y = LEG_LENGTH_Y + 40,                // 终点 y 坐标
	 },
	 
	 [HIGH_LINE] = {
		  /*******不变配置*********/
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "g9",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = YELLOW,            // 颜色
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = LEG_LENGTH_X,              // 起点 x 坐标
    .ui_config.start_y = LEG_LENGTH_Y + 120,              // 起点 y 坐标
    .ui_config.end_x = LEG_LENGTH_X + 100,                // 终点 x 坐标
    .ui_config.end_y = LEG_LENGTH_Y + 120,                // 终点 y 坐标
	 },
	 
	 [MINIMUM_LINE] = {
		 /*******不变配置*********/
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "g10",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = LEG_LENGTH_X,              // 起点 x 坐标
    .ui_config.start_y = LEG_LENGTH_Y,              // 起点 y 坐标
    .ui_config.end_x = LEG_LENGTH_X + 100,                // 终点 x 坐标
    .ui_config.end_y = LEG_LENGTH_Y ,                // 终点 y 坐标
	 },
	 
	 [CAP_FRAME] = {
		 /*******不变配置*********/
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "g11",              // 图形名称
    /*******可变配置*********/
 
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x - 253,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 345 ,              // 起点 y 坐标
		.ui_config.end_x = Client_mid_position_x + 253 ,
		.ui_config.end_y = Client_mid_position_y + 317 ,
	 },
	 
	 [MOVE_L_LINE] = {
		  /*******不变配置*********/
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "g12",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 1,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x - 334,              // 起点 x 坐标
    .ui_config.start_y = 0,              // 起点 y 坐标
    .ui_config.end_x = Client_mid_position_x - 82,                // 终点 x 坐标
    .ui_config.end_y = Client_mid_position_y - 150 ,                // 终点 y 坐标
		 
	 },

	 [MOVE_R_LINE] = {
		  /*******不变配置*********/
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "g13",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 1,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 334,              // 起点 x 坐标
    .ui_config.start_y = 0,              // 起点 y 坐标
    .ui_config.end_x = Client_mid_position_x + 82,                // 终点 x 坐标
    .ui_config.end_y = Client_mid_position_y - 150 ,                // 终点 y 坐标
		 
	 },
	 
	 [LOW_CHAR] = {
		 /*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g14",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x - 340,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y - 130,              // 起点 y 坐标
    .ui_config.text = "L",            // 显示的文字
	 },
	 
	 [MID_CHAR] = {
		 /*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g15",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size =  30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x - 340,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y - 70,              // 起点 y 坐标
    .ui_config.text = "M",            // 显示的文字
	 },
	 
	 [HIGH_CHAR] = {
		 /*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g16",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x - 340,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y - 10,              // 起点 y 坐标
    .ui_config.text = "H",            // 显示的文字
	 },
	 
	 [CAP_DIVISION_1] = {
		 /*******不变配置*********/
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "g17",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x - 250 + 70,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 355,              // 起点 y 坐标
    .ui_config.end_x = Client_mid_position_x - 250 + 70,                // 终点 x 坐标
    .ui_config.end_y = Client_mid_position_y + 305 ,                // 终点 y 坐标
	 },
	 
	 [CAP_DIVISION_2] = {
		 /*******不变配置*********/
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "g18",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x - 250 + 195,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 355,              // 起点 y 坐标
    .ui_config.end_x = Client_mid_position_x - 250 + 195,                // 终点 x 坐标
    .ui_config.end_y = Client_mid_position_y + 305 ,                // 终点 y 坐标
	 },
	 
	 	[FRIC_CHAR] = {
		/*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g19",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度,没用
    .ui_config.start_x = Client_mid_position_x + 640,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 27,              // 起点 y 坐标
    .ui_config.text = "FRIC",            // 显示的文字
	},
		[RESCUE_CHAR] = {
		/*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g20",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度,没用
    .ui_config.start_x = Client_mid_position_x + 640,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 117,              // 起点 y 坐标
    .ui_config.text = "RESCUE",            // 显示的文字
	},
	
	
};

void My_Ui_Init(void)
{
  Init_Ui_List(dynamic_ui_info, sizeof(dynamic_ui_info)/sizeof(ui_info_t),const_ui_info, sizeof(const_ui_info)/sizeof(ui_info_t));
}

void Ui_Info_Update(void)
{
	client_info_update();
	
	//陀螺框更新
	static uint8_t top_last_mode = false;
	
	if(top_last_mode != Balance.Flag->Turn_Flag)
	{
	  if(Balance.Flag->Turn_Flag == true || Balance.Flag->S_Turn_Flag == true)
	  {
		  dynamic_ui_info[TOP_FRAME].ui_config.color = GREEN;
  	}
	  else
	  {
		  dynamic_ui_info[TOP_FRAME].ui_config.color = WHITE;
	  }
	  Enqueue_Ui_For_Sending(&dynamic_ui_info[TOP_FRAME]);
  }
	
	top_last_mode = (Balance.Flag->Turn_Flag || Balance.Flag->S_Turn_Flag);
	
	//发射机构更新
	static uint8_t fric_last_state = 0;
	static uint8_t fric_last_mode = 0;
	
	if(fric_last_mode != D_Board_Rx_Info.is_dial_need_sleep || fric_last_state != D_Board_Tx_Pkt.Launch_state)
	{
	  if(D_Board_Tx_Pkt.Launch_state == 1 && D_Board_Rx_Info.is_dial_need_sleep == 0)
	  {
		  dynamic_ui_info[FRIC_FRAME].ui_config.color = GREEN;
  	}
	  else
	  {
		  dynamic_ui_info[FRIC_FRAME].ui_config.color = WHITE;
	  }
	  Enqueue_Ui_For_Sending(&dynamic_ui_info[FRIC_FRAME]);
  }
	
	fric_last_state = D_Board_Tx_Pkt.Launch_state;
	fric_last_mode = D_Board_Rx_Info.is_dial_need_sleep;
	
	//自救更新
	static uint8_t rescue_mode = false;
	
	if(rescue_mode != Balance.Flag->Rescue_Flag)
	{
	  if(Balance.Flag->Rescue_Flag == true)
	  {
		  dynamic_ui_info[RESCUE_FRAME].ui_config.color = GREEN;
  	}
	  else
	  {
		  dynamic_ui_info[RESCUE_FRAME].ui_config.color = WHITE;
	  }
	  Enqueue_Ui_For_Sending(&dynamic_ui_info[RESCUE_FRAME]);
  }
	
	rescue_mode = Balance.Flag->Rescue_Flag;
	
	//上台阶框更新
	static uint8_t upstep_last_mode = false;
	
	if(upstep_last_mode != Balance.Flag->Knee_Strike_Flag)
	{
		if(Balance.Flag->Knee_Strike_Flag == true)
		{
			dynamic_ui_info[UPSTEP_FRAME].ui_config.color = GREEN;
		}
		else
		{
			dynamic_ui_info[UPSTEP_FRAME].ui_config.color = WHITE;
		}
		Enqueue_Ui_For_Sending(&dynamic_ui_info[UPSTEP_FRAME]);
	}
	
	upstep_last_mode = Balance.Flag->Knee_Strike_Flag;
//	
//	if(My_Judge.org_info->power_heat_data.chassis_power_buffer <= 25.f)
//	{
//		dynamic_ui_info[POWER_FRAME].ui_config.color = ORANGE;
//		dynamic_ui_info[POWER].ui_config.color = WHITE;
//		Enqueue_Ui_For_Sending(&dynamic_ui_info[POWER_FRAME]);
//	}
//	else if(My_Judge.org_info->power_heat_data.chassis_power_buffer <= 8.f)
//	{
//		dynamic_ui_info[POWER_FRAME].ui_config.color = FUCHSIA;
//		dynamic_ui_info[POWER].ui_config.color = FUCHSIA;
//		Enqueue_Ui_For_Sending(&dynamic_ui_info[POWER_FRAME]);
//	}
//	else
//	{
//		dynamic_ui_info[POWER_FRAME].ui_config.color = WHITE;
//		dynamic_ui_info[POWER].ui_config.color = WHITE;
//		Enqueue_Ui_For_Sending(&dynamic_ui_info[POWER_FRAME]);
//	}

  //上台阶数字更新，2025.7.29不更新了
//	static uint8_t upstep_num = 0,upstep_last_num = 0;
//	if(My_Chassis.Knee_Strike_Flag_1 == true)
//	{
//		upstep_num = 1;
//	}
//	else if(My_Chassis.Knee_Strike_Flag_2 == true)
//	{
//		upstep_num = 2;
//	}
//	else 
//	{
//		upstep_num = 0;
//	}
//	
//	if(upstep_last_num != upstep_num)
//	{
//		if(upstep_num == 1)
//		{
//			dynamic_ui_info[UPSTEP_NUM].ui_config.int_num = 1;
//		}
//		else if(upstep_num == 2)
//		{
//			dynamic_ui_info[UPSTEP_NUM].ui_config.int_num = 2;
//		}
//		else
//		{
//			dynamic_ui_info[UPSTEP_NUM].ui_config.int_num = 0;
//		}
//		Enqueue_Ui_For_Sending(&dynamic_ui_info[UPSTEP_NUM]);
//	}
//	
//	upstep_last_num = upstep_num;
	
	//剩余发弹量更新
	static int16_t bullet_remain = 0;
	
	if(bullet_remain != judge.info->projectile_allowance.projectile_allowance_17mm)
	{
		dynamic_ui_info[BULLET_NUM].ui_config.int_num = (int16_t)judge.info->projectile_allowance.projectile_allowance_17mm;
		Enqueue_Ui_For_Sending(&dynamic_ui_info[BULLET_NUM]);
	}
	
	bullet_remain =  judge.info->projectile_allowance.projectile_allowance_17mm;
	
	//飞坡框更新
	
	static uint8_t fly_last_mode = false;
	
	if(fly_last_mode != Balance.Flag->Fly_Flag)
	{
		if(Balance.Flag->Fly_Flag == true)
		{
			dynamic_ui_info[FLY_FRAME].ui_config.color = GREEN;
		}
		else if(Balance.Flag->Fly_Flag == false)
		{
			dynamic_ui_info[FLY_FRAME].ui_config.color = WHITE;
		}
		Enqueue_Ui_For_Sending(&dynamic_ui_info[FLY_FRAME]);
	}
	fly_last_mode = Balance.Flag->Fly_Flag;
	
	//视觉数字更新
	static uint8_t vision_last_mode = 0;
	
	if(vision_last_mode != D_Board_Tx_Pkt.vision_mode)
	{
		if(D_Board_Tx_Pkt.vision_mode == 1)
		{
			dynamic_ui_info[BUFF_NUM].ui_config.int_num = 1;
		}
		else if(D_Board_Tx_Pkt.vision_mode == 2)
		{
			dynamic_ui_info[BUFF_NUM].ui_config.int_num = 2;
		}
		else if(D_Board_Tx_Pkt.vision_mode == 3)
		{
			dynamic_ui_info[BUFF_NUM].ui_config.int_num = 3;
		}
		else if(D_Board_Tx_Pkt.vision_mode == 4)
		{
			dynamic_ui_info[BUFF_NUM].ui_config.int_num = 4;
		}
		else if(D_Board_Tx_Pkt.vision_mode == 5)
		{
			dynamic_ui_info[BUFF_NUM].ui_config.int_num = 5;
		}
		else
		{
			dynamic_ui_info[BUFF_NUM].ui_config.int_num = 0;
		}
		
		Enqueue_Ui_For_Sending(&dynamic_ui_info[BUFF_NUM]);
	}
	vision_last_mode = D_Board_Tx_Pkt.vision_mode;
	
	//视觉框更新
	static uint8_t vision_last_state = 0;
	
	if(vision_last_state != D_Board_Rx_Info.vision_state)
	{
		if(D_Board_Rx_Info.vision_state == 1)
		{
			dynamic_ui_info[BUFF_FRAME].ui_config.color = GREEN;
		}
		else
		{
			dynamic_ui_info[BUFF_FRAME].ui_config.color = WHITE;
		}
		Enqueue_Ui_For_Sending(&dynamic_ui_info[BUFF_FRAME]);
	}
	vision_last_state = D_Board_Rx_Info.vision_state;
	
	//腿长长度更新
	UI_Info_Update_Leg_length();
	
	static uint32_t l_last = 0, r_last = 0;
	
	if(l_last != My_UI_Dynamic_Info.l_leg_length)
	{
		dynamic_ui_info[L_LEG_LENGTH].ui_config.end_y = dynamic_ui_info[L_LEG_LENGTH].ui_config.start_y + My_UI_Dynamic_Info.l_leg_length;
		
		Enqueue_Ui_For_Sending(&dynamic_ui_info[L_LEG_LENGTH]);
	}
	
	l_last = My_UI_Dynamic_Info.l_leg_length;
	
	if(r_last != My_UI_Dynamic_Info.r_leg_length)
	{
		dynamic_ui_info[R_LEG_LENGTH].ui_config.end_y = dynamic_ui_info[R_LEG_LENGTH].ui_config.start_y + My_UI_Dynamic_Info.r_leg_length;
		
		Enqueue_Ui_For_Sending(&dynamic_ui_info[R_LEG_LENGTH]);
	}
	
	r_last = My_UI_Dynamic_Info.r_leg_length;
	
	//超电条更新,24.f:500
	float cap_V = 0;
	static uint16_t cap_line = 0,cap_line_last = 0;
	
	cap_line = (uint16_t)(((cap.info->cap_Ucr * cap.info->cap_Ucr) / (24.f * 24.f)) * 500);
	
	if(cap_line_last != cap_line)
	{
		dynamic_ui_info[CAP_LINE].ui_config.end_x = (Client_mid_position_x -250) + cap_line;
//		dynamic_ui_info[CAP_LINE].ui_config.end_y = Client_mid_position_y + 320;
		if(((cap.info->cap_Ucr * cap.info->cap_Ucr) / (24.f * 24.f)) <= 0.14)
		{
			dynamic_ui_info[CAP_LINE].ui_config.color = FUCHSIA;
		}			
		else if(((cap.info->cap_Ucr * cap.info->cap_Ucr) / (24.f * 24.f)) <= 0.39)
		{
			dynamic_ui_info[CAP_LINE].ui_config.color = ORANGE;
		}
		else
		{
			dynamic_ui_info[CAP_LINE].ui_config.color = GREEN;
		}
		Enqueue_Ui_For_Sending(&dynamic_ui_info[CAP_LINE]);
	}
	cap_line_last = cap_line;
	
	//机体线更新
	static float pitch_err_last = 0.f,pitch_err_now;
	
	pitch_err_now = -Straight_Leg[R_Leg].info->thetab_err;
	pitch_err_now = Lowpass(pitch_err_last,pitch_err_now,0.5f);
	if(pitch_err_last != pitch_err_now)
	{
		My_Chas_Pitch_Update(pitch_err_now);
		Enqueue_Ui_For_Sending(&dynamic_ui_info[PITCH_LINE]);
	}
	pitch_err_last = pitch_err_now;
	
	//摆角线更新
	static float theta_err_last = 0.f,theta_err_now;
	
	theta_err_now = Straight_Leg[R_Leg].info->thetal_err;//右视图
	theta_err_now = Lowpass(theta_err_last,theta_err_now,0.5f);
	if(theta_err_last != theta_err_now)
	{
		My_Chas_Theta_Update(theta_err_now);
		Enqueue_Ui_For_Sending(&dynamic_ui_info[THETA_LINE]);
	}
	theta_err_last = theta_err_now;
	
	//底盘方位角更新
	static float chas_angle_err_last = 0.f,test_chas_angle = 0.f;
	
	test_chas_angle = -(Y_ZERO_ANGLE - gimbal.yaw->rx_info->motor_angle);
	if(abs(test_chas_angle) > PI)
	{
		test_chas_angle -= sgn(test_chas_angle) * 2 * PI;
	}
	
//	test_chas_angle = (test_chas_angle / 4096.f) * PI;
	if(chas_angle_err_last != test_chas_angle)
	{
		My_Chas_Circle_Update(test_chas_angle);
		
	}
	
	//自瞄框更新
	static uint8_t last_vision_state = 0,vision_now_state = 0;
	static uint8_t vision_lost = 0;
	
	if((D_Board_Tx_Pkt.vision_mode == 1 || D_Board_Tx_Pkt.vision_mode == 5) && D_Board_Rx_Info.is_find_Target == 1)
	{
		vision_now_state = 1;
	}
	else if(D_Board_Tx_Pkt.vision_mode == 4 && D_Board_Rx_Info.is_find_Target == 1)
	{
		vision_now_state = 2;
	}
	else if((D_Board_Tx_Pkt.vision_mode == 2 || D_Board_Tx_Pkt.vision_mode == 3) && D_Board_Rx_Info.is_find_dafu == 1)
	{
		vision_now_state = 3;
	}
	else
	{
		vision_now_state = 0;
	}
	
	if(D_Board_Rx_Info.is_hit_now == 1 && D_Board_Rx_Info.is_find_Target == 1 && D_Board_Rx_Info.vision_state == 1)
	{
		vision_lost = 0;
	}
	
	if(vision_last_state != vision_now_state)
	{
		if(vision_now_state == 1)
		{
			dynamic_ui_info[AUTO_CATCH_FRAME].ui_config.color = GREEN;
		}
		else if(vision_now_state == 2)
		{
			dynamic_ui_info[AUTO_CATCH_FRAME].ui_config.color = ORANGE;
		}
		else if(vision_now_state == 3)
		{
			dynamic_ui_info[AUTO_CATCH_FRAME].ui_config.color = PINK;
		}
		else if(vision_now_state == 0)
		{
			dynamic_ui_info[AUTO_CATCH_FRAME].ui_config.color = WHITE;
		}
		Enqueue_Ui_For_Sending(&dynamic_ui_info[AUTO_CATCH_FRAME]);
	}
	
	vision_last_state = vision_now_state;
	
	//视觉丢失
	if(D_Board_Rx_Info.vision_state == 0 && vision_lost == 0)//只更新一次
	{
		dynamic_ui_info[AUTO_CATCH_FRAME].ui_config.color = BLACK;
		Enqueue_Ui_For_Sending(&dynamic_ui_info[AUTO_CATCH_FRAME]);
		vision_lost = 1;
	}
	
	//车体速度更新
	static float speed_last = 0.f,speed_now = 0.f;
	
//	speed_now = XEstimateKF.FilteredValue[1];
	speed_now = Chassis.Posture->info->pitch;
	if(speed_last != speed_now)
	{
		dynamic_ui_info[CAR_SPEED].ui_config.float_num = speed_now;
		Enqueue_Ui_For_Sending(&dynamic_ui_info[CAR_SPEED]);
	}
	speed_last = speed_now;
	
	//腿长模式更新
	static uint8_t length_mode = 1,length_mode_last = 1;
	if(Balance.Flag->Knee_Strike_Flag == true || (Chassis.Leg_Unit[R_Leg]->Link->info->length->l0 > 0.26f && Chassis.Leg_Unit[L_Leg]->Link->info->length->l0 > 0.26f))
	{
		length_mode = 3;
	}
	else if(Balance.Flag->Fly_Flag == true || (Chassis.Leg_Unit[R_Leg]->Link->info->length->l0 >= 0.18f && Chassis.Leg_Unit[R_Leg]->Link->info->length->l0 <= 0.26f 
		       && Chassis.Leg_Unit[L_Leg]->Link->info->length->l0 >= 0.18f && Chassis.Leg_Unit[R_Leg]->Link->info->length->l0 <= 0.26f))
	{
		length_mode = 2;
	}
	else
	{
		length_mode = 1;
	}
	if(length_mode != length_mode_last)
	{
		if(length_mode == 1)
		{
			dynamic_ui_info[LENGTH_FRAME].ui_config.start_y = Client_mid_position_y - 125;
			dynamic_ui_info[LENGTH_FRAME].ui_config.end_y = Client_mid_position_y - 165;
		}
		else if(length_mode == 2)
		{
			dynamic_ui_info[LENGTH_FRAME].ui_config.start_y = Client_mid_position_y - 65;
			dynamic_ui_info[LENGTH_FRAME].ui_config.end_y = Client_mid_position_y - 105;
		}
		else if(length_mode == 3)
		{
			dynamic_ui_info[LENGTH_FRAME].ui_config.start_y = Client_mid_position_y - 5;
			dynamic_ui_info[LENGTH_FRAME].ui_config.end_y = Client_mid_position_y - 45;
		}
		
		Enqueue_Ui_For_Sending(&dynamic_ui_info[LENGTH_FRAME]);
	}
	
	length_mode_last = length_mode;
}


float test_l = 0.f;
float test_r = 0.f;
uint8_t face = 0;//0前1后
float testt = 0.f;

void UI_Info_Update_Leg_length(void)//0.24:120
{	
	if(gimbal.cmd.yaw_mec_tar == gimbal.info.cfg_info.head_to[0])
	{
	  test_l = constrain(Chassis.Leg_Unit[L_Leg]->Link->info->length->l0, MIN_LEG_LENGTH, MAX_LEG_LENGTH) - MIN_LEG_LENGTH;
	  test_r = constrain(Chassis.Leg_Unit[R_Leg]->Link->info->length->l0, MIN_LEG_LENGTH, MAX_LEG_LENGTH) - MIN_LEG_LENGTH;
	}
	else if(gimbal.cmd.yaw_mec_tar == gimbal.info.cfg_info.head_to[4])
	{
		test_r = constrain(Chassis.Leg_Unit[L_Leg]->Link->info->length->l0, MIN_LEG_LENGTH, MAX_LEG_LENGTH) - MIN_LEG_LENGTH;
	  test_l = constrain(Chassis.Leg_Unit[R_Leg]->Link->info->length->l0, MIN_LEG_LENGTH, MAX_LEG_LENGTH) - MIN_LEG_LENGTH;
	}
	float real_range = (MAX_LEG_LENGTH - MIN_LEG_LENGTH);
	
	float gragh_range = abs((float)const_ui_info[HIGH_LINE].ui_config.start_y - (float)const_ui_info[MINIMUM_LINE].ui_config.start_y);
	
	My_UI_Dynamic_Info.l_leg_length = (uint32_t)((test_l / real_range) * gragh_range);
	
	My_UI_Dynamic_Info.r_leg_length = (uint32_t)((test_r / real_range) * gragh_range);	
	
}

/**
 * @brief 把某点绕某点旋转一定角度
 * 
 * @param x 存储旋转后x的地址
 * @param y 存储旋转后y的地址
 * @param raw_x 旋转前x的值
 * @param raw_y 旋转后y的值
 * @param mid_x 旋转原点x
 * @param mid_y 旋转原点y
 * @param angle 旋转的角度rad
 */
void rotate_point(__packed uint16_t *x, __packed uint16_t *y, uint16_t raw_x, uint16_t raw_y, float mid_x, float mid_y, float angle) 
{
  float s = sin(angle);
  float c = cos(angle);
  // 平移到原点
  float origin_x = raw_x - mid_x;
  float origin_y = raw_y - mid_y;
  // 旋转
  float new_x = origin_x * c - origin_y * s;
  float new_y = origin_x * s + origin_y * c;
  // 平移回去并更新原始坐标
  *x = new_x + mid_x;
  *y = new_y + mid_y;
}

/*底盘pitch倾角更新*/
void My_Chas_Pitch_Update(float angle)
{
	rotate_point(&dynamic_ui_info[PITCH_LINE].ui_config.start_x,&dynamic_ui_info[PITCH_LINE].ui_config.start_y,
	              PITCH_CENTER_X - 55,PITCH_CENTER_Y,
	              PITCH_CENTER_X,PITCH_CENTER_Y,
	              angle);
	
	rotate_point(&dynamic_ui_info[PITCH_LINE].ui_config.end_x,&dynamic_ui_info[PITCH_LINE].ui_config.end_y,
	              PITCH_CENTER_X + 55,PITCH_CENTER_Y,
	              PITCH_CENTER_X,PITCH_CENTER_Y,
	              angle);
}
/*底盘摆角更新*/
void My_Chas_Theta_Update(float angle)
{
	rotate_point(&dynamic_ui_info[THETA_LINE].ui_config.end_x,&dynamic_ui_info[THETA_LINE].ui_config.end_y,
	              PITCH_CENTER_X,PITCH_CENTER_Y - 80,
	              PITCH_CENTER_X,PITCH_CENTER_Y,
	              angle);
}

/*底盘方位更新*/
void My_Chas_Circle_Update(float angle)
{
	rotate_point(&dynamic_ui_info[CHAS_HEAD_LINE].ui_config.end_x,&dynamic_ui_info[CHAS_HEAD_LINE].ui_config.end_y,
	              CHAS_CIRCLE_X,CHAS_CIRCLE_Y + CHAS_CIRCLE_R,
	              CHAS_CIRCLE_X,CHAS_CIRCLE_Y,
	              angle);
	
	rotate_point(&dynamic_ui_info[CHAS_SIDE_LINE].ui_config.start_x,&dynamic_ui_info[CHAS_SIDE_LINE].ui_config.start_y,
	              CHAS_CIRCLE_X - CHAS_CIRCLE_R,CHAS_CIRCLE_Y,
	              CHAS_CIRCLE_X,CHAS_CIRCLE_Y,
	              angle);
	
	rotate_point(&dynamic_ui_info[CHAS_SIDE_LINE].ui_config.end_x,&dynamic_ui_info[CHAS_SIDE_LINE].ui_config.end_y,
	              CHAS_CIRCLE_X + CHAS_CIRCLE_R,CHAS_CIRCLE_Y,
	              CHAS_CIRCLE_X,CHAS_CIRCLE_Y,
	              angle);
	
	Enqueue_Ui_For_Sending(&dynamic_ui_info[CHAS_HEAD_LINE]);
	Enqueue_Ui_For_Sending(&dynamic_ui_info[CHAS_SIDE_LINE]);
}
