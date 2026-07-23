#include "ui.h"
#include "priority_ui.h"
#include "infantry.h"
#include "arm_math.h"
#include "ui_protocol.h"
#include "chassis.h"
#include "judge.h"
#include "rp_math.h"
#include "gimbal.h"
#include "board_protocol.h"
#include "vision.h"
#include "launch.h"
#include "cap.h"


//UI_Dynamic_Info_t My_UI_Dynamic_Info;
#define LEFT_UP_X (Client_mid_position_x - 820)
#define LEFT_UP_Y (Client_mid_position_y + 320)
#define RIGHT_UP_X (Client_mid_position_x + 820)
#define RIGHT_UP_Y (Client_mid_position_y + 320)

#define CHAS_CIRCLE_X     (Client_mid_position_x)
#define CHAS_CIRCLE_Y     (Client_mid_position_y + 250 - 5)
#define CHAS_CIRCLE_R     (65)

void rotate_point(__packed uint16_t *x, __packed uint16_t *y, uint16_t raw_x, uint16_t raw_y, float mid_x, float mid_y, float angle);
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
    .ui_config.start_x = Client_mid_position_x + 630,              // 起点 x 坐标,110
    .ui_config.start_y = Client_mid_position_y + 200,              // 起点 y 坐标
		.ui_config.end_x = Client_mid_position_x + 730 ,
		.ui_config.end_y = Client_mid_position_y + 160,
                  
	},
	
	[FRIC_FRAME] = {
		/*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d2",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 630 ,              // 起点 x 坐标,110
    .ui_config.start_y = Client_mid_position_y + 250 ,              // 起点 y 坐标
		.ui_config.end_x = Client_mid_position_x + 770 ,
		.ui_config.end_y = Client_mid_position_y + 210 ,
                  
	},
	
	
	[HOLE_FRAME] = {
		/*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d3",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 630 ,              // 起点 x 坐标,110
    .ui_config.start_y = Client_mid_position_y + 350 ,              // 起点 y 坐标
		.ui_config.end_x = Client_mid_position_x + 770 ,
		.ui_config.end_y = Client_mid_position_y + 310 ,
                  
	},
	
	[MEC_FRAME] = {
		/*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "d4",              // 图形名称
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 630 ,              // 起点 x 坐标,110
    .ui_config.start_y = Client_mid_position_y + 300 ,              // 起点 y 坐标
		.ui_config.end_x = Client_mid_position_x + 730 ,
		.ui_config.end_y = Client_mid_position_y + 260 ,
                  
	},
	

	[VISION_FRAME] = {
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
		.ui_config.end_x = 390 ,
		.ui_config.end_y = Client_mid_position_y + 250 ,
		
	},
	
	[VISION_NUM] = {
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
    .ui_config.start_x = 350,              // 起点 x 坐标
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
	
	
	[CAP_LINE] = {
		 /*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "d11",              // 图形名称
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
    .ui_config.name = "d12",              // 图形名称
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
    .ui_config.name = "d13",              // 图形名称
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
	.ui_config.name = "d14",
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
    .ui_config.start_x = Client_mid_position_x + 640,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 195,              // 起点 y 坐标
    .ui_config.text = "TOP",            // 显示的文字
	},
	
	[FRIC_CHAR] = {
		/*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g2",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度,没用
    .ui_config.start_x = Client_mid_position_x + 640,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 245,              // 起点 y 坐标
    .ui_config.text = "FRIC",            // 显示的文字
	},
	
		[HOLE_CHAR] = {
		/*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g3",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度,没用
    .ui_config.start_x = Client_mid_position_x + 640,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 345,              // 起点 y 坐标
    .ui_config.text = "HOLE",            // 显示的文字
	},
		
		[MEC_CHAR] = {
		/*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    .ui_config.name = "g4",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 2,                // 线条宽度,没用
    .ui_config.start_x = Client_mid_position_x + 640,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 295,              // 起点 y 坐标
    .ui_config.text = "MEC",            // 显示的文字
	},
	

	 [VISION_CHAR] = {
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
    .ui_config.text = "VISION",            // 显示的文字
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
	 
	 [CAP_FRAME] = {
		 /*******不变配置*********/
    .ui_config.ui_type = RECTANGEL,         // UI内容类型
    .ui_config.name = "g7",              // 图形名称
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
    .ui_config.name = "g8",              // 图形名称
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
    .ui_config.name = "g9",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 1,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x + 334,              // 起点 x 坐标
    .ui_config.start_y = 0,              // 起点 y 坐标
    .ui_config.end_x = Client_mid_position_x + 82,                // 终点 x 坐标
    .ui_config.end_y = Client_mid_position_y - 150 ,                // 终点 y 坐标
		 
	 },

	 
	 [CAP_DIVISION_1] = {
		 /*******不变配置*********/
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "g10",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x - 250 + 70,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 355,              // 起点 y 坐标
    .ui_config.end_x = Client_mid_position_x - 250 + 70,                // 终点 x 坐标
    .ui_config.end_y = Client_mid_position_y + 305 ,                // 终点 y 坐标
	 },
//	 
	 [CAP_DIVISION_2] = {
		 /*******不变配置*********/
    .ui_config.ui_type = LINE,         // UI内容类型
    .ui_config.name = "g11",              // 图形名称
    /*******可变配置*********/
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = WHITE,            // 颜色
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = Client_mid_position_x - 250 + 195,              // 起点 x 坐标
    .ui_config.start_y = Client_mid_position_y + 355,              // 起点 y 坐标
    .ui_config.end_x = Client_mid_position_x - 250 + 195,                // 终点 x 坐标
    .ui_config.end_y = Client_mid_position_y + 305 ,                // 终点 y 坐标
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
	static uint8_t top_last_flag = false;
	
	if(top_last_flag != infantry.flag.turn_flag)
	{
	  if(infantry.flag.turn_flag == true)
	  {
		  dynamic_ui_info[TOP_FRAME].ui_config.color = GREEN;
  	}
	  else
	  {
		  dynamic_ui_info[TOP_FRAME].ui_config.color = WHITE;
	  }
	  Enqueue_Ui_For_Sending(&dynamic_ui_info[TOP_FRAME]);
  }
	
	top_last_flag = infantry.flag.turn_flag;
	
	
	//狗洞框更新
	static uint8_t hole_last_mode = 0;
	
	if(hole_last_mode != infantry.mode)
	{
	  if(infantry.mode == I_HOLE)
	  {
		  dynamic_ui_info[HOLE_FRAME].ui_config.color = GREEN;
  	}
	  else
	  {
		  dynamic_ui_info[HOLE_FRAME].ui_config.color = WHITE;
	  }
	  Enqueue_Ui_For_Sending(&dynamic_ui_info[HOLE_FRAME]);
  }
	
	hole_last_mode = infantry.mode;
	
	//机械框更新
	static uint8_t mec_last_flag = false;
	
	if(mec_last_flag != infantry.flag.mec_flag)
	{
	  if(infantry.flag.mec_flag == true)
	  {
		  dynamic_ui_info[MEC_FRAME].ui_config.color = GREEN;
  	}
	  else
	  {
		  dynamic_ui_info[MEC_FRAME].ui_config.color = WHITE;
	  }
	  Enqueue_Ui_For_Sending(&dynamic_ui_info[MEC_FRAME]);
  }
	
	mec_last_flag = infantry.flag.mec_flag;
	
	
	//发射机构更新
	static uint8_t fric_last_state = 0;
	static uint8_t fric_last_mode = 0;
	
	if(fric_last_mode != (board.rx_meg->state_meg.r_fric_state && board.rx_meg->state_meg.l_fric_state) || fric_last_state != launch.state)
	{
		if((board.rx_meg->state_meg.r_fric_state && board.rx_meg->state_meg.l_fric_state) == 0)
	  {
		  dynamic_ui_info[FRIC_FRAME].ui_config.color = BLACK;
  	}
		else if(launch.state == 1 && (board.rx_meg->state_meg.r_fric_state && board.rx_meg->state_meg.l_fric_state) == 1)
	  {
		  dynamic_ui_info[FRIC_FRAME].ui_config.color = GREEN;
  	}
	  else if(launch.state == 0 && (board.rx_meg->state_meg.r_fric_state && board.rx_meg->state_meg.l_fric_state) == 1)
	  {
		  dynamic_ui_info[FRIC_FRAME].ui_config.color = WHITE;
	  }
	  Enqueue_Ui_For_Sending(&dynamic_ui_info[FRIC_FRAME]);
  }
	
	fric_last_state = launch.state;
	fric_last_mode = (board.rx_meg->state_meg.r_fric_state && board.rx_meg->state_meg.l_fric_state);
	
	
	//剩余发弹量更新
	static int16_t bullet_remain = 0;
	
	if(bullet_remain != judge.pkt->projectile_allowance_17mm)
	{
		dynamic_ui_info[BULLET_NUM].ui_config.int_num = (int16_t)judge.pkt->projectile_allowance_17mm;
		Enqueue_Ui_For_Sending(&dynamic_ui_info[BULLET_NUM]);
	}
	
	bullet_remain = judge.pkt->projectile_allowance_17mm;
	
	
	//视觉数字更新
	static uint8_t vision_last_mode = 0;
	
	if(vision_last_mode != vision.mode)
	{
		if(vision.mode == 1)
		{
			dynamic_ui_info[VISION_NUM].ui_config.int_num = 1;
		}
		else if(vision.mode == 2)
		{
			dynamic_ui_info[VISION_NUM].ui_config.int_num = 2;
		}
		else if(vision.mode == 3)
		{
			dynamic_ui_info[VISION_NUM].ui_config.int_num = 3;
		}
		else if(vision.mode == 4)
		{
			dynamic_ui_info[VISION_NUM].ui_config.int_num = 4;
		}
		else if(vision.mode == 5)
		{
			dynamic_ui_info[VISION_NUM].ui_config.int_num = 5;
		}
		else
		{
			dynamic_ui_info[VISION_NUM].ui_config.int_num = 0;
		}
		
		Enqueue_Ui_For_Sending(&dynamic_ui_info[VISION_NUM]);
	}
	vision_last_mode = vision.mode;
	
	//视觉框更新
	static uint8_t vision_last_state = 0;
	
	if(vision_last_state != board.rx_meg->state_meg.vision_state)
	{
		if(board.rx_meg->state_meg.vision_state == 1)
		{
			dynamic_ui_info[VISION_FRAME].ui_config.color = GREEN;
		}
		else
		{
			dynamic_ui_info[VISION_FRAME].ui_config.color = BLACK;
		}
		Enqueue_Ui_For_Sending(&dynamic_ui_info[VISION_FRAME]);
	}
	vision_last_state = board.rx_meg->state_meg.vision_state;
	
	
	
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
	
	
	//底盘方位角更新
	static float chas_angle_err_last = 0.f,test_chas_angle = 0.f;
	
	test_chas_angle = gimbal.info.yaw_mec - YAW_MEC_ZERO_ANGLE;
	if(fabs(test_chas_angle) > PI)
	{
		test_chas_angle -= sgn(test_chas_angle) * 2 * PI;
	}
	
	if(chas_angle_err_last != test_chas_angle)
	{
		My_Chas_Circle_Update(test_chas_angle);
		
	}
	
	//自瞄框更新
	static uint8_t last_vision_state = 0,vision_now_state = 0;
	static uint8_t vision_lost = 0;
	
	if((vision.mode == 1 || vision.mode == 5) && board.rx_meg->vision_meg.is_find_target == 1)
	{
		vision_now_state = 1;
	}
	else if(vision.mode == 4 && board.rx_meg->vision_meg.is_find_target == 1)
	{
		vision_now_state = 2;
	}
	else if((vision.mode == 2 || vision.mode == 3) && board.rx_meg->vision_meg.is_find_target == 1)
	{
		vision_now_state = 3;
	}
	else
	{
		vision_now_state = 0;
	}
	
	if(board.rx_meg->vision_meg.is_find_target == 1 && board.rx_meg->state_meg.vision_state == 1)
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
	if(board.rx_meg->state_meg.vision_state == 0 && vision_lost == 0)//只更新一次
	{
		dynamic_ui_info[AUTO_CATCH_FRAME].ui_config.color = BLACK;
		Enqueue_Ui_For_Sending(&dynamic_ui_info[AUTO_CATCH_FRAME]);
		vision_lost = 1;
	}
	
	//车体速度更新
	static float speed_last = 0.f,speed_now = 0.f;
	
//	speed_now = XEstimateKF.FilteredValue[1];
	speed_now = board.tx_pkt->car_pkt.v_x;
	if(speed_last != speed_now)
	{
		dynamic_ui_info[CAR_SPEED].ui_config.float_num = speed_now;
		Enqueue_Ui_For_Sending(&dynamic_ui_info[CAR_SPEED]);
	}
	speed_last = speed_now;
	
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
