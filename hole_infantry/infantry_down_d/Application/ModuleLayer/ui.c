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

#define BODY_CENTER_X     (Client_mid_position_x - 700)
#define BODY_CENTER_Y     (Client_mid_position_y + 50)
#define PITCH_LENGTH      100

void rotate_point(__packed uint16_t *x, __packed uint16_t *y, uint16_t raw_x, uint16_t raw_y, float mid_x, float mid_y, float angle);
void My_Chas_Circle_Update(float angle);
static void Launch_Motor_Color_Update(uint8_t state, uint8_t launch_state, uint32_t index);
static void Motor_Color_Update(uint8_t state, uint32_t index);
static void Gimbal_Line_Update(float angle,uint8_t height);

ui_info_t dynamic_ui_info [DYNAMIC_NUM] = 
{
	[MODE_CHAR] = {
		/*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY,
    .ui_config.ui_type = CHAR,     
    .ui_config.name = "d1",            
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    
    .ui_config.layer = 1,                
    .ui_config.color = WHITE,            
		 .ui_config.size = 30,               
    .ui_config.width = 3,                
    .ui_config.start_x = Client_mid_position_x + 670,             
    .ui_config.start_y = Client_mid_position_y + 250,             
    .ui_config.text = "SLEEP",          

	},
	
	
	[VISION_FRAME] = {
		/*******不变配置*********/
    .ui_config.priority = LOW_PRIORITY, 
    .ui_config.ui_type = RECTANGEL,        
    .ui_config.name = "d2",              
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    
    .ui_config.layer = 1,               
    .ui_config.color = WHITE,           
    .ui_config.width = 3,               
     .ui_config.start_x = 170 ,             
    .ui_config.start_y = Client_mid_position_y + 310 ,              
		.ui_config.end_x = 390 ,
		.ui_config.end_y = Client_mid_position_y + 250 ,
		
	},
	
	[VISION_NUM] = {
		/*******不变配置*********/
		.ui_config.priority = HIGH_PRIORITY,
    .ui_config.ui_type = INT,          
    .ui_config.name = "d3",             
    /*******可变配置*********/
		.ui_config.operate_type = MODIFY,  
    .ui_config.layer = 1,               
    .ui_config.color = CYAN_BLUE,            
    .ui_config.size = 30,               
    .ui_config.width = 2,               
    .ui_config.start_x = 360,             
    .ui_config.start_y = Client_mid_position_y + 297,             
    .ui_config.int_num = 0,
		
	},
	
	[BULLET_NUM] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY,
    .ui_config.ui_type = INT,           
    .ui_config.name = "d4",             
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = CYAN_BLUE,          
    .ui_config.size = 60,              
    .ui_config.width = 4,              
    .ui_config.start_x = Client_mid_position_x + 310,           
    .ui_config.start_y = Client_mid_position_y - 90,             
    .ui_config.int_num = 0,            
	},
	
	[CHAS_HEAD_LINE] = {
		/*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY,
    .ui_config.ui_type = LINE,        
    .ui_config.name = "d5",             
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 2,               
    .ui_config.color = CYAN_BLUE,           
    .ui_config.width = 4,               
    .ui_config.start_x = CHAS_CIRCLE_X,             
    .ui_config.start_y = CHAS_CIRCLE_Y,              
    .ui_config.end_x = CHAS_CIRCLE_X,               
    .ui_config.end_y = CHAS_CIRCLE_Y + CHAS_CIRCLE_R,               
  },
	
	[CHAS_SIDE_LINE] = {
		/*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY,
    .ui_config.ui_type = LINE,        
    .ui_config.name = "d6",             
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    
    .ui_config.layer = 2,                
    .ui_config.color = WHITE,            
    .ui_config.width = 2,               
    .ui_config.start_x = CHAS_CIRCLE_X - CHAS_CIRCLE_R,             
    .ui_config.start_y = CHAS_CIRCLE_Y,              
    .ui_config.end_x = CHAS_CIRCLE_X + CHAS_CIRCLE_R,               
    .ui_config.end_y = CHAS_CIRCLE_Y,                
	},
	
	
	[CAP_LINE] = {
		 /*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, 
    .ui_config.ui_type = LINE,         
    .ui_config.name = "d7",             
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    
    .ui_config.layer = 1,               
    .ui_config.color = GREEN,           
    .ui_config.width = 25,               
    .ui_config.start_x = Client_mid_position_x - 250,            
    .ui_config.start_y = Client_mid_position_y + 332,             
    .ui_config.end_x = Client_mid_position_x + 250,               
    .ui_config.end_y = Client_mid_position_y + 332,                
	},
	
	
	[AUTO_CATCH_FRAME] = {
		 /*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, 
    .ui_config.ui_type = RECTANGEL,      
    .ui_config.name = "d8",              
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    
    .ui_config.layer = 1,                
    .ui_config.color = WHITE,            
    .ui_config.width = 3,                
     .ui_config.start_x = Client_mid_position_x - 280,             
    .ui_config.start_y = Client_mid_position_y + 170 ,             
		.ui_config.end_x = Client_mid_position_x + 280 ,
		.ui_config.end_y = Client_mid_position_y - 180 ,
	},
	
	[CAR_SPEED] = {
		/*******不变配置*********/
  .ui_config.priority = MID_PRIORITY,
  .ui_config.ui_type = FLOAT,
	.ui_config.name = "d9",
  /*********不变配置*********/
  .ui_config.operate_type = MODIFY, 
  .ui_config.layer = 1, 
  .ui_config.color = CYAN_BLUE, 
  .ui_config.size = 30, 
  .ui_config.width = 2,
  .ui_config.start_x = Client_mid_position_x - 60,
  .ui_config.start_y = Client_mid_position_y - 400, 
  .ui_config.float_num = 0, 
  .ui_config.decimal = 2, 
	},
	
	[R_FRIC_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, 
    .ui_config.ui_type = CIRCLE,        
    .ui_config.name = "d10",              
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    
    .ui_config.layer = 1,                
    .ui_config.color = GREEN,            
    .ui_config.width = 1,                
    .ui_config.start_x = Client_mid_position_x + 750,              
    .ui_config.start_y = Client_mid_position_y + 100,              
		.ui_config.radius = 20,
	},
	
	[L_FRIC_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY,
    .ui_config.ui_type = CIRCLE,       
    .ui_config.name = "d11",           
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,  
    .ui_config.layer = 1,              
    .ui_config.color = GREEN,          
    .ui_config.width = 1,              
    .ui_config.start_x = Client_mid_position_x + 680,            
    .ui_config.start_y = Client_mid_position_y + 100,            
		.ui_config.radius = 20,
	},
	
	[DIAL_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY,
    .ui_config.ui_type = CIRCLE,       
    .ui_config.name = "d12",           
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,  
    .ui_config.layer = 1,              
    .ui_config.color = GREEN,          
    .ui_config.width = 1,              
    .ui_config.start_x = Client_mid_position_x + 715,           
    .ui_config.start_y = Client_mid_position_y + 70,            
		.ui_config.radius = 20,
	},
	
	
	[LF_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, 
    .ui_config.ui_type = CIRCLE,        
    .ui_config.name = "d13",            
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = GREEN,           
    .ui_config.width = 1,               
    .ui_config.start_x = CHAS_CIRCLE_X - CHAS_CIRCLE_R - 30,    
    .ui_config.start_y = CHAS_CIRCLE_Y + 30,                    
		.ui_config.radius = 20,
	},
	
	[LB_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, 
    .ui_config.ui_type = CIRCLE,        
    .ui_config.name = "d14",            
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = GREEN,           
    .ui_config.width = 1,               
    .ui_config.start_x = CHAS_CIRCLE_X - CHAS_CIRCLE_R - 30,          
    .ui_config.start_y = CHAS_CIRCLE_Y - 30,                          
		.ui_config.radius = 20,
	},
	
	[RF_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY,
    .ui_config.ui_type = CIRCLE,       
    .ui_config.name = "d15",           
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,  
    .ui_config.layer = 1,              
    .ui_config.color = GREEN,          
    .ui_config.width = 1,              
    .ui_config.start_x = CHAS_CIRCLE_X + CHAS_CIRCLE_R + 30,          
    .ui_config.start_y = CHAS_CIRCLE_Y + 30,                          
		.ui_config.radius = 20,
	},
	
	[RB_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY,
    .ui_config.ui_type = CIRCLE,       
    .ui_config.name = "d16",           
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,  
    .ui_config.layer = 1,              
    .ui_config.color = GREEN,          
    .ui_config.width = 1,              
    .ui_config.start_x = CHAS_CIRCLE_X + CHAS_CIRCLE_R + 30,             
    .ui_config.start_y = CHAS_CIRCLE_Y - 30,                             
		.ui_config.radius = 20,
	},
	
	[YAW_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, 
    .ui_config.ui_type = CIRCLE,        
    .ui_config.name = "d17",            
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = GREEN,           
    .ui_config.width = 1,               
    .ui_config.start_x = BODY_CENTER_X, 
    .ui_config.start_y = BODY_CENTER_Y, 
		.ui_config.radius = 20,
	},
	
	
	[LEFT_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, 
    .ui_config.ui_type = CIRCLE,        
    .ui_config.name = "d18",            
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = GREEN,           
    .ui_config.width = 1,               
    .ui_config.start_x = BODY_CENTER_X - 100,      
    .ui_config.start_y = BODY_CENTER_Y,            
		.ui_config.radius = 20,
	},
	
	[PITCH_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, 
    .ui_config.ui_type = CIRCLE,        
    .ui_config.name = "d19",            
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,   
    .ui_config.layer = 1,               
    .ui_config.color = GREEN,           
    .ui_config.width = 1,               
    .ui_config.start_x = BODY_CENTER_X, 
    .ui_config.start_y = BODY_CENTER_Y + 120,             
		.ui_config.radius = 20,
	},
	
	
	[PITCH_LINE] = {
		 /*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, 
    .ui_config.ui_type = LINE,         
    .ui_config.name = "d20",             
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    
    .ui_config.layer = 1,                
    .ui_config.color = WHITE,            
    .ui_config.width = 2,                
    .ui_config.start_x = BODY_CENTER_X,  
    .ui_config.start_y = BODY_CENTER_Y + 120,              
    .ui_config.end_x = BODY_CENTER_X + PITCH_LENGTH,       
    .ui_config.end_y = BODY_CENTER_Y + 120,                
	},
};

ui_info_t const_ui_info [CONST_NUM] = 
{

	 [VISION_CHAR] = {
		 /*******不变配置*********/
    .ui_config.ui_type = CHAR,          
    .ui_config.name = "g1",             
    /*******可变配置*********/
    .ui_config.layer = 1,               
    .ui_config.color = WHITE,           
    .ui_config.size = 30,               
    .ui_config.width = 2,               
    .ui_config.start_x = 180,           
    .ui_config.start_y = Client_mid_position_y + 297,              
    .ui_config.text = "VISION",           
	 },
	 
	 [CHAS_CIRCLE] = {
		 /*******不变配置*********/
    .ui_config.ui_type = CIRCLE,        
    .ui_config.name = "g2",             
    /*******可变配置*********/
    .ui_config.layer = 0,               
    .ui_config.color = GREEN,           
    .ui_config.width = 2,               
    .ui_config.start_x = CHAS_CIRCLE_X ,
    .ui_config.start_y = CHAS_CIRCLE_Y, 
    .ui_config.radius = 65, 
	 },
	 
	 [CAP_FRAME] = {
		 /*******不变配置*********/
    .ui_config.ui_type = RECTANGEL,    
    .ui_config.name = "g3",            
    /*******可变配置*********/
 
    .ui_config.layer = 1,              
    .ui_config.color = WHITE,          
    .ui_config.width = 3,              
    .ui_config.start_x = Client_mid_position_x - 253,            
    .ui_config.start_y = Client_mid_position_y + 345 ,           
		.ui_config.end_x = Client_mid_position_x + 253 ,
		.ui_config.end_y = Client_mid_position_y + 317 ,
	 },
	 
	 [MOVE_L_LINE] = {
		  /*******不变配置*********/
    .ui_config.ui_type = LINE,        
    .ui_config.name = "g4",           
    /*******可变配置*********/
    .ui_config.layer = 1,             
    .ui_config.color = WHITE,         
    .ui_config.width = 1,             
    .ui_config.start_x = Client_mid_position_x - 334,            
    .ui_config.start_y = 0,             
    .ui_config.end_x = Client_mid_position_x - 82,               
    .ui_config.end_y = Client_mid_position_y - 150 ,             
		 
	 },

	 [MOVE_R_LINE] = {
		  /*******不变配置*********/
    .ui_config.ui_type = LINE,         
    .ui_config.name = "g5",            
    /*******可变配置*********/
    .ui_config.layer = 1,              
    .ui_config.color = WHITE,          
    .ui_config.width = 1,              
    .ui_config.start_x = Client_mid_position_x + 334,            
    .ui_config.start_y = 0,            
    .ui_config.end_x = Client_mid_position_x + 82,               
    .ui_config.end_y = Client_mid_position_y - 150 ,             
		 
	 },

	 
	 [CAP_DIVISION_1] = {
		 /*******不变配置*********/
    .ui_config.ui_type = LINE,         
    .ui_config.name = "g6",            
    /*******可变配置*********/
    .ui_config.layer = 0,              
    .ui_config.color = WHITE,          
    .ui_config.width = 2,              
    .ui_config.start_x = Client_mid_position_x - 250 + 70,        
    .ui_config.start_y = Client_mid_position_y + 355,             
    .ui_config.end_x = Client_mid_position_x - 250 + 70,          
    .ui_config.end_y = Client_mid_position_y + 305 ,              
	 },
//	 
	 [CAP_DIVISION_2] = {
		 /*******不变配置*********/
    .ui_config.ui_type = LINE,        
    .ui_config.name = "g7",           
    /*******可变配置*********/
    .ui_config.layer = 0,             
    .ui_config.color = WHITE,         
    .ui_config.width = 2,             
    .ui_config.start_x = Client_mid_position_x - 250 + 195,        
    .ui_config.start_y = Client_mid_position_y + 355,              
    .ui_config.end_x = Client_mid_position_x - 250 + 195,          
    .ui_config.end_y = Client_mid_position_y + 305 ,               
	 },
	
	 
	 [BODY_LINE] = {
		 /*******不变配置*********/
    .ui_config.ui_type = LINE,        
    .ui_config.name = "g8",           
    /*******可变配置*********/
    .ui_config.layer = 0,             
    .ui_config.color = WHITE,         
    .ui_config.width = 2,             
    .ui_config.start_x = BODY_CENTER_X - 100,      
    .ui_config.start_y = BODY_CENTER_Y,            
    .ui_config.end_x = BODY_CENTER_X + 100,        
    .ui_config.end_y = BODY_CENTER_Y ,             
	 },
	
	 
	  [LEFT_LINE] = {
		 /*******不变配置*********/
    .ui_config.ui_type = LINE,       
    .ui_config.name = "g9",          
    /*******可变配置*********/
    .ui_config.layer = 0,            
    .ui_config.color = WHITE,        
    .ui_config.width = 2,            
    .ui_config.start_x = BODY_CENTER_X,           
    .ui_config.start_y = BODY_CENTER_Y,           
    .ui_config.end_x = BODY_CENTER_X,             
    .ui_config.end_y = BODY_CENTER_Y + 120 ,      
	 },
	 
};





void My_Ui_Init(void)
{
  Init_Ui_List(dynamic_ui_info, sizeof(dynamic_ui_info)/sizeof(ui_info_t),const_ui_info, sizeof(const_ui_info)/sizeof(ui_info_t));
}

void Ui_Info_Update(void)
{
	client_info_update();
	
	//整车模式更新
	static Infantry_Mode_e  last_mode = I_SLEEP;
	
	if(last_mode != infantry.mode)
	{
		if(infantry.mode == I_SLEEP)
		{
			strcpy(dynamic_ui_info[MODE_CHAR].ui_config.text, "SLEEP");
		}
		else if(infantry.mode == I_INIT)
		{
			strcpy(dynamic_ui_info[MODE_CHAR].ui_config.text, "INIT");
		}
		else if(infantry.mode == I_IMU)
		{
			strcpy(dynamic_ui_info[MODE_CHAR].ui_config.text, "IMU");
		}
		else if(infantry.mode == I_MEC)
		{
			strcpy(dynamic_ui_info[MODE_CHAR].ui_config.text, "MEC");
		}
		else if(infantry.mode == I_TURN)
		{
			strcpy(dynamic_ui_info[MODE_CHAR].ui_config.text, "TOP");
		}
		else if(infantry.mode == I_HOLE)
		{
			strcpy(dynamic_ui_info[MODE_CHAR].ui_config.text, "HOLE");
		}
		
		Enqueue_Ui_For_Sending(&dynamic_ui_info[MODE_CHAR]);

	}
	
	last_mode = infantry.mode;
	
	
	//发射机构状态更新
	static uint8_t last_r_fric_state = 0;
	static uint8_t last_l_fric_state = 0;
	static uint8_t last_dial_state = 0;
	static uint8_t last_launch_state = 0;
	
	if(last_r_fric_state != board.rx_meg->state_meg.r_fric_state || last_launch_state != launch.state)
	{
		Launch_Motor_Color_Update(board.rx_meg->state_meg.r_fric_state, launch.state, R_FRIC_CIRCLE);
	}
	if(last_l_fric_state != board.rx_meg->state_meg.l_fric_state || last_launch_state != launch.state)
	{
		Launch_Motor_Color_Update(board.rx_meg->state_meg.l_fric_state, launch.state, L_FRIC_CIRCLE);
	}
	if(last_dial_state != board.rx_meg->state_meg.dial_motor_state || last_launch_state != launch.state)
	{
		Launch_Motor_Color_Update(board.rx_meg->state_meg.dial_motor_state, launch.state, DIAL_CIRCLE);
	}
  
	last_r_fric_state = board.rx_meg->state_meg.r_fric_state;
  last_l_fric_state = board.rx_meg->state_meg.l_fric_state;
  last_dial_state = board.rx_meg->state_meg.dial_motor_state ;
  last_launch_state = launch.state;


  //底盘轮组状态更新
	static uint8_t last_lf_state = 0;
	static uint8_t last_lb_state = 0;
	static uint8_t last_rf_state = 0;
	static uint8_t last_rb_state = 0;
	
	if(last_lf_state != chassis.wheel->motor[WHEEL_LF]->state->status)
	{
		Motor_Color_Update(chassis.wheel->motor[WHEEL_LF]->state->status, LF_CIRCLE);
	}
	if(last_lb_state != chassis.wheel->motor[WHEEL_LB]->state->status)
	{
		Motor_Color_Update(chassis.wheel->motor[WHEEL_LB]->state->status, LB_CIRCLE);
	}
	if(last_rf_state != chassis.wheel->motor[WHEEL_RF]->state->status)
	{
		 Motor_Color_Update(chassis.wheel->motor[WHEEL_RF]->state->status, RF_CIRCLE);
	}
	if(last_rb_state != chassis.wheel->motor[WHEEL_RB]->state->status)
	{
		 Motor_Color_Update(chassis.wheel->motor[WHEEL_RB]->state->status, RB_CIRCLE);
	}
	
	last_lf_state = chassis.wheel->motor[WHEEL_LF]->state->status;
	last_lb_state = chassis.wheel->motor[WHEEL_LB]->state->status;
  last_rf_state = chassis.wheel->motor[WHEEL_RF]->state->status;
	last_rb_state = chassis.wheel->motor[WHEEL_RB]->state->status;
	
	
	//云台电机状态更新
	static uint8_t last_yaw_state = 0;
	static uint8_t last_pitch_state = 0;
	static uint8_t last_left_state = 0;
	
	if(last_yaw_state != board.rx_meg->state_meg.yaw_motor_state)
	{
		Motor_Color_Update(board.rx_meg->state_meg.yaw_motor_state, YAW_CIRCLE);
	}
	if(last_pitch_state != board.rx_meg->state_meg.pitch_motor_state)
	{
		Motor_Color_Update(board.rx_meg->state_meg.pitch_motor_state, PITCH_CIRCLE);
	}
	if(last_left_state != board.rx_meg->state_meg.height_motor_state)
	{
		Motor_Color_Update(board.rx_meg->state_meg.height_motor_state, LEFT_CIRCLE);
	}
	
	last_yaw_state = board.rx_meg->state_meg.yaw_motor_state;
  last_pitch_state = board.rx_meg->state_meg.pitch_motor_state;
	last_left_state = board.rx_meg->state_meg.height_motor_state;
	
	
	//云台状态更新
	Gimbal_Line_Update(board.rx_meg->gimbal_meg.pitch_mec,board.rx_meg->state_meg.is_down);
	
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
	speed_now = -board.tx_pkt->car_pkt.v_x;
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

/*底盘方位角更新*/
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


//发射机构状态更新
static void Launch_Motor_Color_Update(uint8_t state, uint8_t launch_state, uint32_t index)
{
	if(state == 0)       //电机不在线
  {
    dynamic_ui_info[index].ui_config.color = BLACK;
  }
  else                 
  {
    if(launch_state == 1)   //开发射
    {
      dynamic_ui_info[index].ui_config.color = FUCHSIA;
    }
    else
    {
      dynamic_ui_info[index].ui_config.color = GREEN;
    }
  }
		
  Enqueue_Ui_For_Sending(&dynamic_ui_info[index]);
	
}


static void Motor_Color_Update(uint8_t state, uint32_t index)
{
	if(state == 0)       //电机不在线
  {
    dynamic_ui_info[index].ui_config.color = BLACK;
  }
  else                 //电机在线
  {
    dynamic_ui_info[index].ui_config.color = GREEN;
  }
		
	Enqueue_Ui_For_Sending(&dynamic_ui_info[index]);
	
}


static void Gimbal_Line_Update(float angle,uint8_t height)
{
	static float last_angle = 0;
	static uint8_t last_height = 2;
	
	if(last_angle != angle || last_height != height)
	{
		dynamic_ui_info[PITCH_CIRCLE].ui_config.start_y = BODY_CENTER_Y +40 + height * 40;
		dynamic_ui_info[PITCH_LINE].ui_config.start_y = BODY_CENTER_Y + 40 + height * 40;
		dynamic_ui_info[PITCH_LINE].ui_config.end_y = BODY_CENTER_Y + 40 + height * 40;
		
		
		rotate_point(&dynamic_ui_info[PITCH_LINE].ui_config.end_x,&dynamic_ui_info[PITCH_LINE].ui_config.end_y,
	              BODY_CENTER_X + PITCH_LENGTH, BODY_CENTER_Y + 40 + height * 40,
	              BODY_CENTER_X, BODY_CENTER_Y + 40 + height * 40,   
	              angle);
		
		
		Enqueue_Ui_For_Sending(&dynamic_ui_info[PITCH_CIRCLE]);
		Enqueue_Ui_For_Sending(&dynamic_ui_info[PITCH_LINE]);
	}
	
	last_angle = angle;
	last_height = height;
}
