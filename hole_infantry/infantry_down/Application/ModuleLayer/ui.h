//#ifndef __UI_H
//#define __UI_H


//#include "main.h"
//#include "rp_config.h"
///*准线参数设置----------------------------------------*/
////OFFSET越大抬的pitch越大
//#define _TOILET_DOWN_OFFSET 165
//#define _TOILET_DOWN_WIDTH 60

//#define _OUTPOST_OFFSET 230
//#define _OUTPOST_WIDTH 	40

//#define _HEIGHT_OFFSET 260
//#define _HEIGHT_WIDTH 120

//#define _VISION__RECTANGEL_X_WIDTH 250
//#define _VISION__RECTANGEL_Y_OFFSET 200
///*UI内容宏定义****************************************************/
// 
//typedef enum{
//	//底下超电条
//	D_CAP_VOLTAGE = 0, // 超电电压
//	//左边状态显示
//	D_FRIC_B_SPEED,		//两级摩擦轮转速
//	D_FRIC_F_SPEED,
//	D_UWB_YAW,					 // UWB's yaw
//	D_PTICH_IMU_ANGLE,   // 俯仰角陀螺仪
//	D_VISION_CIRCLE,     // 视觉是否在线
//	D_SPEED_ADAPT_CYCLE,		     // 小陀螺状态
//	D_CAP_ON_CYCLE, 		 // 是否开超电
//	D_CAR_MODE,          // 车行动模式
//	
//	//右边状态显示
//	D_HIT_TARGET_DISTANCE,
//	D_FRIC_STATE_CYCLE,	//符合速度指示
//	
//	D_RFID_CYCLE,   //RFID状态灯
//	
//	//中间状态显示
//	D_MID_RECTANGEL,   // 中间视觉框
//	
//	D_HIT_TARGET_TRIANGLE_1,   // 击打目标方向指示器
//	D_HIT_TARGET_TRIANGLE_2,
//	
//	D_HIT_HIGHLIGHT_LINE_1, //击打命中提示
//	D_HIT_HIGHLIGHT_LINE_2,
//	
//	D_HEAD_CYCLE,				//指示头方向
//	#ifndef UI_SIMPLIFY
//	//视觉装甲板
//	
//	D_VISION_ARMOR_CYCLE,
//	D_VISION_HP_CYCLE,
//	D_VISION_WHITE_CYCLE,
//	
//	//roi
//	D_ROI_UP,
//	D_ROI_LEFT,
//	D_ROI_DOWN,
//	D_ROI_RIGHT,
//	D_ROI_MID,
//	#endif
// 
//	//pitch指示器
//	D_PITCH_POINTER,
//	
//	
// 
//	DYNAMIC_UI_NUM,
//}dynamic_ui_e;

//typedef enum{
//	//状态灯标题
//	C_VISION_CHAR,
//	C_FRIC_CHAR,
//	C_FRIC_ADAPT_CHAR,
//	C_CAP_CHAR,
//	C_CAR_MODE_CHAR,
//	C_RFID_CHAR,
//	//准心线
//	C_MID_LINE,
//	C_HEIGHT_LINE,
//	C_OUTPOST_LINE,
//	C_TOILET_DOWN_LINE,
//	//准线名称
//	C_HEIGHT_CHAR,
//	C_OUTPOST_CHAR,
//	C_TOILET_DOWN_CHAR,
//	
//	//通过线
//	C_PASS_LINE_LEFT,
//	C_PASS_LINE_RIGHT,
//	
////	#ifndef UI_SIMPLIFY
//	//pitch指示器
//	C_PITCH_LINE_0,
//	C_PITCH_LINE_10,
//	C_PITCH_LINE_20,
//	C_PITCH_LINE_30,
//				
//	C_PITCH_LINE_40,
//	C_PITCH_LINE_50,
//	C_PITCH_LINE_N_10,
//	C_PITCH_LINE_N_20,
//	C_PITCH_LINE_N_30,
//	
//	C_PITCH_0_CHAR,
//	C_PITCH_50_CHAR,
//	C_PITCH_30_CHAR,
//	C_PITCH_N_30_CHAR,
////	#endif
//	CONST_UI_NUM,
//}const_ui_e;

///* 车行动模式枚举 */
//enum 
//{
//  offline_CAR,        //离线模式         0
//  init_CAR,           //初始化模式       1
//  mec_CAR,            //机械模式         2
//  gyro_CAR,           //陀螺仪模式       3
//	cycle_CAR,		      //小陀螺模式       4
//  hole_CAR,           //缩头模式         5

//};

//typedef enum
//{
//	SENTRY,
//	HERO,
//	ENGINEER,
//	INFANRTY_3,
//	INFANRTY_4,
//	INFANRTY_5,
//	OUTPOST,
//  BASE,
//}robot_type_e;

//typedef __packed struct 
//{
//	uint16_t armor_radius ; // 显示装甲板离中心的距离
//	
//}my_ui_config_t;

//void My_Ui_Init(void);
//void Ui_Info_Update(void);
//#endif




