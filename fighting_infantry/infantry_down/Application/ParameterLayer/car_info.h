#ifndef __CAR_INFO_H
#define __CAR_INFO_H


/*************************** 机体属性 begin ****************************/
#define WHEEL_RADIUS  0.058f//驱动轮半径，单位：m
//腿杆长，如果为串联腿请将l5置零，l1=l2,l3=l4
#define l1   0.215f
#define l2   0.258f
#define l3   0.258f
#define l4   0.215f
#define l5   0.0f
//各杆质心系数
#define l1_cen     0.472538462f
#define l2_cen     0.f 
#define l3_cen     0.497884383f
#define l4_cen     0.479242587f

//各杆质量
#define m_l1 0.056f
#define m_l2 0.f
#define m_l3 0.2515f
#define m_l4 0.2156f

//驱动轮质量（算上定子）
#define mw  0.5895f

//机体质量
#define mb      19.4078f  //19.8578f  //14.8578fkg
#define m_all   23.2f     //23.65f
#define g  9.81f

//整车旋转半径
#define Rl    0.22068f

#define MAX_LEG_LENGTH    0.325f      //    0.34f  
#define MID_LEG_LENGTH    0.23f
#define MIN_LEG_LENGTH    0.125f      //   0.14f//0.145f    

//单腿质量，四杆总和
#define m_l 1.3066f


#define R_PHI1_UP_ANGLE    (-170.95921f)//-167.776855f
#define R_PHI4_UP_ANGLE    (-7.1415443f)//-12.1129923f
							
#define L_PHI1_UP_ANGLE    (-172.16772f)
#define L_PHI4_UP_ANGLE    (-4.8580765f)
/*************************** 机体属性 end ****************************/


/*************************** 控制配置 begin ****************************/

#define TAR_LEG_LENGTH_INITIAL   (0.17f)//初始目标腿长
#define THETAL_OFFSET             0.09f
#define OFF_GROUND_SUPPORT 		   40.f//离地支持力阈值，越小越难触发，单位：N
#define MAX_LIFT_SPEED           0.2f    //单位：m/s  腿长改变最大速度
#define MAX_SPIN_SPEED           1.8f   //单位：rad/s 车体转向运动最大速度

/*软件限位相关，保护机械结构,应该可以不用加*/
#define LIMIT_RANGE      (10.f)
#define SD_POS_FIX_TOR_K			(-0.1f)   //关节限位力矩补偿系数 -0.1

/*卸力阻尼时间与阻尼系数*/
#define DAMPING_DELAY_MAX_CNT     4000   //阻尼持续时间3s
#define Wheel_Damping_Coefficient 0.0001f //
#define R_Sd_Damping_Coefficient    4.f
#define L_Sd_Damping_Coefficient    4.f

#define MAX_STRAIGHT_SPEED	2.5f

#define RC_INPUT_SD1_ORDER_CORRECT 1.f
#define KEY_INPUT_SD1_ORDER_CORRECT 1.f

#define KEY_SDL_K   0.0005f
#define KEY_TURN_K   0.0005f
    

/*************************** 控制配置 end ****************************/




/*************************** 零点、方向配置 begin ****************************/

/*关节电机零点*///0.9879，0.5574    1.9595
#define R_F_HORIZON_ANGLE (-1.25016558 + 1.9595)
#define R_B_HORIZON_ANGLE (0.121713638 + 0.5574)
#define L_F_HORIZON_ANGLE (1.68467212 - 1.9595)
#define L_B_HORIZON_ANGLE (-1.96827126 - 0.5574)

/*关节电机零点运算方向校正*/
#define R_F_HORIZON_ANGLE_ORDER_CORRECT 1//
#define R_B_HORIZON_ANGLE_ORDER_CORRECT -1//
#define L_F_HORIZON_ANGLE_ORDER_CORRECT -1//
#define L_B_HORIZON_ANGLE_ORDER_CORRECT 1//

/*电机编码器值递增方向修正，逆时针为1，顺时针为-1*/
#define R_F_TIME -1//
#define R_B_TIME 1//
#define L_F_TIME 1//
#define L_B_TIME -1//

/*建模与VMC的Tp方向矫正*/
#define L_TP_LQR_ORDER_CORRECT    -1
#define R_TP_LQR_ORDER_CORRECT    -1

/*建模与VMC的vir_phi0方向矫正*/
#define L_VIR_PHI0_ORDER_CORRECT  -1
#define R_VIR_PHI0_ORDER_CORRECT  -1

/*建模与电机扭矩输出方向矫正*/
#define L_F_ORDER_CORRECT    -1 //关节电机
#define L_B_ORDER_CORRECT    -1
#define R_F_ORDER_CORRECT    1
#define R_B_ORDER_CORRECT    1

#define L_W_ORDER_CORRECT    -1//驱动轮
#define R_W_ORDER_CORRECT    1

/* 双腿协调Tp_sync方向矫正 */
#define L_SYNC_ORDER_CORRECT   -1
#define R_SYNC_ORDER_CORRECT   1

/* Roll角控制Tp_roll方向矫正 */
#define L_TP_Roll_ORDER_CORRECT    1
#define R_TP_Roll_ORDER_CORRECT    -1

/* 转向控制Tw_turn方向矫正 */
#define R_TURN_ORDER_CORRECT    1 
#define L_TURN_ORDER_CORRECT    -1

/* 侧向前馈竖直力F_inertial方向矫正 */
#define L_F_INERTIAL_ORDER_CORRECT -1
#define R_F_INERTIAL_ORDER_CORRECT 1

/* 消除电机定子转动对位移影响方向矫正 */
#define R_STATOR_ORDER_CORRECT 1
#define L_STATOR_ORDER_CORRECT 1

/* 用于求s、sd1的轮速、轮总角度方向矫正 */
#define R_W_SPEED_ORDER_CORRECT 1//
#define L_W_SPEED_ORDER_CORRECT 1//

#define R_W_ANGLESUM_ORDER_CORRECT 1//
#define L_W_ANGLESUM_ORDER_CORRECT -1//

/* 关节电机总角度方向矫正 */
#define R_F_SD_ANGLESUM_ORDER_CORRECT -1//
#define R_B_SD_ANGLESUM_ORDER_CORRECT -1
#define L_F_SD_ANGLESUM_ORDER_CORRECT 1
#define L_B_SD_ANGLESUM_ORDER_CORRECT 1

/*************************** 零点、方向配置 end ****************************/


#define TIME_STEP			0.001f//任务运行周期，单位：s

typedef enum
{
	R_Leg,
	
	L_Leg,
	
	Leg_Num,
}Leg_e;

#endif
