#include "PID_Instance.h"

/* 腿长控制外环 */
pid_ctrl_t My_Link_Length_Pid[Leg_Num] =
{
	[R_Leg]={
		.kp = 15.f,                    //20.f,//1000
	  .ki = 0.f,                        //0.f,//0.5
	  .kd = 0.f,                     // 0.f,//300000
	  .a = 1.f,
	  .integral_max = 1.f,
	  .out_max = 200.f,	
	},
	[L_Leg]={
		.kp = 14.f,                     //20.f,//1000
	  .ki = 0.f,                     //0.f,//0.5
	  .kd = 0.f,                    //300000
	  .a = 1.f,
	  .integral_max = 1.f,
	  .out_max = 200.f,	
	},
};

/* 腿长控制内环 */
pid_ctrl_t My_Link_Length_Speed_Pid[Leg_Num] =
{
	[R_Leg]={
		.kp = 100.f,          //60.f,                    //1000
	  .ki = 0.01f,                               //0.5
	  .kd = 0.f,                      //300000
	  .a = 0.f,
	  .integral_max = 50.f,
	  .out_max = 45.f,	
	},
	[L_Leg]={
		.kp = 100.f,        //60.f,                      //1000
	  .ki = 0.f,                           //0.5
	  .kd = 0.f,                       //300000
	  .a = 0.f,
	  .integral_max = 0.f,
	  .out_max = 45.f,	
	},
};


/* 单环Roll轴控制 */
pid_ctrl_t My_Link_Roll_Pid[Leg_Num] =
{
	[R_Leg]={
		.kp = 0.f,               // 500.f,                 //600.f     
	  .ki = 0.f,                       
	  .kd = 0.f,                //5000.f,                       
	  .a = 0.5f,
	  .integral_max = 10.f,
	  .out_max = 200.f,	         //400.f
	},
	[L_Leg]={
		.kp = 0.f,         //500.f,                             
	  .ki = 0.f,                              
	  .kd = 0.f,            //5000.f,                              
	  .a = 0.5f,                               
	  .integral_max = 10.f,
	  .out_max = 200.f,	         //400.f
	},
};

/* 外环yaw控制 */
pid_ctrl_t My_yaw_Pid[Leg_Num] =
{
	[R_Leg]={
		.kp = 6.f,                  //12.f,                   
	  .ki = 0.f,                    
	  .kd = 0.f,                    
	  .a = 1.f,
	  .integral_max = 2.f,
	  .out_max = 5.f,	
	},
	[L_Leg]={
		.kp = 6.f,                 //12.f,
	  .ki = 0.f,
	  .kd = 0.f,
	  .a = 1.f,
	  .integral_max = 2.f,
	  .out_max = 5.f,	
	},
};


/* 内环yaw控制 */
pid_ctrl_t My_yaw_speed_Pid[Leg_Num] =
{
	[R_Leg]={
		.kp = 7.f,
	  .ki = 0.01f,
	  .kd = 0.f,
	  .a = 1.f,
	  .integral_max = 1.f,
	  .out_max = 200.f,	
	},
	[L_Leg]={
		.kp = 7.f,
	  .ki = 0.01f,
	  .kd = 0.f,
	  .a = 1.f,
	  .integral_max = 1.f,
	  .out_max = 200.f,	
	},
};

/* 单环双腿协调控制 */
pid_ctrl_t My_Link_sync_Pid[Leg_Num] =
{
	[R_Leg]={
		.kp = 0.f,
	  .ki = 0.f,
	  .kd = 0.f,
	  .a = 1.f,
	  .integral_max = 1.f,
	  .out_max = 200.f,	
	},
	[L_Leg]={
		.kp = 0.f,
	  .ki = 0.f,
	  .kd = 0.f,
	  .a = 1.f,
	  .integral_max = 1.f,
	  .out_max = 200.f,	
	},
};

/* vir_phi0控制内环 */
pid_ctrl_t My_Link_vir_phi0_speed_Pid[Leg_Num] =
{
	[R_Leg]={
		.kp = 0.f,
	  .ki = 0.f,
	  .kd = 0.f,
	  .a = 1.f,
	  .integral_max = 1.f,
	  .out_max = 200.f,	
	},
	[L_Leg]={
		.kp = 0.f,
	  .ki = 0.f,
	  .kd = 0.f,
	  .a = 1.f,
	  .integral_max = 1.f,
	  .out_max = 200.f,	
	},
};

/* vir_phi0控制外环 */
pid_ctrl_t My_Link_vir_phi0_Pid[Leg_Num] = 
{
	[R_Leg] = 
	{
	.kp = 0.f,//
    .ki = 0.f,
    .kd = 0.f,
		.a = 1.f,
    .integral_max = 2.f,
    .out_max = 20.f,//
	},
	[L_Leg] = 
	{
	  .kp = 0.f,//
    .ki = 0.f,
    .kd = 0.f,
	.a = 1.f,
    .integral_max = 2.f,
    .out_max = 5.f,//
	},
};

/* 单环vir_phi0d1控制 */
pid_ctrl_t My_Link_vir_phi0_d1_Pid[Leg_Num] = 
{
	[R_Leg] = 
	{
		.kp = 0.f,//
    .ki = 0.f,
    .kd = 0.f,
		.a = 1.f,
    .integral_max = 2.f,
    .out_max = 0.f,//
	},
	[L_Leg] = 
	{
		.kp = 0.f,
    .ki = 0.f,
    .kd = 0.f,
		.a = 1.f,
    .integral_max = 2.f,
    .out_max = 0.f,//
	},
};


Chassis_Pid_t chassis_PID={
	.roll_cal[R_Leg]=&My_Link_Roll_Pid[R_Leg],
	.roll_cal[L_Leg]=&My_Link_Roll_Pid[L_Leg],
	
	.sync_cal[R_Leg]=&My_Link_sync_Pid[R_Leg],
	.sync_cal[L_Leg]=&My_Link_sync_Pid[L_Leg],
	
	.length_cal[R_Leg]=&My_Link_Length_Pid[R_Leg],
	.length_cal[L_Leg]=&My_Link_Length_Pid[L_Leg],
	
	.length_speed_cal[R_Leg]=&My_Link_Length_Speed_Pid[R_Leg],
	.length_speed_cal[L_Leg]=&My_Link_Length_Speed_Pid[L_Leg],

	.yaw_cal[R_Leg]=&My_yaw_Pid[R_Leg],
	.yaw_cal[L_Leg]=&My_yaw_Pid[L_Leg],
	
	.yaw_speed_cal[R_Leg]=&My_yaw_speed_Pid[R_Leg],
	.yaw_speed_cal[L_Leg]=&My_yaw_speed_Pid[L_Leg],
	
	.vir_phi0_cal[R_Leg]=&My_Link_vir_phi0_Pid[R_Leg],
	.vir_phi0_cal[L_Leg]=&My_Link_vir_phi0_Pid[L_Leg],
	
	.vir_phi0_speed_cal[R_Leg]=&My_Link_vir_phi0_speed_Pid[R_Leg],
	.vir_phi0_speed_cal[L_Leg]=&My_Link_vir_phi0_speed_Pid[L_Leg],
	

	.vir_phi0d1_cal[R_Leg]=&My_Link_vir_phi0_d1_Pid[R_Leg],
	.vir_phi0d1_cal[L_Leg]=&My_Link_vir_phi0_d1_Pid[L_Leg],

};

