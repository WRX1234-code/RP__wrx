#ifndef __PID_H
#define __PID_H
#include "main.h"


typedef struct pid_ctrl {
	float	  target;
	float	  measure;
	float 	err;
	float 	last_err;
	float	  kp;
	float 	ki;
	float 	kd;
	float 	pout;
	float 	iout;
	float 	dout;
	float 	out;
	float  	last_out;
	float	  integral;
	float 	integral_max;
	float   difference;
	float 	out_max;
  float   filter_value;
} pid_ctrl_t;


typedef struct {
	pid_ctrl_t	speed;
	pid_ctrl_t	angle;
} motor_pid_t;


typedef struct motor_pid_all_struct
{
	motor_pid_t            speed_pid; 
	motor_pid_t            mec_pid; 
	motor_pid_t            gyro_pid;
	motor_pid_t            position_pid;
	motor_pid_t            angle_pid;
	motor_pid_t            user_define_pid;
	
}motor_pid_all_t; 

void integral_to_zero(pid_ctrl_t *pid);
void single_pid_ctrl(pid_ctrl_t *pid);
float  all_pid_calc (pid_ctrl_t *out,pid_ctrl_t *inn,float target,float mea_out,float mea_in,float inner_kp,uint8_t err_cal_mode);
float feedforward_pid_calc(pid_ctrl_t *out,pid_ctrl_t *inn,float target,float mea_out,float mea_in,float inner_kp,uint8_t err_cal_mode);
#endif
