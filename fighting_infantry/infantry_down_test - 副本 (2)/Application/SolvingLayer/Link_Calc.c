/**
  ******************************************************************************
  * File Name          : Link_Calc.c
  * Description        : Code for Link_Calc applications
  ******************************************************************************
  * @attention
  * Copyright (c) 2026 SZU RobotPilots.
  * @author 
  * Liang 741427745@qq.com
  ******************************************************************************
  * 
  ==============================================================================
                      ##### How To Use #####
  ==============================================================================
  (#) 调用初始化函数
		Link_Init
	  之后的步骤可以使用函数指针，此处以原函数名讲解
  
  (#) 调用外部数据获取API函数
		Link_Measure_data_update
		Link_Target_data_update
		
  (#) 调用连杆解算函数
		Link_Update
  
  (#) 在计算《驱动轮支持力》前调用，作用是获取沿杆方向力、虚拟腿关节力矩
		Fbl_and_Tp_Mea_Cal
  
  (#) 在计算《直腿模型控制量u》后调用，作用是将虚拟腿关节力矩、沿杆方向力
	  转化为真实关节力矩,F_Sd_Output_Torque,B_Sd_Output_Torque
		Joint_Torque_Cal
	
  (#) 
  */
#include "Link_Calc.h"

static void Link_Measure_data_update(Link_t* Link,float phi1,float phi1_d1,
										float phi4,float phi4_d1,float torque_phi1_mea,float torque_phi4_mea);
static void Link_Target_data_update(Link_t* Link,float F_bl_target,float Tp_target);
static void Link_Update(Link_t* Link);

static void Joint_Torque_Cal(Link_t* Link);
static void Fbl_and_Tp_Mea_Cal(Link_t* Link,float T1,float T2);
static void Link_Coordinate_Cal(Link_t* Link);
static void Link_Phi2_3_Cal(Link_t* Link);
static void Link_C_Cal(Link_t* Link);
static void Link_Leg_Length_Cal(Link_t* Link);
static void Link_phi0_Cal(Link_t* Link);
static void Link_Debug(Link_t* Link);
static void Link_Centroid_Coordinate_Cal(Link_t* Link);


/**
  * @brief  连杆初始化
  * @param  Link_Var_t* Link_Var
  * @retval None
  */
void Link_Init(Link_t* Link)
{
	Link->tar_data_update=Link_Target_data_update;
	Link->mea_data_update=Link_Measure_data_update;
	Link->torque_cal = Joint_Torque_Cal;
	
	Link->link_update = Link_Update;
	
	Link->Fb1_Tp_cal = Fbl_and_Tp_Mea_Cal;
	
	
}

/**
  * @brief  连杆数据更新
  * @param  Four_Bar_Link_t* My_Four_Bar_Link
  * @retval None
  */
static void Link_Update(Link_t* Link)
{
	/*更新A,B,D,E点的坐标*/
	Link_Coordinate_Cal(Link);
	
	/*更新phi2,phi3的值*/
	Link_Phi2_3_Cal(Link);
	
	/*更新C点坐标*/
	Link_C_Cal(Link);
	
	/*更新等效腿长信息*/
	Link_Leg_Length_Cal(Link);
	
	/*更新等效腿关节角度*/
	Link_phi0_Cal(Link);
	
	/*更新连杆质心坐标*/
	Link_Centroid_Coordinate_Cal(Link);
	
	Link_Debug(Link);
}

/**
  * @brief  外部测量值获取API
  * @param  Four_Bar_Link_t* My_Four_Bar_Link
  * @retval None
  */
static void Link_Measure_data_update(Link_t* Link,float phi1,float phi1_d1,
										float phi4,float phi4_d1,float torque_phi1_mea,float torque_phi4_mea)
{
	Link_info_t* info = Link->info;
	
	info->Ex_data->phi1=phi1;
	info->Ex_data->phi1_d1=phi1_d1;
	info->Ex_data->phi4=phi4;
	info->Ex_data->phi4_d1=phi4_d1;
	info->Ex_data->torque_phi1=torque_phi1_mea;
	info->Ex_data->torque_phi4=torque_phi4_mea;
	
	info->angle->phi1=info->Ex_data->phi1;
	info->angle->phi4=info->Ex_data->phi4;
	info->angle->phi1_d1=info->Ex_data->phi1_d1;
	info->angle->phi4_d1=info->Ex_data->phi4_d1;
	
	info->force->torque_phi1_mea=info->Ex_data->torque_phi1;
	info->force->torque_phi4_mea=info->Ex_data->torque_phi4;
}
/**
  * @brief  外部目标值获取API
  * @param  Four_Bar_Link_t* My_Four_Bar_Link
  * @retval None
  */
static void Link_Target_data_update(Link_t* Link,float F_bl_target,float Tp_target)
{
	Link_info_t* info = Link->info;
	
	info->force->F_bl_target=F_bl_target;
	info->force->Tp_target=Tp_target;
}

/**
  * @brief  连杆A,B,D,E点坐标计算
  * @param  Four_Bar_Link_t* My_Four_Bar_Link
  * @retval None
  */
static void Link_Coordinate_Cal(Link_t* Link)
{
	Link_info_t* info = Link->info;
	Link_Coord_t* coord = Link->info->coord;
	
	float phi1 = info->angle->phi1;
	float phi4 = info->angle->phi4;
	float phi1_d1 = info->angle->phi1_d1;
	float phi4_d1 = info->angle->phi4_d1;
	//A点坐标
	coord->xa = 0;
	coord->ya = 0;
	//E点坐标
	coord->xe = l5;
	coord->ye = 0;
  
	//B点坐标
  coord->xb = l1*arm_cos_f32(phi1);
  coord->yb = l1*arm_sin_f32(phi1);
	coord->xb_d1 = -l1*arm_sin_f32(phi1)*phi1_d1;
  coord->yb_d1 = l1*arm_cos_f32(phi1)*phi1_d1;
	
	//D点坐标
	coord->xd = l1*arm_cos_f32(phi4)+l5;
  coord->yd = l1*arm_sin_f32(phi4);
	coord->xd_d1 = -l1*arm_sin_f32(phi4)*phi4_d1;
  coord->yd_d1 = l1*arm_cos_f32(phi4)*phi4_d1;
}

/**
  * @brief  计算phi2,phi3
  * @param  Four_Bar_Link_t* My_Five_Bar_Link
  * @retval None
  */
static void Link_Phi2_3_Cal(Link_t* Link)
{
	Link_Leg_Length_t* Leg = Link->info->length;
	Link_Coord_t* coord = Link->info->coord;
	Link_Angle_t* angle = Link->info->angle;
	
	float xd = coord->xd;
	float yd = coord->yd;
	float xd_d1 = coord->xd_d1;
	float yd_d1 = coord->yd_d1;
	float xb = coord->xb;
	float yb = coord->yb;
	float xb_d1 = coord->xb_d1;
	float yb_d1 = coord->yb_d1;
	
	float temp_A0 = 2.f*l2 * (xd - xb);
	float temp_A0_d1 = 2.f*l2*(xd_d1 - xb_d1);
	float temp_B0 = 2.f*l2 * (yd - yb);
	float temp_B0_d1 = 2.f*l2*(yd_d1 - yb_d1);
	arm_sqrt_f32(powf(xd-xb,2.f) + powf(yd-yb,2.f),&Leg->lbd);
	float temp_C0 = Leg->lbd*Leg->lbd;
	float temp_C0_d1 = 2.f*(xd-xb)*(xd_d1-xb_d1)+2.f*(yd-yb)*(yd_d1-yb_d1);
	
	float temp_D0, temp_2phi2;
	arm_sqrt_f32(powf(temp_A0,2.f) + powf(temp_B0,2.f) - powf(temp_C0,2.f),&temp_D0);
	float temp_D0_d1 = ((temp_A0*temp_A0_d1)+(temp_B0*temp_B0_d1)-(temp_C0*temp_C0_d1))/temp_D0;
	arm_atan2_f32((temp_B0 + temp_D0), (temp_A0 + temp_C0), &temp_2phi2);
	
	angle->phi2 = temp_2phi2 * 2.f;
	angle->phi2_d1 = 2.f*((temp_A0+temp_C0)*(temp_B0_d1+temp_D0_d1)-(temp_A0_d1+temp_C0_d1)*(temp_B0+temp_D0))\
	                         /(powf(temp_A0+temp_C0, 2)+powf(temp_B0+temp_D0, 2));
	arm_atan2_f32((yb-yd) + l2*arm_sin_f32(angle->phi2), (xb-xd) + l2*arm_cos_f32(angle->phi2), &angle->phi3);
}

/**
  * @brief  计算C点坐标
  * @param  Five_Bar_Link_t* My_Five_Bar_Link
  * @retval None
  */
static void Link_C_Cal(Link_t* Link)
{
	Link_info_t* info = Link->info;
	float phi1, phi2, phi1_d1, phi2_d1;
	
	phi1 = info->angle->phi1;
	phi2 = info->angle->phi2;
	phi1_d1 = info->angle->phi1_d1;
	phi2_d1 = info->angle->phi2_d1;
	
	info->coord->xc = l1*arm_cos_f32(phi1) + l2*arm_cos_f32(phi2);
	info->coord->yc = l1*arm_sin_f32(phi1) + l2*arm_sin_f32(phi2);
	info->coord->xc_d1 = -l1*arm_sin_f32(phi1)*phi1_d1 - l2*arm_sin_f32(phi2)*phi2_d1;
	info->coord->yc_d1 = l1*arm_cos_f32(phi1)*phi1_d1 + l2*arm_cos_f32(phi2)*phi2_d1;
}


/**
  * @brief  计算等效腿长l0,l0_dot,l0_ddot
  * @param  Four_Bar_Link_t* My_Four_Bar_Link
  * @retval None
  */
static void Link_Leg_Length_Cal(Link_t* Link)
{
	Link_info_t* info = Link->info;
	float xc, yc, xc_d1, yc_d1;
	xc = info->coord->xc;
	yc = info->coord->yc;
	xc_d1 = info->coord->xc_d1;
	yc_d1 = info->coord->yc_d1;
	
	arm_sqrt_f32(xc*xc + yc*yc, &info->length->l0);
	info->length->good_l0_dot = -(xc*xc_d1+yc*yc_d1)/info->length->l0;
	info->length->l0 = Lowpass(info->length->l0_last, info->length->l0, 0.3f);
	
	
	info->length->l0_dot = Lowpass(info->length->l0_dot_last,((info->length->l0 - info->length->l0_last) / TIME_STEP), 0.25f);
	
	info->length->l0_last = info->length->l0;
	
	info->length->l0_dot2 = Lowpass(info->length->l0_dot2_last, (info->length->l0_dot - info->length->l0_dot_last) / TIME_STEP, 0.1f);
	
	info->length->l0_dot_last = info->length->l0_dot;
	
	info->length->l0_dot2_last = info->length->l0_dot2;
}

/**
  * @brief  计算等效腿关节角度phi0
  * @param  Four_Bar_Link_t* My_Four_Bar_Link
  * @retval None
  */
static void Link_phi0_Cal(Link_t* Link)
{
	Link_info_t* info = Link->info;
	float phi0 = 0.f, xc = 0.f, yc = 0.f, xc_d1, yc_d1,x_middle=0;
	xc = info->coord->xc;
	yc = info->coord->yc;
	xc_d1 = info->coord->xc_d1;
	yc_d1 = info->coord->yc_d1;
    x_middle=xc-l5/2;
	arm_atan2_f32(yc, x_middle, &phi0);
	
	info->angle->phi0 = phi0;
	info->angle->good_phi0_d1 = (yc_d1*xc-yc*xc_d1)/(xc*xc+yc*yc);
	info->angle->phi0_d1 = (phi0 - info->angle->phi0_last) / TIME_STEP;
	info->angle->phi0_last = phi0;
	
	/*将phi0准换为类似编码器的效果*/
	info->angle->vir_phi0 = (phi0 - 1.5708f);
	if(info->angle->vir_phi0 < -PI)
	{
		info->angle->vir_phi0 += PI * 2.f;
	}
	info->angle->vir_phi0_d1 = info->angle->good_phi0_d1;
}


/**
  * @brief  处理连杆数据增强可读性，数据仅用于调试
  * @param  Four_Bar_Link_t* My_Four_Bar_Link
  * @retval None
  */
static void Link_Debug(Link_t* Link)
{
	Link_info_t* info = Link->info;
	
	info->angle->phi0_ = info->angle->phi0 * Rad2Angle;
	
	info->angle->phi1_ = info->angle->phi1 * Rad2Angle;
	
	info->angle->phi2_ = info->angle->phi2 * Rad2Angle;
	
	info->angle->phi3_ = info->angle->phi3 * Rad2Angle;
	
	info->angle->phi4_ = info->angle->phi4 * Rad2Angle;
	
	info->angle->vir_phi0_ = info->angle->vir_phi0 * Rad2Angle;
}

/**
  * @brief  计算关节电机输出力矩
  * @param  Four_Bar_Link_t* My_Four_Bar_Link, float Fl, float Tp
  * @retval 关节输出力矩T1,T2
  */
static void Joint_Torque_Cal(Link_t* Link)
{
	float phi0 = 0.f, phi1 = 0.f, phi2 = 0.f, phi3 = 0.f,phi4 = 0.f, l0 = 0.f,Fbl=0,Tp=0;
  
  phi0 = Link->info->angle->phi0;
  
  phi1 = Link->info->angle->phi1;
  
  phi2 = Link->info->angle->phi2;
  
  phi3 = Link->info->angle->phi3;
	
  phi4= Link->info->angle->phi4;
  l0 = Link->info->length->l0;
  Tp =Link->info->force->Tp_target;
  Fbl =Link->info->force->F_bl_target;
	
  Link->info->F_Sd_Output_Torque =  (Fbl*l1*arm_sin_f32(phi0 - phi3)*arm_sin_f32(phi1 - phi2))/arm_sin_f32(phi3 - phi2) + (Tp*l1*arm_cos_f32(phi0 \
  - phi3)*arm_sin_f32(phi1 - phi2))/(l0*arm_sin_f32( phi3 - phi2)); 
	
  Link->info->B_Sd_Output_Torque =  (Fbl*l4*arm_sin_f32(phi0 - phi2)*arm_sin_f32(phi3 - phi4))/arm_sin_f32(phi3 - phi2) \
  + (Tp*l4*arm_cos_f32(phi0 - phi2)*arm_sin_f32(phi3 - phi4))/(l0*arm_sin_f32( phi3 - phi2));
	
}



/**
  * @brief  利用VMC逆矩阵，由电机反馈力矩计算五连杆输出竖直方向力与转矩
  * @retval None
  */
static void Fbl_and_Tp_Mea_Cal(Link_t* Link,float T1,float T2)
{
	float phi0 = 0.f, phi1 = 0.f, phi2 = 0.f, phi3 = 0.f, phi4 = 0.f, l0 = 0.f;
	
	
	phi0 = Link->info->angle->phi0;
  
	phi1 = Link->info->angle->phi1;
		
	phi2 = Link->info->angle->phi2;
  
	phi3 = Link->info->angle->phi3;
	
	phi4 = Link->info->angle->phi4;
  
	l0 = Link->info->length->l0;
	
Link->info->force->F_bl_mea = -(T1 * arm_cos_f32(phi0 - phi2)) / (l1 * arm_sin_f32(phi1 - phi2)) + 
                              (T2 * arm_cos_f32(phi0 - phi3)) / (l4 * arm_sin_f32(phi3 - phi4));

Link->info->force->Tp_mea = (T1 * l0 * arm_sin_f32(phi0 - phi2)) / (l1 * arm_sin_f32(phi1 - phi2)) - 
                           (T2 * l0 * arm_sin_f32(phi0 - phi3)) / (l4 * arm_sin_f32(phi3 - phi4));

}

/**
  * @brief  连杆各杆质心坐标计算，用于计算前馈补偿
  * @retval None
  */
static void Link_Centroid_Coordinate_Cal(Link_t* Link)
{
  
  float xa = 0.f, xb = 0.f, xc = 0.f, xd = 0.f, xe = 0.f;
  float ya = 0.f, yb = 0.f, yc = 0.f, yd = 0.f, ye = 0.f;
  
  xa = Link->info->coord->xa;  ya = Link->info->coord->ya;
  xb = Link->info->coord->xb;  yb = Link->info->coord->yb;
  xc = Link->info->coord->xc;  yc = Link->info->coord->yc;
  xd = Link->info->coord->xd;  yd = Link->info->coord->yd;
  xe = Link->info->coord->xe;  ye = Link->info->coord->ye;
  
  Link->info->centroid->mx_l1 = l1_cen*(xb - xa) + xa;
  Link->info->centroid->my_l1 = l1_cen*(yb - ya) + ya;
  Link->info->centroid->mx_l2 = l2_cen*(xc - xb) + xb;
  Link->info->centroid->my_l2 = l2_cen*(yc - yb) + yb;
  Link->info->centroid->mx_l3 = l3_cen*(xd - xc) + xc;
  Link->info->centroid->my_l3 = l3_cen*(yd - yc) + yc;
  Link->info->centroid->mx_l4 = l4_cen*(xe - xd) + xd;
  Link->info->centroid->my_l4 = l4_cen*(ye - yd) + yd;
  //向量合成
  Link->info->coord->xp = (Link->info->centroid->mx_l1*m_l1 + Link->info->centroid->mx_l2*m_l2 + Link->info->centroid->mx_l3*m_l3 + Link->info->centroid->mx_l4*m_l4) / (m_l1 + m_l2 + m_l3 + m_l4);
  Link->info->coord->yp = (Link->info->centroid->my_l1*m_l1 + Link->info->centroid->my_l2*m_l2 + Link->info->centroid->my_l3*m_l3 + Link->info->centroid->my_l4*m_l4) / (m_l1 + m_l2 + m_l3 + m_l4);
  //偏离C点的系数
  Link->info->centroid->centriod_coefficient = sqrt(pow((Link->info->coord->xc - Link->info->coord->xp), 2) + pow((Link->info->coord->yc - Link->info->coord->yp), 2)) \
                                   /pow( (pow((Link->info->coord->xc - l5/2.f), 2) + pow(Link->info->coord->yc, 2) ) , 0.5);
  //靠近C点的程度                            
  Link->info->centroid->centriod_coefficient = 1.f - Link->info->centroid->centriod_coefficient;
  
}

