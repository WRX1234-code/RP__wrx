/**
 ******************************************************************************
 * @file    QuaternionEKF.h
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _BMI_EKF_H
#define _BMI_EKF_H
#include "kalman_filter.h"

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/* ����������任Ϊ��̨����ṹ�� */
typedef struct
{
    float arz;
    float ary;
    float arx;
    float trans[9];
} gimbal_transform_t;

typedef struct
{
    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    uint8_t ConvergeFlag;//�˲����Ƿ�������־����
    uint8_t StableFlag;//�˶�״̬�Ƿ��ȶ���־����
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float q[4];        // ��Ԫ������ֵ
    float GyroBias[3]; // ��������ƫ����ֵ

    float Gyro[3];
    float Accel[3];

    float OrientationCosine[3];//Ԥ��ֵ�͸�����ķ�������

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float Roll;
    float Pitch;
    float Yaw;

    float YawTotalAngle;

    float Q1; // ��Ԫ�����¹�������
    float Q2; // ��������ƫ��������
    float R;  // ���ٶȼ���������

    float dt; // ��̬��������
    mat ChiSquare;
    float ChiSquare_Data[1];      // ���������⺯��
    float ChiSquareTestThreshold; // ����������ֵ
    float lambda;                 // ��������

    int16_t YawRoundCount;

    float YawAngleLast;
} QEKF_INS_t;

extern gimbal_transform_t EKFgim_trans;
extern QEKF_INS_t QEKF_INS;
extern float chiSquare;
extern float ChiSquareTestThreshold;
void IMU_QuaternionEKF_Init(float* init_quaternion,float process_noise1, float process_noise2, float measure_noise, float lambda);
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void transform_init(gimbal_transform_t *gim_trans);
void Vector_Transform(float gx, float gy, float gz,\
	                  float ax, float ay, float az,\
	                  float *ggx, float *ggy, float *ggz,\
					  float *aax, float *aay, float *aaz);
void BMI_Get_Acceleration(float pitch, float roll, float yaw,\
						  float ax, float ay, float az,\
						  float *accx, float *accy, float *accz);

#endif
