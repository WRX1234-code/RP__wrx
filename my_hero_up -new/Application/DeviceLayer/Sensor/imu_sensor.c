/* Includes ------------------------------------------------------------------*/
#include "imu_sensor.h"
#include "drv_gpio.h"
#include "drv_tick.h"

#if IMU_USE_EKF == 1
#include "bmi_EKF.h"
#endif
#if IMU_USE_MAHONY == 1
#include "bmi.h"
#endif
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void imu_init(imu_sensor_t *self);
void imu_heart_beat(work_state_t *heart);
void imu_update(imu_sensor_t *self);
//void imu_set_temperature(imu_sensor_t *self, float temp);
#if IMU_USE_EKF == 1
static void InitQuaternion(float *init_q4);
static float Sqrt(float x);
#endif //IMU_USE_EKF
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


pid_ctrl_t imu_temp_pid = {
    .kp = 1000.0f,
    .ki = 0.0f,
    .out_max = 4500,
};

imu_info_t imu_info = 
{
	.offset_info.gx_offset = 0.f,
	.offset_info.gy_offset = 0.f,
	.offset_info.gz_offset = 0.f,
	.init_flag = 0,
};

/* Exported variables --------------------------------------------------------*/
imu_sensor_t imu_sensor = {

	.info = &imu_info,
	.driver.tpye = DR_SPI2,
	.work_state.dev_state = DEV_OFFLINE,
	.id = DEV_ID_IMU,	
	.work_state.cali_end = 0,	
	.work_state.offline_max_cnt = 50,	
	.work_state.err_cnt = 0,
    .temp_pid = &imu_temp_pid,
	
	.init = &imu_init,
	.update = &imu_update,
    .heart_beat = &imu_heart_beat,
//    .set_temperature = &imu_set_temperature,
};

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
float imu_read[3];
uint8_t init_cnt = 200;
/**
 * @brief  imu��ʼ��
 */
void imu_init(struct imu_struct *self)
{
	uint32_t tickstart = HAL_GetTick();

	self->work_state.dev_state = DEV_OFFLINE;
	self->work_state.init_code = BMI088_init();

	while(self->work_state.init_code)
	{
		if (++self->work_state.err_cnt == init_cnt)
		{
			__set_FAULTMASK(1); 
			NVIC_SystemReset();
			break;
		}
        self->work_state.err_code = IMU_INIT_ERR;
        self->work_state.init_code = BMI088_init();
	}	

	
	if (self->work_state.init_code == 0)
	{
		self->work_state.dev_state = DEV_ONLINE;
		
//		self->work_state.err_cnt = 0;
		self->info->init_flag = 1;
		
#if IMU_USE_MAHONY == 1
		/* Mahony��ʼ�� */
		transform_init(&gim_trans);
#endif //IMU_USE_MAHONY
		
#if IMU_USE_EKF == 1
		/* EKF��ʼ�� */
		transform_init(&EKFgim_trans);
		// float init_quaternion[4] = {0.999019921, -0.0315267481, -0.0310692526};
		float init_quaternion[4] = {0};
		InitQuaternion(init_quaternion);
		IMU_QuaternionEKF_Init(init_quaternion, 15, 0.001, 100000, 1);
#endif //IMU_USE_EKF
		
		self->work_state.err_code = IMU_NONE_ERR;
		imu_sensor.info->offset_info.gx_offset = 0.f;
		imu_sensor.info->offset_info.gy_offset = 0.f;
		imu_sensor.info->offset_info.gz_offset = 0.f;
	}
	else
	{
		self->work_state.dev_state = DEV_OFFLINE;
		self->work_state.err_code = IMU_INIT_ERR;
		self->work_state.offline_cnt = self->work_state.offline_max_cnt;
		self->info->init_flag = 0;
	}
	
}

/**
 * @brief  imuʧ�����
 */
void imu_heart_beat(work_state_t *heart)
{
	heart->offline_cnt++;
	if(heart->offline_cnt > heart->offline_max_cnt) 
	{
		heart->offline_cnt = heart->offline_max_cnt;
		heart->dev_state = DEV_OFFLINE;
	}
	else 
	{
		if(heart->dev_state == DEV_OFFLINE)
        {
            heart->dev_state = DEV_ONLINE;
        }
	}
}

/**
 * @brief  imu�����¶�
 */
//void imu_set_temperature(imu_sensor_t *self, float temp)
//{
//    self->temp_pid->err = temp - imu_info.base_info.temperature;
//    single_pid_ctrl(self->temp_pid);
//	/* �¶��쳣ֵ���� */
//    if(imu_info.base_info.temperature > 50 || imu_info.base_info.temperature < 0
//       || self->temp_pid->out < 0)
//    {
//        self->temp_pid->out = 0;
//    }

//    IMU_Set_PWM(self->temp_pid->out);
//}

ave_filter_t imu_pitch_dif_speed_ave_filter;
ave_filter_t imu_roll_dif_speed_ave_filter;
ave_filter_t imu_yaw_dif_speed_ave_filter;
/* ��ʱ���� */
static float pitch, roll, yaw;
static float gyrox, gyroy, gyroz;
static float accx, accy, accz;
static float gyrox_, gyroy_, gyroz_;
static float accx_, accy_, accz_;
static float gyro[3], accel[3], temp;
static int16_t imu_cnt = 0;
#if IMU_USE_EKF == 1
// ����ʱ��
static float imu_dt;
static uint32_t imu_tick_now, imu_tick_last;
#endif
void imu_update(imu_sensor_t *imu_sen)
{

    imu_info_t *imu_info = imu_sen->info;
	
	/* ��ȡ���������� */
	BMI088_read(gyro, accel, &temp);
	
	imu_info->raw_info.acc_x = accel[0];
	imu_info->raw_info.acc_y = accel[1];
	imu_info->raw_info.acc_z = accel[2];
	imu_info->raw_info.gyro_x = gyro[0];
	imu_info->raw_info.gyro_y = gyro[1];
	imu_info->raw_info.gyro_z = gyro[2];
	
	/* ����ϵ�任 */
	Vector_Transform(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2],\
	                 &gyrox, &gyroy, &gyroz, &accx, &accy, &accz);
	
	/* ������У�� */
	if (imu_sen->work_state.err_code == IMU_DATA_CALI)
	{
		if (imu_cnt < 2000)
		{
			imu_info->offset_info.gx_offset -= gyrox * 0.0005f;
			imu_info->offset_info.gy_offset -= gyroy * 0.0005f;
			imu_info->offset_info.gz_offset -= gyroz * 0.0005f;
			imu_cnt++;
		}
		else
		{
			imu_cnt = 0;
			imu_sen->work_state.err_code = IMU_NONE_ERR;
			
			if (abs(imu_info->offset_info.gx_offset) > 5.f)
				imu_info->offset_info.gx_offset = 0;
			if (abs(imu_info->offset_info.gy_offset) > 5.f)
				imu_info->offset_info.gy_offset = 0;
			if (abs(imu_info->offset_info.gz_offset) > 5.f)
				imu_info->offset_info.gz_offset = 0;
			imu_sen->work_state.cali_end = 1;
		}
	}
	else
	{
		gyrox += imu_info->offset_info.gx_offset;
		gyroy += imu_info->offset_info.gy_offset;
		gyroz += imu_info->offset_info.gz_offset;
	}
	
	/* ԭʼ���ݵ�ͨ�˲� */
	gyrox_ = Lowpass(gyrox_, gyrox, 1);
	gyroy_ = Lowpass(gyroy_, gyroy, 1);
	gyroz_ = Lowpass(gyroz_, gyroz, 1);
	accx_ = Lowpass(accx_, accx, 0.8);
	accy_ = Lowpass(accy_, accy, 0.2);
	accz_ = Lowpass(accz_, accz, 0.2);
	
	/* �������������� */
#if IMU_USE_MAHONY == 1
	BMI_Get_EulerAngle(&imu_info->base_info.pitch, &imu_info->base_info.roll, &imu_info->base_info.yaw,\
										 &gyrox_, &gyroy_, &gyroz_, \
										 &accx_, &accy_, &accz_);
#endif

#if IMU_USE_EKF == 1
	// TODO: ins �滻Ϊ imu
	// ����ʱ��
	imu_tick_now = micros();
	if(imu_tick_last == 0) // ��һ�����⴦��
	{
		imu_tick_last = imu_tick_now - 1000; // ���1ms
	}
	imu_dt = (imu_tick_now - imu_tick_last) * 0.000001f; // us to s
	imu_tick_last = imu_tick_now;

    // ���ĺ���,EKF������Ԫ��
    IMU_QuaternionEKF_Update(gyrox, gyroy, gyroz, accx, accy, accz, imu_dt);

		imu_info->base_info.yaw = QEKF_INS.Yaw;
    imu_info->base_info.pitch = QEKF_INS.Pitch;
    imu_info->base_info.roll = QEKF_INS.Roll;
    imu_info->base_info.yaw_total_angle = QEKF_INS.YawTotalAngle;
#endif
	/* ��ȡ��������ϵ�ļ��ٶ� */						 
	pitch = imu_info->base_info.pitch, roll = imu_info->base_info.roll, yaw = imu_info->base_info.yaw;
	BMI_Get_Acceleration(pitch, roll, yaw,\
											 accx_, accy_, accz_,\
											 &imu_info->base_info.accx, &imu_info->base_info.accy, &imu_info->base_info.accz);
	
	/* �������������� */
	//pitch
	imu_info->base_info.rate_pitch = gyroy_ / (double)0.017453;
	imu_info->base_info.ave_rate_pitch = ave_fil_update(&imu_pitch_dif_speed_ave_filter, imu_info->base_info.rate_pitch, 3);
	
	//roll
	imu_info->base_info.rate_roll = gyrox_ / (double)0.017453;
	imu_info->base_info.ave_rate_roll = ave_fil_update(&imu_roll_dif_speed_ave_filter, imu_info->base_info.rate_roll, 3);
	
	//yaw
	imu_info->base_info.rate_yaw = gyroz_ / (double)0.017453;
	imu_info->base_info.ave_rate_yaw = ave_fil_update(&imu_yaw_dif_speed_ave_filter, imu_info->base_info.rate_yaw, 3);
	
	imu_sen->work_state.offline_cnt = 0;
	
	/* imu��ȡ�����ж�  */
	if ((accel[0] == 0) && (accel[1] == 0) && (accel[2] == 0) \
		 && (gyro[0] == 0) && (gyro[1] == 0) && (gyro[2]== 0))
	{
		if(++imu_sen->work_state.err_cnt >= 100)
		{
			imu_sen->work_state.dev_state = DEV_OFFLINE;
			imu_sen->work_state.err_code = IMU_DATA_ERR;
			imu_sen->work_state.offline_cnt = imu_sen->work_state.offline_max_cnt;
			imu_sen->work_state.err_cnt = 100;
		}
	}

    /* imu��ȡ�¶� */
    imu_info->base_info.temperature = temp;

}

#if IMU_USE_EKF == 1
// ʹ�ü��ٶȼƵ����ݳ�ʼ��Roll��Pitch,��Yaw��0,�������Ա����ڳ�ʼʱ�����̬�������
static void InitQuaternion(float *init_q4)
{
    float acc_sum[3] = {0};
	float gyro[3], acc_init[3], temp;

    // ��ȡ100�μ��ٶȼ�����,ȡƽ��ֵ��Ϊ��ʼֵ
    for (uint8_t i = 0; i < 100; ++i)
    {
        BMI088_read(gyro, acc_init, &temp);
        acc_sum[0] += acc_init[0];
        acc_sum[1] += acc_init[1];
        acc_sum[2] += acc_init[2];
        delay_ms(1);
    }
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] = acc_sum[i]/100;
		float pitch = atan2(-acc_init[0], Sqrt(acc_init[1] * acc_init[1] + acc_init[2] * acc_init[2]));
		float roll = atan2(acc_init[1], acc_init[2]);
		
		float half_pitch = pitch / 2.0f;
    float half_roll = roll / 2.0f;

    init_q4[0] = cos(half_pitch) * cos(half_roll);
    init_q4[1] = sin(half_pitch) * cos(half_roll);
    init_q4[2] = cos(half_pitch) * sin(half_roll);
    init_q4[3] = sin(half_pitch) * sin(half_roll);
}

// ���ٿ���
static float Sqrt(float x)
{
    float y;
    float delta;
    float maxError;

    if (x <= 0)
    {
        return 0;
    }

    // initial guess
    y = x / 2;

    // refine
    maxError = x * 0.001f;

    do
    {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}


#endif //IMU_USE_EKF
