#include "buzzer.h"
#include "stm32h7xx_hal.h"

// ========== 达妙妙板 STM32H723 蜂鸣器硬件定义 100%对应原理图 PB15 TIM12_CH2 ==========
#define BEEP_GPIO_PORT        GPIOB
#define BEEP_GPIO_PIN         GPIO_PIN_15
// 重点！！！你库原生真实存在的复用宏！！PB15 TIM12_CH2 复用AF1
// 不用库自带长宏，直接用数字AF编号，100%所有H7库全版本通用，永远不会未定义报错
#define BEEP_GPIO_AF          1
#define BEEP_TIM              TIM12
#define BEEP_TIM_CHANNEL      TIM_CHANNEL_2

// 定时器句柄 静态全局声明
static TIM_HandleTypeDef htim12;

void Buzzer_Init(void)
{
    // 1. 时钟使能：先开定时器时钟，再开GPIO端口时钟
    __HAL_RCC_TIM12_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // 2. PB15 复用功能GPIO初始化
    GPIO_InitTypeDef gpio_conf = {0};
    gpio_conf.Pin = BEEP_GPIO_PIN;
    gpio_conf.Mode = GPIO_MODE_AF_PP;        // 复用推挽输出
    gpio_conf.Pull = GPIO_NOPULL;
    gpio_conf.Speed = GPIO_SPEED_FREQ_LOW;
    // ==============================================
    // 【核心报错彻底解决】
    // 你这个版本H7库 结构体成员 就是 Alternate！不是AlternateFunction！
    // 直接填AF编号数字1，不依赖任何库宏，永远不会报“未定义标识符”
    // ==============================================
    gpio_conf.Alternate = BEEP_GPIO_AF;
    HAL_GPIO_Init(BEEP_GPIO_PORT, &gpio_conf);

    // 3. TIM12 PWM参数配置 精准4000Hz（官方原理图最佳频率）
    // 你板子固定时钟链路：
    // 系统时钟 550MHz  →  APB1总线时钟 137.5MHz  →  TIM12定时器时钟 = 137.5MHz
    htim12.Instance = BEEP_TIM;
    htim12.Init.Prescaler = 274;        // 预分频：137500000 / (274+1) = 500000Hz
    htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim12.Init.Period = 124;           // 重装载：500000 / (124+1) = 4000Hz 完美频率
    htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim12);

    // 4. PWM通道2配置 50%占空比（无源蜂鸣器声音最洪亮）
    TIM_OC_InitTypeDef oc_conf = {0};
    oc_conf.OCMode = TIM_OCMODE_PWM1;
    oc_conf.Pulse = 62;                 // 占空比 = 62 / 125 = 50%
    oc_conf.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc_conf.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim12, &oc_conf, BEEP_TIM_CHANNEL);
}

// ==================== 蜂鸣器控制函数 ====================
// 开启蜂鸣器（启动PWM，发声）
void Buzzer_On(void)
{
    HAL_TIM_PWM_Start(&htim12, BEEP_TIM_CHANNEL);
}

// 关闭蜂鸣器（停止PWM，完全静音）
void Buzzer_Off(void)
{
    HAL_TIM_PWM_Stop(&htim12, BEEP_TIM_CHANNEL);
}

// 比赛工程专用：单次短鸣提示音（复位、动作完成滴一声）
void Buzzer_ShortBeep(void)
{
    Buzzer_On();
    HAL_Delay(150);
    Buzzer_Off();
}


//if (in_air) {
//    float pitch = My_Chassis->Posture->info->pitch;
//    float roll  = My_Chassis->Posture->info->roll;
//    float yaw   = My_Chassis->Posture->info->yaw;
//    
//    float ax_b = imu_acc_x;
//    float ay_b = imu_acc_y;
//    float az_b = imu_acc_z;  // z向上为正，静止时≈-9.8
//    
//    // 机体→世界旋转
//    float cy = cos(yaw), sy = sin(yaw);
//    float cp = cos(pitch), sp = sin(pitch);
//    float cr = cos(roll), sr = sin(roll);
//    
//    float ax_w = (cy*cp)*ax_b + (-sy*cr+cy*sp*sr)*ay_b + (sy*sr+cy*sp*cr)*az_b;
//    float ay_w = (sy*cp)*ax_b + (cy*cr+sy*sp*sr)*ay_b + (-cy*sr+sy*sp*cr)*az_b;
//    float az_w = (-sp)*ax_b + (cp*sr)*ay_b + (cp*cr)*az_b;
//    
//    // 补偿重力（z向上为正，重力向下）
//    az_w += GRAVITY;
//    
//    // 世界坐标系积分
//    air_vel_x += ax_w * dt;
//    air_vel_y += ay_w * dt;
//    air_vel_z += az_w * dt;
//    
//    air_pos_x += air_vel_x * dt;
//    air_pos_y += air_vel_y * dt;
//    air_pos_z += air_vel_z * dt;
//}

//if (离地检测 && !in_air) {
//    in_air = true;
//    
//    // 保存世界坐标系速度（着地时卡尔曼滤波结果可靠）
//    takeoff_vel_x = My_filter_S_d1;  // 水平速度
//    takeoff_vel_z = 0;               // 离地瞬间垂直速度（水平地面）
//    
//    // 腾空积分初值
//    air_vel_x = takeoff_vel_x;
//    air_vel_y = 0;
//    air_vel_z = takeoff_vel_z;
//    
//    air_pos_x = My_filter_S;  // 离地水平位置
//    air_pos_y = 0;
//    air_pos_z = 0;  // 地面高度
//}










