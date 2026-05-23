///* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- 
// * @Files: cap_board.c/h
// * 
// * @Author: Ye Jinyi
// * 
// * @First  Edit Date: 2024.2.23
// * @Latest Edit Date: 
// * 
// * @illustrate: 
// *              
// * 
// * @attention: 将整个capboard结构体与两个通信结构体分离处理，因为通信结构体不需字节对齐，
// *             如果放进capboard结构体会使整个大结构体字节不对齐，降低访问速度
// *             
// *             好像生成不了占空比为100%的方波，所以CMP1xR比较值有最小限制、CMP3xR有最大限制，不然就会输出低电平
// *             
// *-------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

///* Including Files  ------------------------------------------------------------------------------------------------------------------------------------------------- */
//#include "WL_RX_board.h"
//#include "driver.h"
//#include "arm_math.h"
//#include <string.h>
//#include "can_protocol.h"
//#include "rp_math.h"

//#ifdef _I_NEED_TO_PRINT_INFO_

//// 调试用
//#include "DWT.h"

//#endif

///* External Variables Declarations ---------------------------------------------------------------------------------------------------------------------------------- */
//extern HRTIM_HandleTypeDef hhrtim1;

///* Define Exported Variables ---------------------------------------------------------------------------------------------------------------------------------------- */
///* 创建电容板实例 */
//WL_RX_board_t WL_RX_board = {
//    
//    /* 电压电流信息部分 */
//    .dma_adc_raw_data = {0},
//    .adc_filter_info = {0},
//    .adc_cali_info = {0},
//    
//    /* 电容板状态量 */
//    .WL_RX_board_state = {
//        .total_state = IN_ERROR,
//        .init_state = NOT,
//        .ability_state = NOTABLE,
//        .using_state = IDLE,
//        
//        .power_supply_state = UNDER,
//        .cap_voltage_state  = UNDER,
//        .cap_current_state  = UNDER,
//        
//        .code_match_state = MISMATCHED,
//        .communicate_state = MISSED,
//    },
//    
//    /* pid参数赋值 */
//    .pid_power  = {
//        .Kp = 0.003f,       // 0.003
//        .Ki = 0.0004f,      // 0.0004
//        .Kd = 0.f,
//        .out_min = CAP_I_MIN_LIMIT,
//        .out_max = CAP_I_MAX_LIMIT,
//    },
//#if (_CALIBRATE_CAP_I_ == 1U)
//    .pid_cap_i  = {
//    
//        .Kp = 0.f,
//        .Ki = 0.f,
//        .Kd = 0.f,
//        .out_min = 0.1f,
//        .out_max = 10.f,
//    },
//#else
//    .pid_cap_i  = {
//    
//        .Kp = 0.004f,       // 0.004
//        .Ki = 0.001f,       // 0.001
//        .Kd = 0.f,
//        .out_min = 0.1f,
//        .out_max = 10.f,
//    },
//#endif
//    /* 实际输出量初值 */
//    .final_TIMERB_cmp1 = HRTIM_PERIOD_0_5,
//    .final_TIMERB_cmp3 = HRTIM_PERIOD_0_5,
//    .final_TIMERD_cmp1 = HRTIM_PERIOD_0_5,
//    .final_TIMERD_cmp3 = HRTIM_PERIOD_0_5,
//    
//};

///* 实例化通信相关结构体 */
//wireless_tx_info_t WL_RX_board_tx_info;
//#if (_ON_CAR_USING_ == 1U)

//WL_RX_board_rx_info_t WL_RX_board_rx_info;

//#else

//// 调试用
//WL_RX_board_rx_info_t WL_RX_board_rx_info = {
//    
//    .chassis_power_buffer = 60.f,          // 底盘能量缓冲（在buffer_pid_ctrl中被使用）
//    .chassis_power_limit  = 80.f,          // 机器人底盘功率限制上限（在power_pid_ctrl中被使用）
//    .cap_power_out_limit  = -120.f,        // 电容放电功率限制（在cap_i_pid_ctrl中被使用）
//    .cap_power_in_limit   = 120.f,         // 电容充电功率限制（在cap_i_pid_ctrl中被使用）
//    
//    .bit_control = {
//        .cap_switch = 1,
//        .turbo_mode = 0,
//    },
//        
//};

//#endif




///* Define Privated Variables ---------------------------------------------------------------------------------------------------------------------------------------- */
//static uint16_t miss_master_cnt;           // 和主机通信丢失的计数值

///* Define Privated Functions ---------------------------------------------------------------------------------------------------------------------------------------- */
///*!
// * @brief: 低通滤波，每个量只取1个(最后1个PWM周期)数据
// *         
// */
//void data_filter_convert(uint16_t origin[MEASURE_NUM], float filtered[MEASURE_NUM])
//{
//    /* 上次的值 */
//    static float last_data[MEASURE_NUM];
//    
//    /* 这次的值 */
//    static float now_data[MEASURE_NUM];
//    
//    /* 低通滤波系数 */
//    const static float alpha = 0.3f;
//    
//    uint8_t i;
//    for (i = 0;i < MEASURE_NUM;i++)
//    {
//        now_data[i] = (float)origin[i];
//        now_data[i] = alpha * now_data[i] + (1.f - alpha) * last_data[i];
//        last_data[i] = now_data[i];
//    }
//    
//    
//    /* 由硬件电路得出 */
//    /* 电压量 = 原始数据 / 范围 * 参考电压 * 运放增益的倒数 */
//    /* filtered[V] = data / ADC_SCALE * CHIP_V_REF * 20.f; */
//    filtered[BAT_V] = (now_data[BAT_V ] / ADC_SCALE) * CHIP_V_REF * 20.f;
//    filtered[CAP_V] = (now_data[CAP_V ] / ADC_SCALE) * CHIP_V_REF * 20.f;
//    
//    /* 电流量 = （原始数据 / 范围 * 参考电压 - 参考电压的一半） / INA240增益 / 采样电阻大小 */
//    /* filtered[C] = (data / ADC_SCALE * CHIP_V_REF - INA_V_REF / 2.f) / 50.f / 0.001f; */
//    filtered[CAP_I]  = (now_data[CAP_I ] / ADC_SCALE * CHIP_V_REF - INA_V_REF / 2.f) * 20.f;
//    filtered[BAT_I]  = (now_data[BAT_I ] / ADC_SCALE * CHIP_V_REF - INA_V_REF / 2.f) * 20.f;
//    
//}

///*!
// * @usage: 校准原始数据
// */
//static inline void info_calibrate(WL_RX_board_t *handle)
//{
//    /* y = Ax + B */
//    handle->adc_cali_info[BAT_I ] = handle->adc_filter_info[BAT_I ] * BAT_I_COEFF_A  + BAT_I_COEFF_B ;
//    handle->adc_cali_info[CAP_I ] = handle->adc_filter_info[CAP_I ] * CAP_I_COEFF_A  + CAP_I_COEFF_B ;
//    
//    /* z = Ax + By */
//    handle->adc_cali_info[BAT_V ] = handle->adc_filter_info[BAT_V ] * BAT_V_COEFF_A + \
//                                    handle->adc_cali_info[BAT_I ] * BAT_V_COEFF_B ;
//    
//    /* z = Ax + B - ESRy */
//    handle->adc_cali_info[CAP_V ] = handle->adc_filter_info[CAP_V ] * CAP_V_COEFF_A  + CAP_V_COEFF_B \
//                                    - handle->adc_cali_info[CAP_I ] * CAP_ESR;
//    
//}

///*!
// * @usage: 计算相关数据
// */
//static inline void info_calculate(WL_RX_board_t *handle)
//{
//    /* 定义临时变量 */
//    float now_BAT_V  = handle->adc_cali_info[BAT_V];
//    float now_BAT_I  = handle->adc_cali_info[BAT_I];
//    float now_CAP_V  = handle->adc_cali_info[CAP_V];
//    float now_CAP_I  = handle->adc_cali_info[CAP_I];
//    
//    handle->bat_v = now_BAT_V;
//    handle->bat_i = now_BAT_I;
//    handle->cap_v = now_CAP_V;
//    handle->cap_i = now_CAP_I;

//    /* 更新当前电源输入功率 */
//    handle->input_power = now_BAT_V * now_BAT_I;
//    
//    /* 更新当前电容组功率 */
//    handle->cap_power = now_CAP_V * now_CAP_I;
//    
//    /* 更新当前电压比率 */
//    handle->now_volt_ratio = now_CAP_V / now_BAT_V;
//    
//    /* 更新当前电量百分比 */
//    handle->percentage_energy = (now_CAP_V / CAP_V_MAX_LIMIT) * 100.f;
//    
//}

///*!
// * @usage: 计算pwm波占空比
// * @notice: volt_ratio = V_out / V_in
// *          根据V_out / V_in = D_bu / D_bo （左上管占空比 / 右上管占空比）
// *          且D < 0.95, 可以算出放大倍数范围为0.05~19倍
// */
//static inline void pwm_duty_calculate(WL_RX_board_t *handle)
//{
//    float volt_ratio = handle->final_out_volt_ratio;
//    float buck_duty, boost_duty;
//    
//    /* 限幅，不取极限值 */
//    volt_ratio = constrain(volt_ratio, 0.1f, 10.f);
//    
//    /* 根据电压比值对应电路模式，使用迟滞比较来防止状态来回切换导致震荡 */
//    if (volt_ratio < 0.8f)
//    {
//        handle->WL_RX_board_state.pwm_state = BUCK;
//    }
//    else if (volt_ratio > 1.25f)
//    {
//        handle->WL_RX_board_state.pwm_state = BOOST;
//    }
//    else if (volt_ratio > 0.9f && volt_ratio < 1.111f)
//    {
//        handle->WL_RX_board_state.pwm_state = BUCK_BOOST;
//    }
//    
//    /* 根据电路模式计算占空比 */
//    if (handle->WL_RX_board_state.pwm_state == BUCK)
//    {
//        buck_duty = 0.9f * volt_ratio;
//        boost_duty = 0.9f;
//    }
//    else if (handle->WL_RX_board_state.pwm_state == BOOST)
//    {
//        buck_duty = 0.9f;
//        boost_duty = 0.9f / volt_ratio;
//    }
//    else if (handle->WL_RX_board_state.pwm_state == BUCK_BOOST)
//    {
//        buck_duty = (volt_ratio + 1.f) * 0.4f;               // 这样做是为了平滑过渡
//        boost_duty = (1.f / volt_ratio + 1.f) * 0.4f;        // 系数依据边界值来计算
//    }
//    
//    /* 生成中心对称的PWM波 */
//    handle->final_TIMERB_cmp1 = HRTIM_PERIOD_0_5 * (1.f - buck_duty) ;
//    handle->final_TIMERB_cmp3 = HRTIM_PERIOD_0_5 * (1.f + buck_duty) ;
//    handle->final_TIMERD_cmp1 = HRTIM_PERIOD_0_5 * (1.f - boost_duty);
//    handle->final_TIMERD_cmp3 = HRTIM_PERIOD_0_5 * (1.f + boost_duty);
//    
//    handle->final_TIMERB_cmp1 = constrain(handle->final_TIMERB_cmp1, MIN_CMP_VALUE, MAX_CMP_VALUE);
//    handle->final_TIMERB_cmp3 = constrain(handle->final_TIMERB_cmp3, MIN_CMP_VALUE, MAX_CMP_VALUE);
//    handle->final_TIMERD_cmp1 = constrain(handle->final_TIMERD_cmp1, MIN_CMP_VALUE, MAX_CMP_VALUE);
//    handle->final_TIMERD_cmp3 = constrain(handle->final_TIMERD_cmp3, MIN_CMP_VALUE, MAX_CMP_VALUE);
//    
//}

///*!
// * @usage: 更新pwm波，实现pwm波占空比计算和pwm波更新的分离，方便实现保护与调试
// * @notice: 
// */
//static inline void pwm_update(WL_RX_board_t *handle)
//{
//    /* 进入临界区（保存状态并禁用中断） */
//    uint32_t primask = __get_PRIMASK();         // 保存当前中断状态
//    __disable_irq();                            // 屏蔽所有中断
//    
//    hhrtim1.Instance->sTimerxRegs[HR_TIMERB_IDX].CMP1xR = handle->final_TIMERB_cmp1;
//    hhrtim1.Instance->sTimerxRegs[HR_TIMERB_IDX].CMP3xR = handle->final_TIMERB_cmp3;
//    hhrtim1.Instance->sTimerxRegs[HR_TIMERD_IDX].CMP1xR = handle->final_TIMERD_cmp1;
//    hhrtim1.Instance->sTimerxRegs[HR_TIMERD_IDX].CMP3xR = handle->final_TIMERD_cmp3;
//    
//    /* 退出临界区（恢复原始中断状态） */
//    __set_PRIMASK(primask);  
//    
//}

///*!
// * @usage: 输入功率环
// * @brief: 第一部分判断是否存在重新初始化，第二部分进入计算
// */
//static inline void power_pid_ctrl(WL_RX_board_t *handle, float target_P)
//{
//    /* 上次的初始化状态 */
//    static init_state_e last_init_state = NOT;
//    
//    /* 每次初始化时都要给该增量式pid的输出赋0 */
//    if (handle->WL_RX_board_state.init_state == DONE \
//        && last_init_state == NOT)
//    {
//        /* 基本对应电流为0的情况 */
//        handle->pid_power.out = 0.f;
//        handle->pid_power.last_out = 0.f; 
//    }
//    last_init_state = handle->WL_RX_board_state.init_state;
//    
//    
//    handle->pid_power.target  = target_P;
//    handle->pid_power.measure = handle->input_power;
//    
//    incremental_pid_ctrl(&handle->pid_power);
//}

///*!
// * @usage: 电容电流环
// * @brief: 第一部分判断是否存在重新初始化，第二部分限幅，第三部分进入计算
// */
//static inline void cap_i_pid_ctrl(WL_RX_board_t *handle, float target_I)
//{
//    /* 上次的初始化状态 */
//    static init_state_e last_init_state = NOT;
//    
//    /* 每次初始化时都要给该增量式pid的输出赋一个初值 */
//    if (handle->WL_RX_board_state.init_state == DONE \
//        && last_init_state == NOT)
//    {
//        /* capbrd->now_volt_ratio即对应电流为0的情况 */
//        handle->pid_cap_i.out = handle->now_volt_ratio;
//        handle->pid_cap_i.last_out = handle->now_volt_ratio;
//    }
//    last_init_state = handle->WL_RX_board_state.init_state;
//    
//    
//    /* 电流值动态限幅变量 */
//    static float min_CAP_I, max_CAP_I;
//    float cap_v;
//    
//    /* 三级限幅，根据主机允许的电容组最大充放电功率来限制 */
//    cap_v = fmaxf(handle->adc_cali_info[CAP_V], 0.01f);             // 防止除以0出现错误
//    min_CAP_I = WL_RX_board_rx_info.cap_power_out_limit / cap_v;       // 提示：不规范的全局变量引用
//    max_CAP_I = WL_RX_board_rx_info.cap_power_in_limit  / cap_v;       // 提示：不规范的全局变量引用
//    
//    /* 二级限幅，根据硬指标来限制 */
//    min_CAP_I = (min_CAP_I < CAP_I_MIN_LIMIT) ? CAP_I_MIN_LIMIT : min_CAP_I;
//    max_CAP_I = (max_CAP_I > CAP_I_MAX_LIMIT) ? CAP_I_MAX_LIMIT : max_CAP_I;
//    
//    /* 一级限幅，当电容低压时只允许充电，当电容满电时只允许放电 */
//    if (WL_RX_board.percentage_energy < BOTTOM_ABILITY_PERCENTAGE_ENERGY)
//    {
//        min_CAP_I = 0.f;
//    }
//    if (WL_RX_board.percentage_energy > 98.f)
//    {
//        max_CAP_I = 0.f;
//    }
//    
//    
//    /* 计算目标值 */
//    handle->pid_cap_i.target  = constrain(target_I, min_CAP_I, max_CAP_I);
//    handle->pid_cap_i.measure = handle->adc_cali_info[CAP_I];
//    
//    incremental_pid_ctrl(&handle->pid_cap_i);
//    
//}

///*!
// * @usage: 上电软启动时间状态机
// */
//static inline float soft_start(WL_RX_board_t *handle, float target)
//{
//    /* 软启动记录变量 */
//    static uint8_t soft_start_state = 0;        // 0表示未处于软启动状态，1表示处于软启动状态
//    static float soft_start_cnt = 0.f;
//    
//    
//    /* 上次的初始化状态 */
//    static init_state_e last_init_state = NOT;
//    
//    /* 每次初始化时都要软启动 */
//    if (handle->WL_RX_board_state.init_state == DONE \
//        && last_init_state == NOT)
//    {
//        soft_start_state = 1;
//    }
//    last_init_state = handle->WL_RX_board_state.init_state;
//    
//    
//    /* 设定1000ms内到达目标值，逐步逼近 */
//    if (soft_start_state == 1)
//    {
//        
//        target = target * (soft_start_cnt / 33333.f);
//        
//        soft_start_cnt += 1.f;
//        
//        if (soft_start_cnt > 33333.f)
//        {
//            soft_start_state = 0;
//            soft_start_cnt = 0;
//        }
//        
//    }
//    
//    return target;
//}

///*!
// * @usage: 各环的汇总控制
// * @brief: 上一环的out（可能经过处理）是下一环的target
// *         上电软启动
// *         快充满电时不满足功率环（如果有剩余功率充电，功率环的输出正常情况下会累加到最大，但是在此处会被限制）
// *         只限充电电流，放电不受影响
// */

//float initial_target_P;
//float final_target_P;
//float initial_target_I;
//float final_target_I;

//static inline void integrate_pid_control(WL_RX_board_t *handle)
//{
//#if (_CALIBRATE_CAP_I_ == 0U)
//    initial_target_P = WL_RX_board_rx_info.chassis_power_limit;
//    
//    /* 输入功率环 */
//    /* 上电软启动 */
//    final_target_P = soft_start(handle, initial_target_P);
//    
//    power_pid_ctrl(handle, final_target_P);
//    initial_target_I = handle->pid_power.out;
//#endif
//    
//    /* 电容电流环 */
//    /* 快充满电时减小充电电流 */
//    if (handle->percentage_energy > CLOSE_FULL_PERCENTAGE_ENERGY && initial_target_I > 0)
//    {
//        /* 以三次函数的形式缓近0 */
//        float coeff;
//        coeff = 10.f - (handle->percentage_energy / 10.f);
//        final_target_I = (coeff * coeff * coeff) * initial_target_I;
//    }
//    else
//    {
//        final_target_I = initial_target_I;
//    }
//    
//    cap_i_pid_ctrl(handle, final_target_I);
//    
//    
//    /* 最终输出 */
//    WL_RX_board.final_out_volt_ratio = WL_RX_board.pid_cap_i.out;
//    
//}

///*!
// * @usage: 状态检查、状态量更新，多使用迟滞比较防止状态频繁切换
// */
//static inline void check_states(WL_RX_board_t *handle)
//{
//    /* 定义临时变量 */
//    float now_BAT_V, now_CAP_V, now_CAP_I;
//    
//    now_BAT_V = handle->adc_cali_info[BAT_V];
//    now_CAP_V = handle->adc_cali_info[CAP_V];
//    now_CAP_I = handle->adc_cali_info[CAP_I];
//    
//    /* 初始化状态机 */
//    static uint16_t init_cnt = 0;   // 重新初始化计数值
//    switch (handle->WL_RX_board_state.init_state)
//    {
//        case NOT:                   // 需要重新初始化
//        {
//            /* 电源输入正常且主机令电容板开启达到10000个计数值就初始化完成 */
//            if (handle->WL_RX_board_state.power_supply_state == NORMAL \
//                && WL_RX_board_rx_info.bit_control.cap_switch == 1)
//            {
//                init_cnt++;
//                if (init_cnt > 10000)
//                {
//                    handle->WL_RX_board_state.init_state = DONE;
//                    
//                    /* 上电先输出使电流大概为0的pwm波 */
//                    handle->final_out_volt_ratio = handle->now_volt_ratio;
//                    /* pwm_calculate中有迟滞，在特定区间无法去到正确模式，所以手动给一下 */
//                    if (handle->now_volt_ratio > 0.8f && handle->now_volt_ratio < 1.125f)
//                    {
//                        handle->WL_RX_board_state.pwm_state = BUCK_BOOST;
//                    }
//                    
//                    init_cnt = 0;
//                }
//                
//            }
//            else
//            {
//                init_cnt = 0;
//            }
//            break;
//        }
//        case DONE:                  // 已初始化完成
//        {
//        
//        /* 如果上车，需要结合通信状态 */
//#if (_ON_CAR_USING_ == 1U)
//            /* 电容电流不正常、电源不正常、通信断联、主机令电容板关闭就重新需要初始化 */
//            if (handle->WL_RX_board_state.cap_current_state != NORMAL \
//                || handle->WL_RX_board_state.power_supply_state != NORMAL \
//                || handle->WL_RX_board_state.code_match_state != MATCHED \
//                || handle->WL_RX_board_state.communicate_state != CONNECTED \
//                || WL_RX_board_rx_info.bit_control.cap_switch != 1)
//            {
//                handle->WL_RX_board_state.init_state = NOT;
//                init_cnt = 0;
//            }
//#else 
//            /* 电容电流不正常、电源不正常、主机令电容板关闭就重新需要初始化 */
//            if (handle->WL_RX_board_state.cap_current_state != NORMAL \
//                || handle->WL_RX_board_state.power_supply_state != NORMAL \
//                || handle->WL_RX_board_state.code_match_state != MATCHED \
//                || WL_RX_board_rx_info.bit_control.cap_switch != 1)
//            {
//                handle->WL_RX_board_state.init_state = NOT;
//                init_cnt = 0;
//            }
//#endif
//    
//            break;
//        }
//        default:
//            break;
//    }
//    
//    /* 更新当前电容组可放电状态 */
//    if (handle->percentage_energy > MIN_ABILITY_PERCENTAGE_ENERGY)
//    {
//        handle->WL_RX_board_state.ability_state = ABLE;
//    }
//    else if (handle->percentage_energy < BOTTOM_ABILITY_PERCENTAGE_ENERGY)
//    {
//        handle->WL_RX_board_state.ability_state = NOTABLE;
//    }
//    
//    /* 更新电容组使用状态 */
//    if (fabsf(now_CAP_I) < 0.1f)
//    {
//        handle->WL_RX_board_state.using_state = IDLE;
//    }
//    else
//    {
//        handle->WL_RX_board_state.using_state = (now_CAP_I > 0.f) ? CHARGING : DISCHARGING;
//    }
//    
//    /* 电源输入状态机 */
//    static uint32_t BAT_V_over_cnt = 0;     // 母线过压计数值
//    static uint32_t BAT_V_under_cnt = 0;    // 母线欠压计数值
//    switch (handle->WL_RX_board_state.power_supply_state)
//    {
//        case UNDER:                         //欠压
//        {
//            if (now_BAT_V > BAT_INDUCTION_V)
//            {
//                handle->WL_RX_board_state.power_supply_state = NORMAL;
//            }
//            break;
//        }
//        case OVER:                          //过压
//        {
//            if (now_BAT_V < BAT_V_MAX_LIMIT)
//            {
//                handle->WL_RX_board_state.power_supply_state = NORMAL;
//            }
//            break;
//        }
//        case NORMAL:                        //正常
//        {
//            if (now_BAT_V < BAT_WORK_V)
//            {
//                BAT_V_under_cnt++;
//                if (BAT_V_under_cnt > 10000)  // 30us加一次的情况下是300ms
//                {
//                    handle->WL_RX_board_state.power_supply_state = UNDER;
//                    BAT_V_under_cnt = 0;
//                }
//            }
//            else if (now_BAT_V > BAT_V_MAX_LIMIT + 2.f)
//            {
//                BAT_V_over_cnt++;
//                if (BAT_V_over_cnt > 100000)  // 30us加一次的情况下是3s
//                {
//                    handle->WL_RX_board_state.power_supply_state = OVER;
//                    BAT_V_over_cnt = 0;
//                }
//            }
//            break;
//        }
//        default:
//            break;
//    }
//    
//    /* 电容组电压状态机 */
//    switch (handle->WL_RX_board_state.cap_voltage_state)
//    {
//        case UNDER:             // 欠压
//        {
//            if (now_CAP_V > CAP_V_MIN_LIMIT)
//            {
//                handle->WL_RX_board_state.cap_voltage_state = NORMAL;
//            }
//            break;
//        }
//        case OVER:              // 过压
//        {
//            if (now_CAP_V < CAP_V_MAX_LIMIT)
//            {
//                handle->WL_RX_board_state.cap_voltage_state = NORMAL;
//            }
//            break;
//        }
//        case NORMAL:            //正常
//        {
//            if (now_CAP_V < CAP_V_MIN_LIMIT - 0.5f)
//            {
//                handle->WL_RX_board_state.cap_voltage_state = UNDER;
//            }
//            else if (now_CAP_V > CAP_V_MAX_LIMIT + 1.f)
//            {
//                handle->WL_RX_board_state.cap_voltage_state = OVER;
//            }
//            break;
//        }
//        default:
//            break;
//    }
//    
//    /* 电容组电流状态机 */
//    static uint32_t CAP_I_over_cnt = 0;      // 电容充电电流过大计数值
//    static uint32_t CAP_I_under_cnt = 0;     // 电容放电电流过大计数值
//    switch (handle->WL_RX_board_state.cap_current_state)
//    {
//        case UNDER:             // 放电电流过大
//        {
//            if (now_CAP_I > CAP_I_MIN_LIMIT)
//            {
//                handle->WL_RX_board_state.cap_current_state = NORMAL;
//            }
//            break;
//        }
//        case OVER:              // 充电电流过大
//        {
//            if (now_CAP_I < CAP_I_MAX_LIMIT)
//            {
//                handle->WL_RX_board_state.cap_current_state = NORMAL;
//            }
//            break;
//        }
//        case NORMAL:            //正常
//        {
//            if (now_CAP_I < CAP_I_MIN_LIMIT - 1.f)
//            {
//                CAP_I_under_cnt++;
//                if (CAP_I_under_cnt > 10)        // 30us加一次的情况下是0.3ms
//                {
//                    handle->WL_RX_board_state.cap_current_state = UNDER;
//                    CAP_I_under_cnt = 0;
//                }
//            }
//            else if (now_CAP_I > CAP_I_MAX_LIMIT + 1.f)
//            {
//                CAP_I_over_cnt++;
//                if (CAP_I_over_cnt > 10)        // 30us加一次的情况下是0.3ms
//                {
//                    handle->WL_RX_board_state.cap_current_state = OVER;
//                    CAP_I_over_cnt = 0;
//                }
//            }
//            break;
//        }
//        default:
//            break;
//    }
//    
//    /* 更新与主机的通信状态 */
//    /* 接收到数据将在can数据处理函数中使communicate_state = CONNECTED */
//    miss_master_cnt++;
//    if (miss_master_cnt > 33333)        // 30us加一次的情况下是1s收不到就认为失联
//    {
//        handle->WL_RX_board_state.communicate_state = MISSED;
//    }
//    
//    /* 如果上车，需要结合通信状态 */
//#if (_ON_CAR_USING_ == 1U)
//    
//    /* 更新总状态 */
//    if (handle->WL_RX_board_state.init_state \
//        + handle->WL_RX_board_state.cap_current_state \
//        + handle->WL_RX_board_state.power_supply_state \
//        + handle->WL_RX_board_state.code_match_state \
//        + handle->WL_RX_board_state.communicate_state == 0)    // 所有都是 NORMAL/DONE 的话总和就是0
//    {
//        handle->WL_RX_board_state.total_state = IN_NORMAL;
//    }
//    else
//    {
//        handle->WL_RX_board_state.total_state = IN_ERROR;
//    }
//    
//#else
//    
//    /* 更新总状态 */
//    if (handle->WL_RX_board_state.init_state \
//        + handle->WL_RX_board_state.cap_current_state \
//        + handle->WL_RX_board_state.power_supply_state \
//        + handle->WL_RX_board_state.code_match_state == 0)    // 所有都是 NORMAL/DONE 的话总和就是0
//    {
//        handle->WL_RX_board_state.total_state = IN_NORMAL;
//    }
//    else
//    {
//        handle->WL_RX_board_state.total_state = IN_ERROR;
//    }
//    
//#endif
//    
//}

///*!
// * @usage: 强制输出低电平
// */
//static inline void force_pwm_negative(void)
//{
//    /* 进入临界区（保存状态并禁用中断） */
//    uint32_t primask = __get_PRIMASK();         // 保存当前中断状态
//    __disable_irq();                            // 屏蔽所有中断
//    
//    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | \
//                                           HRTIM_OUTPUT_TB2 | \
//                                           HRTIM_OUTPUT_TD1 | \
//                                           HRTIM_OUTPUT_TD2 );
//    
//    /* 退出临界区（恢复原始中断状态） */
//    __set_PRIMASK(primask);  
//    
//}

///*!
// * @usage: 恢复pwm输出
// */
//static inline void resume_pwm_output(void)
//{
//    /* 进入临界区（保存状态并禁用中断） */
//    uint32_t primask = __get_PRIMASK();         // 保存当前中断状态
//    __disable_irq();                            // 屏蔽所有中断
//    
//    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 | \
//                                            HRTIM_OUTPUT_TB2 | \
//                                            HRTIM_OUTPUT_TD1 | \
//                                            HRTIM_OUTPUT_TD2 );
//    
//    /* 退出临界区（恢复原始中断状态） */
//    __set_PRIMASK(primask);  
//    
//}

///*!
// * @usage: 状态错误强制输出低电平
// */
//static inline void protection(void)
//{
//    if (WL_RX_board.WL_RX_board_state.total_state == IN_ERROR)
//    {
//        force_pwm_negative();
//    }
//    else if (WL_RX_board.WL_RX_board_state.total_state == IN_NORMAL)
//    {
//        resume_pwm_output();
//    }
//}

///*!
// * @brief: can发送数据给主控
// */
//static inline void tx_info_to_master(WL_RX_board_t *WL_RX_board, wireless_tx_info_t *WL_RX_brd_tx_info)
//{
//    /* 临时发送数组 */
//    const uint8_t DATA_LENGTH = 8;
//    uint8_t data[DATA_LENGTH];
//    
//    /* 更新发送数据 */
//    float tx_power = WL_RX_board->input_power;
//    if (tx_power <= 0.f)
//    {
//        tx_power = 0.f; // 如果功率小于0，发送0
//    }
//    
//    WL_RX_brd_tx_info->charging_power = float_to_int16(tx_power, 150, 0, 32000, -32000);
//    
//    /* 总状态正常且电量足够才允许放电 */
//    if (WL_RX_board->WL_RX_board_state.total_state == IN_NORMAL)
//    {
//        WL_RX_brd_tx_info->is_charging = 1;
//    }
//    else
//    {
//        WL_RX_brd_tx_info->is_charging = 0;
//    }
//    
//    /* 拷贝内存 */
//    memcpy(data, WL_RX_brd_tx_info, DATA_LENGTH);
//    
//    /* 发送数据 */
//    FDCAN3_SendData(WIRELESS_CAN_TX_ID, DATA_LENGTH, data);
//}

///*!
// * @brief: LED显示状态
// */
//static inline void LED_show_state(void)
//{
//    typedef enum {RED = 0, BLUE}LED_sequence_e;
//    typedef enum {OFF = 0, ON, BLINK}LED_state_e;
//    static LED_state_e LED_state[2] = {OFF, OFF};
//    
//    static uint32_t cnt;
//    
//    /* 基本状态使用LED展示 */
//    if (WL_RX_board.WL_RX_board_state.total_state == NORMAL)
//    {
//        LED_state[BLUE] = ON;
//    }
//    else if (WL_RX_board.WL_RX_board_state.total_state == IN_ERROR)
//    {
//        if (WL_RX_board.WL_RX_board_state.code_match_state == MISMATCHED)
//        {
//            LED_state[BLUE] = BLINK;
//        }
//        else
//        {
//            LED_state[BLUE] = OFF;
//        }
//    }
//    
//    if (WL_RX_board.WL_RX_board_state.communicate_state == MISSED)
//    {
//        LED_state[RED] = ON;
//    }
//    else if (WL_RX_board.WL_RX_board_state.total_state == CONNECTED)
//    {
//        LED_state[RED] = OFF;
//    }
//    
//    /* 蓝灯 */
//    if (LED_state[BLUE] == ON)
//    {
//        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
//    }
//    else if (LED_state[BLUE] == OFF)
//    {
//        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
//    }
//    else if (LED_state[BLUE] == BLINK)
//    {
//        cnt++;
//        /* 1.2ms周期下，240ms执行一次括号内容 */
//        if (cnt % 200 == 0)
//        {
//            HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
//        }
//        
//    }
//    
//    /* 红灯 */
//    if (LED_state[RED] == ON)
//    {
//        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
//    }
//    else if (LED_state[RED] == OFF)
//    {
//        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
//    }
//    
//}

///* Define Exported Functions ---------------------------------------------------------------------------------------------------------------------------------------- */
///*!
// * @usage: 检查代码是否与板号匹配
// */
//inline void check_code_match(void)
//{
//    
//    WL_RX_board.device_unique_id[0] = *(uint32_t *)(0x1FFF7590);
//    WL_RX_board.device_unique_id[1] = *(uint32_t *)(0x1FFF7594);
//    WL_RX_board.device_unique_id[2] = *(uint32_t *)(0x1FFF7598);
//    
//    if (WL_RX_board.device_unique_id[0] == DEVICE_UID_LOW \
//        && WL_RX_board.device_unique_id[1] == DEVICE_UID_MID \
//        && WL_RX_board.device_unique_id[2] == DEVICE_UID_MOST)
//    {
//        WL_RX_board.WL_RX_board_state.code_match_state = MATCHED;
//    }
//    
//}

///*!
// * @brief: 解析主控can发送的数据
// */
//inline void get_master_rx_info(uint8_t *data)
//{
//    /* 接收数组长度 */
//    const uint8_t DATA_LENGTH = 8;
//    
//    /* 拷贝内存 */
//    memcpy(&WL_RX_board_rx_info, data, DATA_LENGTH);
//    
//    /* 信息确认、限幅处理 */
//    WL_RX_board_rx_info.chassis_power_buffer = (WL_RX_board_rx_info.chassis_power_buffer > 250) ? \
//                                             250 : WL_RX_board_rx_info.chassis_power_buffer;
//    WL_RX_board_rx_info.cap_power_out_limit = (WL_RX_board_rx_info.cap_power_out_limit > 0) ? \
//                                            0 : WL_RX_board_rx_info.cap_power_out_limit;
//    WL_RX_board_rx_info.cap_power_out_limit = (WL_RX_board_rx_info.cap_power_out_limit < CAP_POWER_OUT_MAX) ? \
//                                            CAP_POWER_OUT_MAX : WL_RX_board_rx_info.cap_power_out_limit;
//    
//    /* 更新与主机的连接状态 */
//    miss_master_cnt = 0;
//    WL_RX_board.WL_RX_board_state.communicate_state = CONNECTED;
//}


//#if ((_ON_CAR_USING_ == 1U) || (_BEFORE_CAR_TEST_ == 1U) || (_CALIBRATE_CAP_I_ == 1U))

///*!
// * @brief: 主控制程序
// */
//void main_control_task(void)
//{
//    /* 滤波、转换数据 */
//    data_filter_convert(WL_RX_board.dma_adc_raw_data, WL_RX_board.adc_filter_info);
//    
//    /* 校准原始数据 */
//    info_calibrate(&WL_RX_board);
//    
//    /* 计算功率等数据 */
//    info_calculate(&WL_RX_board);
//    
//    /* pid计算输出 */
//    integrate_pid_control(&WL_RX_board);
//    
//    /* 计算pwm波占空比 */
//    pwm_duty_calculate(&WL_RX_board);
//    
//    /* 状态检测及保护 */
//    check_states(&WL_RX_board);
//    protection();
//    
//    /* 更新pwm波 */
//    pwm_update(&WL_RX_board);
//    
//    /* 30us周期下，1.2ms执行一次括号内容 */
//    static uint32_t i;
//    i++;
//    if (i % 40 == 0)
//    {
//       /* 给主机发送信息 */
//       tx_info_to_master(&WL_RX_board ,&WL_RX_board_tx_info);
//        
//        /* 基本状态使用LED展示 */
//        LED_show_state();
//    }
//    
//}

//#endif

//#if (_CALIBRATE_EXCEPT_CAP_I_ == 1U)

///*!
// * @brief: 主控制程序
// */
//void main_control_task(void)
//{
//    /* 滤波、转换数据 */
//    data_filter_convert(WL_RX_board.dma_adc_raw_data, WL_RX_board.adc_filter_info);
//    
//    /* 校准原始数据 */
//    info_calibrate(&WL_RX_board);
//    
//    /* 计算功率等数据 */
//    info_calculate(&WL_RX_board);
//    
//}

//#endif

//#if (_PWM_TEST_ == 1U)

///*!
// * @brief: 主控制程序
// */
//void main_control_task(void)
//{
//    /* 滤波、转换数据 */
//    data_filter_convert(WL_RX_board.dma_adc_raw_data, WL_RX_board.adc_filter_info);
//    
//    /* 校准原始数据 */
//    info_calibrate(&WL_RX_board);
//    
//    /* 计算功率等数据 */
//    info_calculate(&WL_RX_board);
//    
//    /* 计算pwm波占空比 */
//    pwm_duty_calculate(&WL_RX_board);
//    
//    /* 更新pwm波 */
//    pwm_update(&WL_RX_board);
//    
//}

//#endif
