//#include "ui.h"
//#include "chassis.h"
//#include "gimbal.h"
//#include "infantry.h"
//#include "priority_ui.h"
//#include "cap.h"
//#include "board_protocol.h"
//#include "judge_protocol.h"

//void rotate_point(__packed uint16_t *x, __packed uint16_t *y, uint16_t raw_x, uint16_t raw_y, float mid_x, float mid_y, float angle);
//void Get_Hp(float *current_hp,float *max_hp,robot_type_e robot_type);

//my_ui_config_t my_ui_config = {
//  .armor_radius = 100,
//};

//ui_info_t dynamic_ui_info [DYNAMIC_UI_NUM] = 
//{
//  [D_CAP_VOLTAGE] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = ARC, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 20, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X , // 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y , //  坐标
//  .ui_config.end_x = 360, // x 半轴长度
//  .ui_config.end_y = 360, // y 半轴长度
//  .ui_config.start_angel = 240, // 起始角度
//  .ui_config.end_angel = 240, // 终止角度
//  },
//  [D_FRIC_B_SPEED] = {
//    /*不变配置*/
//    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//    .ui_config.ui_type = FLOAT, // UI内容类型
//    /*可变配置*/
//    .ui_config.operate_type = MODIFY, // 操作类型
//    .ui_config.layer = 0, // 图层数，0~9
//    .ui_config.color = YELLOW, // 颜色
//    .ui_config.size = 20, // 字体大小
//    .ui_config.width = 3, // 线条宽度
//    .ui_config.start_x = CLIENT_MID_POSITION_X + 470, // 起点 x 坐标
//    .ui_config.start_y = CLIENT_MID_POSITION_Y + 320, // 起点 y 坐标
//    .ui_config.float_num = 66.66, // 显示的数字
//    .ui_config.decimal = 0, // 小数位有效个数
//  },
//  [D_FRIC_F_SPEED] = {
//    /*不变配置*/
//    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//    .ui_config.ui_type = FLOAT, // UI内容类型
//    /*可变配置*/
//    .ui_config.operate_type = MODIFY, // 操作类型
//    .ui_config.layer = 0, // 图层数，0~9
//    .ui_config.color = PINK, // 颜色
//    .ui_config.size = 20, // 字体大小 
//    .ui_config.width = 3, // 线条宽度
//    .ui_config.start_x = CLIENT_MID_POSITION_X +600, // 起点 x 坐标
//    .ui_config.start_y = CLIENT_MID_POSITION_Y +320, // 起点 y 坐标
//    .ui_config.float_num = 66.66, // 显示的数字
//    .ui_config.decimal = 0, // 小数位有效个数
//  },
//  [D_UWB_YAW] = {
//    /*不变配置*/
//    .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
//    .ui_config.ui_type = FLOAT, // UI内容类型
//    /*可变配置*/
//    .ui_config.operate_type = MODIFY, // 操作类型
//    .ui_config.layer = 0, // 图层数，0~9
//    .ui_config.color = GREEN, // 颜色
//    .ui_config.size = 20, // 字体大小 
//    .ui_config.width = 2, // 线条宽度
//    .ui_config.start_x = CLIENT_MID_POSITION_X + 730, // 起点 x 坐标
//    .ui_config.start_y = CLIENT_MID_POSITION_Y + 320, // 起点 y 坐标
//    .ui_config.float_num = 66.66, // 显示的数字
//    .ui_config.decimal = 2, // 小数位有效个数
//  },
//  [D_PTICH_IMU_ANGLE] = {
//  /*不变配置*/
//  .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = FLOAT, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = GREEN, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X + 80,  // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 350, // 起点 y 坐标
//  .ui_config.float_num = 66.66, // 显示的数字
//  .ui_config.decimal = 2, // 小数位有效个数
//  },

//  [D_VISION_CIRCLE] = {
//  /*不变配置*/
//  .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = CIRCLE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 30, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 610, // 圆心 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 60, // 圆心 y 坐标
//  .ui_config.radius = 7, // 半径
//  },
//  
//  [D_SPEED_ADAPT_CYCLE] = {
//  /*不变配置*/
//  .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = CIRCLE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 30, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 610, // 圆心 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 172, // 圆心 y 坐标
//  .ui_config.radius = 7, // 半径
//  },
//  [D_CAP_ON_CYCLE] = {
//  /*不变配置*/
//  .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = CIRCLE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 30, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 610, // 圆心 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 116, // 圆心 y 坐标
//  .ui_config.radius = 7, // 半径
//  },
//  [D_CAR_MODE] = {
//  /*不变配置*/
//  .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = GREEN, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 610, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 17, // 起点 y 坐标
//  .ui_config.text = "DUNE", // 显示的文字
//  }, 
//  [D_FRIC_STATE_CYCLE] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = CIRCLE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 30, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X + 650, // 圆心 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y - 20, // 圆心 y 坐标
//  .ui_config.radius = 7, // 半径
//  },
//  
//  [D_HIT_TARGET_DISTANCE] = {
//  /*不变配置*/
//  .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = FLOAT, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = RED_BLUE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X + -50, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 350, // 起点 y 坐标
//  .ui_config.decimal = 2,
//  .ui_config.float_num = 0, // 显示的数字
//  },

//  [D_RFID_CYCLE] = {
// /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = CIRCLE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 30, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 610, // 圆心 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 238 , // 圆心 y 坐标
//  .ui_config.radius = 7, // 半径
//  },
//  [D_MID_RECTANGEL] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = RECTANGEL, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = BLACK, // 颜色
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - _VISION__RECTANGEL_X_WIDTH, // 左上角 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + _VISION__RECTANGEL_Y_OFFSET, // 左上角 y 坐标
//  .ui_config.end_x = CLIENT_MID_POSITION_X + _VISION__RECTANGEL_X_WIDTH, // 右下角 x 坐标
//  .ui_config.end_y = CLIENT_MID_POSITION_Y - _VISION__RECTANGEL_Y_OFFSET, // 右下角 y 坐标
//  },
//  [D_HIT_TARGET_TRIANGLE_1] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = RED_BLUE, // 颜色
//  .ui_config.width = 6, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 10, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 120, // 起点 y 坐标
//  .ui_config.end_x = CLIENT_MID_POSITION_X , // 终点 x 坐标
//  .ui_config.end_y = CLIENT_MID_POSITION_Y + 137, // 终点 y 坐标
//  },
//  [D_HIT_TARGET_TRIANGLE_2] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = RED_BLUE, // 颜色
//  .ui_config.width = 6, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X , // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 137, // 起点 y 坐标
//  .ui_config.end_x = CLIENT_MID_POSITION_X + 10, // 终点 x 坐标
//  .ui_config.end_y = CLIENT_MID_POSITION_Y + 120, // 终点 y 坐标
//  }, 
//  [D_HIT_HIGHLIGHT_LINE_1] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 3, // 图层数，0~9
//  .ui_config.color = CYAN_BLUE, // 颜色
//  .ui_config.width = 3, // 线条宽度
//  .ui_config.start_x = 0, // 起点 x 坐标
//  .ui_config.start_y = 0, // 起点 y 坐标
//  .ui_config.end_x = 0, // 终点 x 坐标
//  .ui_config.end_y = 0, // 终点 y 坐标
//  },
//  [D_HIT_HIGHLIGHT_LINE_2] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 3, // 图层数，0~9
//  .ui_config.color = CYAN_BLUE, // 颜色
//  .ui_config.width = 3, // 线条宽度
//  .ui_config.start_x = 0, // 起点 x 坐标
//  .ui_config.start_y = 0, // 起点 y 坐标
//  .ui_config.end_x = 0, // 终点 x 坐标
//  .ui_config.end_y = 0, // 终点 y 坐标
//  },
//  [D_HEAD_CYCLE] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = CIRCLE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X , // 圆心 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y +130, // 圆心 y 坐标
//  .ui_config.radius = 20, // 半径
//  },
//  #ifndef UI_SIMPLIFY
//  [D_VISION_ARMOR_CYCLE] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = CIRCLE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = BLACK, // 颜色
//  .ui_config.width = 6, // 线条宽度
//  .ui_config.start_x = 0 , // 圆心 x 坐标
//  .ui_config.start_y = 0 , // 圆心 y 坐标
//  .ui_config.radius = 11, // 半径
//  },
//  [D_VISION_HP_CYCLE] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = ARC, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = GREEN, // 颜色
//  .ui_config.width = 6, // 线条宽度
//  .ui_config.start_x = 0, // 圆心 x 坐标
//  .ui_config.start_y = 0, // 圆心 y 坐标
//  .ui_config.end_x = 11, // x 半轴长度
//  .ui_config.end_y = 11, // y 半轴长度
//  .ui_config.start_angel = 0, // 起始角度
//  .ui_config.end_angel = 0, // 终止角度
//  },
//  [D_VISION_WHITE_CYCLE] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = ARC, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 6, // 线条宽度
//  .ui_config.start_x = 0, // 圆心 x 坐标
//  .ui_config.start_y = 0, // 圆心 y 坐标
//  .ui_config.end_x = 11, // x 半轴长度
//  .ui_config.end_y = 11, // y 半轴长度
//  .ui_config.start_angel = 0, // 起始角度
//  .ui_config.end_angel = 0, // 终止角度
//  },
//  [D_ROI_LEFT] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = RED_BLUE, // 颜色
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = 0, // 起点 x 坐标
//  .ui_config.start_y = 0, // 起点 y 坐标
//  .ui_config.end_x = 0, // 终点 x 坐标
//  .ui_config.end_y = 0, // 终点 y 坐标
//  },
//  [D_ROI_UP] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = RED_BLUE, // 颜色
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = 0, // 起点 x 坐标
//  .ui_config.start_y = 0, // 起点 y 坐标
//  .ui_config.end_x = 0, // 终点 x 坐标
//  .ui_config.end_y = 0, // 终点 y 坐标
//  },
//  [D_ROI_RIGHT] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = RED_BLUE, // 颜色
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = 0, // 起点 x 坐标
//  .ui_config.start_y = 0, // 起点 y 坐标
//  .ui_config.end_x = 0, // 终点 x 坐标
//  .ui_config.end_y = 0, // 终点 y 坐标
//  },
//  [D_ROI_DOWN] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = RED_BLUE, // 颜色
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = 0, // 起点 x 坐标
//  .ui_config.start_y = 0, // 起点 y 坐标
//  .ui_config.end_x = 0, // 终点 x 坐标
//  .ui_config.end_y = 0, // 终点 y 坐标
//  },
//  [D_ROI_MID] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = FUCHSIA, // 颜色
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = 0, // 起点 x 坐标
//  .ui_config.start_y = 0, // 起点 y 坐标
//  .ui_config.end_x = 0, // 终点 x 坐标
//  .ui_config.end_y = 0, // 终点 y 坐标
//  },
//  #endif
//  [D_PITCH_POINTER] = {
//  /*不变配置*/
//  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.operate_type = MODIFY, // 操作类型
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = RED_BLUE, // 颜色
//  .ui_config.width = 5, // 线条宽度
//  .ui_config.start_x = 1260, // 起点 x 坐标
//  .ui_config.start_y = 540, // 起点 y 坐标
//  .ui_config.end_x = 1340, // 终点 x 坐标
//  .ui_config.end_y = 540, // 终点 y 坐标
//  },
// 
//};

//ui_info_t const_ui_info [CONST_UI_NUM] = 
//{
//  [C_VISION_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 800, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 72, // 起点 y 坐标
//  .ui_config.text = "VISION", // 显示的文字
//  },
//  [C_FRIC_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X + 530, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y , // 起点 y 坐标
//  .ui_config.text = "FRIC", // 显示的文字
//  },
//  [C_FRIC_ADAPT_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 800, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 190, // 起点 y 坐标
//  .ui_config.text = "FRIC_ADAPT", // 显示的文字
//  },
//  [C_RFID_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 800, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 255, // 起点 y 坐标
//  .ui_config.text = "RFID", // 显示的文字
//  },
//  [C_CAP_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 800, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 127, // 起点 y 坐标
//  .ui_config.text = "CAP", // 显示的文字
//  },
//  [C_CAR_MODE_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 2, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 800, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y + 17, // 起点 y 坐标
//  .ui_config.text = "MODE", // 显示的文字
//  },
//  [C_MID_LINE] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = ORANGE, // 颜色
//  .ui_config.width = 2, // 线条宽度
//	  #if HERO_TYPE!=2
//	   .ui_config.start_x = CLIENT_MID_POSITION_X+0, // 起点 x 坐标
//	  #else
//	   .ui_config.start_x = CLIENT_MID_POSITION_X-15, // 起点 x 坐标
//	  #endif
//		.ui_config.start_y = CLIENT_MID_POSITION_Y, // 起点 y 坐标
//	  
//	  #if HERO_TYPE!=2
//	   .ui_config.end_x = CLIENT_MID_POSITION_X+0, // 起点 x 坐标
//	  #else
//	   .ui_config.end_x = CLIENT_MID_POSITION_X-15, // 起点 x 坐标
//	  #endif
//	  
//  .ui_config.end_y = 0, // 终点 y 坐标
//  },
//  [C_TOILET_DOWN_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = FUCHSIA, // 颜色
//  .ui_config.size = 10, // 字体大小
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X + _TOILET_DOWN_WIDTH + 20, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y - _TOILET_DOWN_OFFSET + 5, // 起点 y 坐标
//  .ui_config.text = "TOILET_DOWN", // 显示的文字
//  },
//  [C_OUTPOST_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = FUCHSIA, // 颜色
//  .ui_config.size = 10, // 字体大小
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X + _OUTPOST_WIDTH + 20, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y - _OUTPOST_OFFSET + 5, // 起点 y 坐标
//  .ui_config.text = "OUTPOST", // 显示的文字
//  },
//  [C_HEIGHT_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = CYAN_BLUE, // 颜色
//  .ui_config.size = 10, // 字体大小
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X + _HEIGHT_WIDTH + 20, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y - _HEIGHT_OFFSET + 5, // 起点 y 坐标
//  .ui_config.text = "HEIGHT", // 显示的文字
//  },
//  [C_HEIGHT_LINE] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = CYAN_BLUE, // 颜色
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - _HEIGHT_WIDTH, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y - _HEIGHT_OFFSET, // 起点 y 坐标
//  .ui_config.end_x = CLIENT_MID_POSITION_X + _HEIGHT_WIDTH, // 终点 x 坐标
//  .ui_config.end_y = CLIENT_MID_POSITION_Y - _HEIGHT_OFFSET, // 终点 y 坐标
//  },
//  [C_OUTPOST_LINE] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = CYAN_BLUE, // 颜色
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - _OUTPOST_WIDTH, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y - _OUTPOST_OFFSET, // 起点 y 坐标
//  .ui_config.end_x = CLIENT_MID_POSITION_X + _OUTPOST_WIDTH, // 终点 x 坐标
//  .ui_config.end_y = CLIENT_MID_POSITION_Y - _OUTPOST_OFFSET, // 终点 y 坐标
//  },
//  [C_TOILET_DOWN_LINE] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 1, // 图层数，0~9
//  .ui_config.color = CYAN_BLUE, // 颜色
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - _TOILET_DOWN_WIDTH, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y - _TOILET_DOWN_OFFSET, // 起点 y 坐标
//  .ui_config.end_x = CLIENT_MID_POSITION_X + _TOILET_DOWN_WIDTH, // 终点 x 坐标
//  .ui_config.end_y = CLIENT_MID_POSITION_Y - _TOILET_DOWN_OFFSET, // 终点 y 坐标
//  },
//  [C_PASS_LINE_LEFT] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X - 155, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y - 60, // 起点 y 坐标
//  .ui_config.end_x = CLIENT_MID_POSITION_X - 420, // 终点 x 坐标
//  .ui_config.end_y = 0, // 终点 y 坐标
//  },
//  [C_PASS_LINE_RIGHT] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色·
//  .ui_config.width = 1, // 线条宽度
//  .ui_config.start_x = CLIENT_MID_POSITION_X + 155, // 起点 x 坐标
//  .ui_config.start_y = CLIENT_MID_POSITION_Y - 60, // 起点 y 坐标
//  .ui_config.end_x = CLIENT_MID_POSITION_X + 420, // 终点 x 坐标
//  .ui_config.end_y = 0, // 终点 y 坐标
//  },
////  #ifndef UI_SIMPLIFY
//  [C_PITCH_LINE_0] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 10, // 线条宽度
//  .ui_config.start_x = 1300, // 起点 x 坐标
//  .ui_config.start_y = 540, // 起点 y 坐标
//  .ui_config.end_x = 1330, // 终点 x 坐标
//  .ui_config.end_y = 540, // 终点 y 坐标
//  },
//  [C_PITCH_LINE_10] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 10, // 线条宽度
//  .ui_config.start_x = 1295, // 起点 x 坐标
//  .ui_config.start_y = 599, // 起点 y 坐标
//  .ui_config.end_x = 1324, // 终点 x 坐标
//  .ui_config.end_y = 604, // 终点 y 坐标
//  },
//  [C_PITCH_LINE_20] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 10, // 线条宽度
//  .ui_config.start_x = 1279, // 起点 x 坐标
//  .ui_config.start_y = 656, // 起点 y 坐标
//  .ui_config.end_x = 1307, // 终点 x 坐标
//  .ui_config.end_y = 666, // 终点 y 坐标
//  },
//  [C_PITCH_LINE_30] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 10, // 线条宽度
//  .ui_config.start_x = 1254, // 起点 x 坐标
//  .ui_config.start_y = 710, // 起点 y 坐标
//  .ui_config.end_x = 1280, // 终点 x 坐标
//  .ui_config.end_y = 725, // 终点 y 坐标
//  },
//  [C_PITCH_LINE_40] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 10, // 线条宽度
//  .ui_config.start_x = 1220, // 起点 x 坐标
//  .ui_config.start_y = 758, // 起点 y 坐标
//  .ui_config.end_x = 1243, // 终点 x 坐标
//  .ui_config.end_y = 778, // 终点 y 坐标
//  },
//  [C_PITCH_LINE_50] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 10, // 线条宽度
//  .ui_config.start_x = 1178, // 起点 x 坐标
//  .ui_config.start_y = 800, // 起点 y 坐标
//  .ui_config.end_x = 1198, // 终点 x 坐标
//  .ui_config.end_y = 823, // 终点 y 坐标
//  },
//  [C_PITCH_LINE_N_10] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 10, // 线条宽度
//  .ui_config.start_x = 1294, // 起点 x 坐标
//  .ui_config.start_y = 480, // 起点 y 坐标
//  .ui_config.end_x = 1324, // 终点 x 坐标
//  .ui_config.end_y = 476, // 终点 y 坐标
//  },
//  [C_PITCH_LINE_N_20] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 10, // 线条宽度
//  .ui_config.start_x = 1279, // 起点 x 坐标
//  .ui_config.start_y = 424, // 起点 y 坐标
//  .ui_config.end_x = 1308, // 终点 x 坐标
//  .ui_config.end_y = 413, // 终点 y 坐标
//  },
//  [C_PITCH_LINE_N_30] = {
//  /*不变配置*/
//  .ui_config.ui_type = LINE, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = WHITE, // 颜色
//  .ui_config.width = 10, // 线条宽度
//  .ui_config.start_x = 1254, // 起点 x 坐标
//  .ui_config.start_y = 370, // 起点 y 坐标
//  .ui_config.end_x = 1280, // 终点 x 坐标
//  .ui_config.end_y = 355, // 终点 y 坐标
//  }, 
//  [C_PITCH_50_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = CYAN_BLUE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 3, // 线条宽度
//  .ui_config.start_x = 1200, // 起点 x 坐标
//  .ui_config.start_y = 865 , // 起点 y 坐标
//  .ui_config.text = "50", // 显示的文字
//  },
//  [C_PITCH_0_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = CYAN_BLUE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 3, // 线条宽度
//  .ui_config.start_x = 1340, // 起点 x 坐标
//  .ui_config.start_y = 550 , // 起点 y 坐标
//  .ui_config.text = "0", // 显示的文字
//  },
//  [C_PITCH_N_30_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = CYAN_BLUE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 3, // 线条宽度
//  .ui_config.start_x = 1290, // 起点 x 坐标
//  .ui_config.start_y = 346 , // 起点 y 坐标
//  .ui_config.text = "-30", // 显示的文字
//  },
//  [C_PITCH_30_CHAR] = {
//  /*不变配置*/
//  .ui_config.ui_type = CHAR, // UI内容类型
//  /*可变配置*/
//  .ui_config.layer = 0, // 图层数，0~9
//  .ui_config.color = CYAN_BLUE, // 颜色
//  .ui_config.size = 20, // 字体大小
//  .ui_config.width = 3, // 线条宽度
//  .ui_config.start_x = 1300, // 起点 x 坐标
//  .ui_config.start_y = 760 , // 起点 y 坐标
//  .ui_config.text = "30", // 显示的文字
//  },
////  #endif
//};


///**
// * @brief 初始化UI 在main.c中调用
// * 
// */
//void My_Ui_Init(void)
//{
//  Init_Ui_List(dynamic_ui_info, sizeof(dynamic_ui_info)/sizeof(ui_info_t),const_ui_info, sizeof(const_ui_info)/sizeof(ui_info_t));
//}

//// float max_hp,current_hp;

///**
// * @brief UI信息更新 任务中调用
// * 
// */

//void Ui_Info_Update(void)
//{
//	client_info_update();
//  /*超电UI更新****************************************************/
//   static float last_cap_voltage = 0;
//	//#define BUFFER
//	#ifdef BUFFER
//	float cap_voltage = (float)judge.power_heat_data.chassis_power_buffer;
//	dynamic_ui_info[D_CAP_VOLTAGE].ui_config.end_angel = 240.f + 75.f*(cap_voltage/60.f);
//	#else
//	 
//	float cap_voltage = cap.info->cap_Ucr;
//	 dynamic_ui_info[D_CAP_VOLTAGE].ui_config.end_angel = 240.f + 75.f*(cap_voltage/24.f)*(cap_voltage/24.f);
//	#endif
//  
//  if (last_cap_voltage != cap_voltage)
//  {
//    
//	 #ifdef BUFFER
//	if (cap_voltage < 25)//电压低于15V变红
//	#else
//	if (cap_voltage < 15)//电压低于15V变红
//	#endif 
//    {
//      dynamic_ui_info[D_CAP_VOLTAGE].ui_config.color = PINK;
//    }
//	else if(cap_voltage>23.4f)
//	{
//      dynamic_ui_info[D_CAP_VOLTAGE].ui_config.color = GREEN;
//    }
//    else //电压高于15V变白+
//    {
//      dynamic_ui_info[D_CAP_VOLTAGE].ui_config.color = WHITE;
//    }
//	if (wireless_rx_info.is_charging==1)
//	{
//		dynamic_ui_info[D_CAP_VOLTAGE].ui_config.color=YELLOW;
//	}
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_CAP_VOLTAGE]);
//  }
//  last_cap_voltage = cap_voltage;
//  /*UWB XY更新*****************************************/
//  static float last_fric_b_speed = 0, last_fric_f_speed = 0, last_uwb_yaw = 0;
//  float fric_f_speed =  communicate.car_data1_rx_info->fric_f_speed;
//  float fric_b_speed =  communicate.car_data1_rx_info->fric_b_speed;
//  float uwb_yaw = judge.game_robot_pos.angle;
//  if (last_fric_f_speed != fric_f_speed)
//  {
//    dynamic_ui_info[D_FRIC_F_SPEED].ui_config.float_num = fric_f_speed;
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_FRIC_F_SPEED]);
//  }
//  if (last_fric_b_speed != fric_b_speed)
//  {
//    dynamic_ui_info[D_FRIC_B_SPEED].ui_config.float_num = fric_b_speed;
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_FRIC_B_SPEED]);
//  }
//  if (last_uwb_yaw != uwb_yaw)
//  {
//    dynamic_ui_info[D_UWB_YAW].ui_config.float_num = uwb_yaw;
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_UWB_YAW]);
//  }
//  last_uwb_yaw = uwb_yaw;
//  last_fric_f_speed = fric_f_speed;
//  last_fric_b_speed = fric_b_speed;
//  /*云台机械角度更新***************************************************/
//  /*云台角度UI更新****************************************************/
//  static float last_pitch_imu_angle = 0;
//  float pitch_imu_angle = (float)board.rx_meg->gimbal_meg.pitch_imu; // 云台角度转浮点数
//  if (last_pitch_imu_angle != pitch_imu_angle)
//  {
//    dynamic_ui_info[D_PTICH_IMU_ANGLE].ui_config.float_num = pitch_imu_angle;
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_PTICH_IMU_ANGLE]);
//  }
//  last_pitch_imu_angle = pitch_imu_angle;
// /*视觉允许打弹状态UI更新****************************************************/
//  static uint8_t last_is_vision_online = 0;
//  uint8_t is_vision_online = communicate.car_data0_rx_info->car_state.bit.is_vision_online;
//  if (last_is_vision_online != is_vision_online)
//  {
//    if (is_vision_online) // 视觉允许打弹
//    {
//      dynamic_ui_info[D_VISION_CIRCLE].ui_config.color = GREEN;
//    }
//    else
//    {
//      dynamic_ui_info[D_VISION_CIRCLE].ui_config.color = WHITE;
//    }
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_VISION_CIRCLE]);
//  }
//  last_is_vision_online = is_vision_online;
//  /*无自适应UI更新****************************************************/
//  static uint8_t last_is_open_adapt_flag = 0;
//  uint8_t is_open_adapt_flag = communicate.car_data0_rx_info->car_state.bit.is_open_adapt ;
//  if (is_open_adapt_flag != last_is_open_adapt_flag)
//  {
//    if (is_open_adapt_flag==1) 
//    {
//      dynamic_ui_info[D_SPEED_ADAPT_CYCLE].ui_config.color = GREEN;
//    }
//    else
//    {
//      dynamic_ui_info[D_SPEED_ADAPT_CYCLE].ui_config.color = WHITE;
//    }
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_SPEED_ADAPT_CYCLE]);
//  }
//  last_is_open_adapt_flag = is_open_adapt_flag;
//  /*超电UI更新****************************************************/
//  static uint8_t last_is_on_cap = 0;
//  uint8_t is_on_cap = cap.info->ability;
//  if (last_is_on_cap != is_on_cap)
//  {
//    if (is_on_cap) 
//    {
//      dynamic_ui_info[D_CAP_ON_CYCLE].ui_config.color = GREEN;
//    }
//    else
//    {
//      dynamic_ui_info[D_CAP_ON_CYCLE].ui_config.color = WHITE;
//    }
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_CAP_ON_CYCLE]);
//  }
//  last_is_on_cap = is_on_cap;
//  /*车辆模式UI更新****************************************************/
//  static uint8_t last_car_mode = 0;
//  uint8_t car_mode = infantry.mode;

//  if (last_car_mode != car_mode)
//  {
//    memset(dynamic_ui_info[D_CAR_MODE].ui_config.text,0,sizeof(dynamic_ui_info[D_CAR_MODE].ui_config.text));
//    switch (car_mode)
//    {
//    case offline_CAR:
//      strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "OFFLINE");
//      break;
//    case init_CAR:
//      strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "INIT");
//      break;
//    case cycle_CAR:
//      strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "CYCLE");
//      break;
//    case gyro_CAR:
//      strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "GYRO");
//      break;
//    case mec_CAR:
//      strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "MEC");
//      break;
//    case hole_CAR:
//      strcpy(dynamic_ui_info[D_CAR_MODE].ui_config.text, "HOLE");
//      break;
//    default:
//      break;
//    }
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_CAR_MODE]);
//  }
//  last_car_mode = car_mode;
//  /*摩擦轮速度UI更新****************************************************/
//  /*中间框颜色****************************************************/
//  static uint8_t last_vision_mode_state = 0;
//  uint8_t vision_mode_state = communicate.car_data0_rx_info->car_move_mode == vision_cycle_CAR || communicate.car_data0_rx_info->car_move_mode == vision_gyro_CAR;
//  static uint8_t last_is_find_target = 0;
//  uint8_t is_find_target = board.rx_meg->vision_meg.is_find_target;

//  if (last_vision_mode_state != vision_mode_state || last_is_find_target != is_find_target)
//  {
//    if (vision_mode_state && is_find_target) // 视觉模式且找到目标
//    {
//      dynamic_ui_info[D_MID_RECTANGEL].ui_config.color = GREEN;
//    }
//    else if (vision_mode_state && !is_find_target) // 视觉模式但未找到目标
//    {
//      dynamic_ui_info[D_MID_RECTANGEL].ui_config.color = WHITE;
//    }
//    else if (!vision_mode_state && is_find_target) // 非视觉模式但找到目标
//    {
//      dynamic_ui_info[D_MID_RECTANGEL].ui_config.color = FUCHSIA;
//    }
//    else // 非视觉模式且未找到目标
//    {
//      dynamic_ui_info[D_MID_RECTANGEL].ui_config.color = BLACK;
//    }
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_MID_RECTANGEL]);
//  }
//  last_vision_mode_state = vision_mode_state;
//  last_is_find_target = is_find_target;
//  /*头朝向指示****************************************************/
//  static float last_chassis_angle = 0;
//  float chassis_angle = gimbal.info.yaw_mec_err_rad;
//  if(last_chassis_angle!=chassis_angle)
//  {
//	rotate_point(&dynamic_ui_info[D_HEAD_CYCLE].ui_config.start_x,
//                    &dynamic_ui_info[D_HEAD_CYCLE].ui_config.start_y,
//                    CLIENT_MID_POSITION_X ,
//                    CLIENT_MID_POSITION_Y +142,
//                    CLIENT_MID_POSITION_X,
//                    CLIENT_MID_POSITION_Y  ,
//                    chassis_angle);
//  }
//  Enqueue_Ui_For_Sending(&dynamic_ui_info[D_HEAD_CYCLE]);
//  last_chassis_angle =chassis_angle;
//  /*击打目标指示****************************************************/
//  static float last_target_angle_err = 0;

//  float target_angle_err = (float)(communicate.game_robot_pos_tx_info->angle_err) * 3.14 / 180.f ; // 底盘角度转浮点数
//  if (last_target_angle_err != target_angle_err)
//  {
//   // 旋转头部三角形
//    rotate_point(&dynamic_ui_info[D_HIT_TARGET_TRIANGLE_1].ui_config.start_x,
//                    &dynamic_ui_info[D_HIT_TARGET_TRIANGLE_1].ui_config.start_y,
//                    CLIENT_MID_POSITION_X - 10,
//                    CLIENT_MID_POSITION_Y + 125,
//                    CLIENT_MID_POSITION_X,
//                    CLIENT_MID_POSITION_Y,
//                    target_angle_err);
//    rotate_point(&dynamic_ui_info[D_HIT_TARGET_TRIANGLE_1].ui_config.end_x,
//                    &dynamic_ui_info[D_HIT_TARGET_TRIANGLE_1].ui_config.end_y,
//                    CLIENT_MID_POSITION_X,
//                    CLIENT_MID_POSITION_Y + 142,
//                    CLIENT_MID_POSITION_X,
//                    CLIENT_MID_POSITION_Y,
//                    target_angle_err);
//    rotate_point(&dynamic_ui_info[D_HIT_TARGET_TRIANGLE_2].ui_config.start_x,
//                    &dynamic_ui_info[D_HIT_TARGET_TRIANGLE_2].ui_config.start_y,
//                    CLIENT_MID_POSITION_X,
//                    CLIENT_MID_POSITION_Y + 142,
//                    CLIENT_MID_POSITION_X,
//                    CLIENT_MID_POSITION_Y,
//                    target_angle_err);
//    rotate_point(&dynamic_ui_info[D_HIT_TARGET_TRIANGLE_2].ui_config.end_x,  
//                    &dynamic_ui_info[D_HIT_TARGET_TRIANGLE_2].ui_config.end_y,
//                    CLIENT_MID_POSITION_X + 10,
//                    CLIENT_MID_POSITION_Y + 125,
//                    CLIENT_MID_POSITION_X,
//                    CLIENT_MID_POSITION_Y,
//                    target_angle_err);

//                    
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_HIT_TARGET_TRIANGLE_1]);
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_HIT_TARGET_TRIANGLE_2]);
//  }
//  last_target_angle_err = target_angle_err;
//  /*击打命中提示叉叉************************************************/
//  uint16_t enemy_outpost_HP;static uint16_t last_enemy_outpost_HP;
//  uint16_t enemy_base_HP;static uint16_t last_enemy_base_HP;
//  static uint8_t hit_highlight_flag;//显示图标为0，不显示时为0
//  static uint16_t hit_tick;//击打到时刻开始计时
//  static uint16_t last_hit_tick;//用来检测跳变
//  //更新血量
//  if(communicate.game_robot_status_tx_info->game_process.bit.car_color==0)//红方
//  {
//	
//	  enemy_outpost_HP=judge.game_robot_HP.blue_outpost_HP;
//	  enemy_base_HP=judge.game_robot_HP.blue_base_HP;
//  }
//  else
//  {
//	  enemy_outpost_HP=judge.game_robot_HP.red_outpost_HP;
//	  enemy_base_HP=judge.game_robot_HP.red_base_HP;
//  }
//  //检测建筑物掉血符合范围
//  if((last_enemy_outpost_HP-enemy_outpost_HP>=100&&last_enemy_outpost_HP-enemy_outpost_HP<=400)
//	  ||(last_enemy_base_HP-enemy_base_HP>=100&&last_enemy_base_HP-enemy_base_HP<=400))
//  {
//	  hit_highlight_flag=1;
//	  hit_tick=0;//复位时间
//	  //画叉叉
//	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_1].ui_config.start_x=CLIENT_MID_POSITION_X-50;
//	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_1].ui_config.start_y=CLIENT_MID_POSITION_Y+50;
//	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_1].ui_config.end_x=CLIENT_MID_POSITION_X+50;
//	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_1].ui_config.end_y=CLIENT_MID_POSITION_Y-50;
//	  
//	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_2].ui_config.start_x=CLIENT_MID_POSITION_X+50;
//	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_2].ui_config.start_y=CLIENT_MID_POSITION_Y+50;
//	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_2].ui_config.end_x=CLIENT_MID_POSITION_X-50;
//	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_2].ui_config.end_y=CLIENT_MID_POSITION_Y-50;
//	  //发送
//	  Enqueue_Ui_For_Sending(&dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_1]);
//	  Enqueue_Ui_For_Sending(&dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_2]);
//  }
//  if(hit_highlight_flag==1)
//  {
//	hit_tick++;
//  }
//  if(hit_tick>=600)//显示时间结束，标志位和时间清零
//  {
//	 //清除
//	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_1].ui_config.start_x=0;
//	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_1].ui_config.start_y=0;
//	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_1].ui_config.end_x=0;
//	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_1].ui_config.end_y=0;
//																
//	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_2].ui_config.start_x=0;
//	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_2].ui_config.start_y=0;
//	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_2].ui_config.end_x=0;
//	  dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_2].ui_config.end_y=0;
//	  //发送
//	  Enqueue_Ui_For_Sending(&dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_1]);
//	  Enqueue_Ui_For_Sending(&dynamic_ui_info[D_HIT_HIGHLIGHT_LINE_2]);
//	hit_tick=0;
//	hit_highlight_flag=0;//不再计时，实现跳变效果
//  }
//  
//  last_enemy_outpost_HP=enemy_outpost_HP;
//  last_enemy_base_HP=enemy_base_HP;
//  
//  /*弹速显示**************************************************/
//  /*摩擦轮转速差显示*******************************************/
//  /*摩擦轮状态显示********************************************/
//  static uint8_t last_fric_state = 0;
//   static uint8_t last_fric_online = 0;
//  uint8_t fric_state = communicate.car_data0_rx_info->car_state.bit.fri_speed_state;
//  uint8_t fric_online = judge.game_robot_status.power_management_shooter_output;//读裁判系统
//  
//  if (fric_state != last_fric_state||last_fric_online!=fric_online)//如果摩擦轮转速变或者在线状态变
//  {
//    if ( fric_online== 0)//摩擦轮掉线，黑
//    {
//      dynamic_ui_info[D_FRIC_STATE_CYCLE].ui_config.color = BLACK;
//    }
//    else if(fric_online== 1&&fric_state==0)//摩擦轮在线但是没转，青色
//    {
//      dynamic_ui_info[D_FRIC_STATE_CYCLE].ui_config.color = CYAN_BLUE;      
//    }
//	else if(fric_online== 1&&fric_state==1)//摩擦轮电机在线并且在转，绿色
//	{
//	  dynamic_ui_info[D_FRIC_STATE_CYCLE].ui_config.color = GREEN;   
//	}
//	
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_FRIC_STATE_CYCLE]);
//  }
//  last_fric_state = fric_state;
//	last_fric_online=fric_online;
//  /*RFID显示********************************************/
//  static uint8_t last_rfid_state = 0;
//  uint8_t rfid_state = (judge.rfid_status.rfid_status != 0);
//  if (last_rfid_state != rfid_state)
//  {
//    if (rfid_state == 1)
//    {
//      dynamic_ui_info[D_RFID_CYCLE].ui_config.color = GREEN;
//    }
//    else
//    {
//      dynamic_ui_info[D_RFID_CYCLE].ui_config.color = WHITE;
//    }
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_RFID_CYCLE]);
//  }
//  last_rfid_state = rfid_state;

//  /*视觉装甲板显示********************************************/
//  #ifndef UI_SIMPLIFY
//  static uint16_t last_vision_armor_x,last_vision_armor_y;
//  uint16_t vision_armor_x = communicate.car_data2_rx_info->ui_x;
//  uint16_t vision_armor_y = 1080 - communicate.car_data2_rx_info->ui_y;
//  if (communicate.car_data0_rx_info->car_state.bit.is_find_target == 1)
//  {
//    if (last_vision_armor_x != vision_armor_x || last_vision_armor_y != vision_armor_y)
//    {
//      //位置更新
//      dynamic_ui_info[D_VISION_ARMOR_CYCLE].ui_config.start_x = vision_armor_x;
//      dynamic_ui_info[D_VISION_ARMOR_CYCLE].ui_config.start_y = vision_armor_y;
//      dynamic_ui_info[D_VISION_HP_CYCLE].ui_config.start_x = vision_armor_x;
//      dynamic_ui_info[D_VISION_HP_CYCLE].ui_config.start_y = vision_armor_y;
//      dynamic_ui_info[D_VISION_WHITE_CYCLE].ui_config.start_x = vision_armor_x;
//      dynamic_ui_info[D_VISION_WHITE_CYCLE].ui_config.start_y = vision_armor_y;
//      //角度更新
//      float max_hp,current_hp;
//      Get_Hp(&current_hp,&max_hp,communicate.car_data2_rx_info->detect_num);
//      dynamic_ui_info[D_VISION_HP_CYCLE].ui_config.end_angel = current_hp/max_hp*360.f;
//      if (current_hp == 0)
//      {
//        dynamic_ui_info[D_VISION_HP_CYCLE].ui_config.end_angel = 1;
//      }
//      //白色过渡动画
//      dynamic_ui_info[D_VISION_WHITE_CYCLE].ui_config.start_angel = dynamic_ui_info[D_VISION_HP_CYCLE].ui_config.end_angel;
//      float target_white_end_angel = dynamic_ui_info[D_VISION_WHITE_CYCLE].ui_config.start_angel + 1;
//      if (dynamic_ui_info[D_VISION_WHITE_CYCLE].ui_config.end_angel > target_white_end_angel)//会逐渐靠近目标角度（也就是血条结束角度，也是自身的开始角度），但是永远不会到达，因为每次都是差值的20%
//      {
//        dynamic_ui_info[D_VISION_WHITE_CYCLE].ui_config.end_angel -= 0.2*(dynamic_ui_info[D_VISION_WHITE_CYCLE].ui_config.end_angel - target_white_end_angel);
//      }
//      else//初始化过渡动画结束角度
//      {
//        dynamic_ui_info[D_VISION_WHITE_CYCLE].ui_config.end_angel = target_white_end_angel;
//      }
//      
//      
//      //颜色更新
//      if (current_hp > (0.5*max_hp))
//      {
//        dynamic_ui_info[D_VISION_HP_CYCLE].ui_config.color = GREEN;
//      }
//      else if (current_hp >= 200)
//      {
//        dynamic_ui_info[D_VISION_HP_CYCLE].ui_config.color = YELLOW;
//      }
//      else
//      {
//        dynamic_ui_info[D_VISION_HP_CYCLE].ui_config.color = FUCHSIA;
//      }
//        
//      Enqueue_Ui_For_Sending(&dynamic_ui_info[D_VISION_HP_CYCLE]);
//      Enqueue_Ui_For_Sending(&dynamic_ui_info[D_VISION_ARMOR_CYCLE]);
//      Enqueue_Ui_For_Sending(&dynamic_ui_info[D_VISION_WHITE_CYCLE]);
//    }
//  }
//  else if (dynamic_ui_info[D_VISION_ARMOR_CYCLE].ui_config.start_x != 0)
//  {
//    dynamic_ui_info[D_VISION_ARMOR_CYCLE].ui_config.start_x = 0;
//    dynamic_ui_info[D_VISION_ARMOR_CYCLE].ui_config.start_y = 0;
//    dynamic_ui_info[D_VISION_HP_CYCLE].ui_config.start_x = 0;
//    dynamic_ui_info[D_VISION_HP_CYCLE].ui_config.start_y = 0;
//    dynamic_ui_info[D_VISION_WHITE_CYCLE].ui_config.start_x = 0;
//    dynamic_ui_info[D_VISION_WHITE_CYCLE].ui_config.start_y = 0;
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_VISION_HP_CYCLE]);
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_VISION_ARMOR_CYCLE]);
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_VISION_WHITE_CYCLE]);
//  }
//  last_vision_armor_x = vision_armor_x;
//  last_vision_armor_y = vision_armor_y;
//  #endif
//  /*基地距离********************************************/
//  static float last_base_distance;
//  float target_distance = communicate.game_robot_pos_tx_info->target_distance / 100.F;
//  if (last_base_distance != target_distance)
//  {
//    dynamic_ui_info[D_HIT_TARGET_DISTANCE].ui_config.float_num = target_distance;
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_HIT_TARGET_DISTANCE]);
//  }
//  last_base_distance = target_distance;
//  /*PITCH指针****************************************************/
////	#ifndef UI_SIMPLIFY
//  float pitch_angel=(communicate.car_data1_rx_info->pitch_motor_angle)/180.f*3.14;
//  static float  last_pitch_angel;
//  // 旋转线条
//  if(last_pitch_angel!=pitch_angel)
//  {
//	rotate_point(&dynamic_ui_info[D_PITCH_POINTER].ui_config.start_x,
//                    &dynamic_ui_info[D_PITCH_POINTER].ui_config.start_y,
//                    1260,
//                    540,
//                    CLIENT_MID_POSITION_X,
//                    CLIENT_MID_POSITION_Y,
//                    pitch_angel);
//    rotate_point(&dynamic_ui_info[D_PITCH_POINTER].ui_config.end_x,
//                    &dynamic_ui_info[D_PITCH_POINTER].ui_config.end_y,
//                    1340,
//                    540,
//                    CLIENT_MID_POSITION_X,
//                    CLIENT_MID_POSITION_Y,
//                    pitch_angel);
//  }
//    last_pitch_angel=pitch_angel;
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_PITCH_POINTER]);
////  #endif
//  #ifndef UI_SIMPLIFY
//  /*ROI*/
//  static uint8_t last_roi;
//  uint8_t current_roi = communicate.car_data3_rx_info->uix_lb;
//  if (last_roi != current_roi)
//  {
//    dynamic_ui_info[D_ROI_UP].ui_config.start_x = communicate.car_data3_rx_info->uix_lt * 10;
//    dynamic_ui_info[D_ROI_UP].ui_config.start_y = 1080 - communicate.car_data3_rx_info->uiy_lt * 5;
//    dynamic_ui_info[D_ROI_UP].ui_config.end_x = communicate.car_data3_rx_info->uix_rt * 10;
//    dynamic_ui_info[D_ROI_UP].ui_config.end_y = 1080 - communicate.car_data4_rx_info->uiy_rt * 5;
//    dynamic_ui_info[D_ROI_DOWN].ui_config.start_x = communicate.car_data3_rx_info->uix_lb * 10;
//    dynamic_ui_info[D_ROI_DOWN].ui_config.start_y = 1080 - communicate.car_data3_rx_info->uiy_lb * 5;
//    dynamic_ui_info[D_ROI_DOWN].ui_config.end_x = communicate.car_data3_rx_info->uix_rb * 10;
//    dynamic_ui_info[D_ROI_DOWN].ui_config.end_y = 1080 - communicate.car_data3_rx_info->uiy_rb * 5;
//    dynamic_ui_info[D_ROI_LEFT].ui_config.start_x = communicate.car_data3_rx_info->uix_lt * 10;
//    dynamic_ui_info[D_ROI_LEFT].ui_config.start_y = 1080 - communicate.car_data3_rx_info->uiy_lt * 5;
//    dynamic_ui_info[D_ROI_LEFT].ui_config.end_x = communicate.car_data3_rx_info->uix_lb * 10;
//    dynamic_ui_info[D_ROI_LEFT].ui_config.end_y = 1080 - communicate.car_data3_rx_info->uiy_lb * 5;
//    dynamic_ui_info[D_ROI_RIGHT].ui_config.start_x = communicate.car_data3_rx_info->uix_rt * 10;
//    dynamic_ui_info[D_ROI_RIGHT].ui_config.start_y = 1080 - communicate.car_data4_rx_info->uiy_rt * 5;
//    dynamic_ui_info[D_ROI_RIGHT].ui_config.end_x = communicate.car_data3_rx_info->uix_rb * 10;
//    dynamic_ui_info[D_ROI_RIGHT].ui_config.end_y = 1080 - communicate.car_data3_rx_info->uiy_rb * 5;
//    dynamic_ui_info[D_ROI_MID].ui_config.start_x = communicate.car_data4_rx_info->uix_left * 10;
//    dynamic_ui_info[D_ROI_MID].ui_config.start_y = 1080 - communicate.car_data4_rx_info->uiy_left * 5;
//    dynamic_ui_info[D_ROI_MID].ui_config.end_x = communicate.car_data2_rx_info->uix_right * 10;
//    dynamic_ui_info[D_ROI_MID].ui_config.end_y = 1080 - communicate.car_data2_rx_info->uiy_right * 5;
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_ROI_UP]);
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_ROI_DOWN]);
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_ROI_LEFT]);
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_ROI_RIGHT]);
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[D_ROI_MID]);
//  }
//  last_roi = current_roi;
//  #endif
// 
//}

















///*图形处理**********************************************************************************/

//#include <rp_math.h>

///**
// * @brief 把某点绕某点旋转一定角度
// * 
// * @param x 存储旋转后x的地址
// * @param y 存储旋转后y的地址
// * @param raw_x 旋转前x的值
// * @param raw_y 旋转后y的值
// * @param mid_x 旋转原点x
// * @param mid_y 旋转原点y
// * @param angle 旋转的角度rad
// */
//void rotate_point(__packed uint16_t *x, __packed uint16_t *y, uint16_t raw_x, uint16_t raw_y, float mid_x, float mid_y, float angle) 
//{
//  float s = sin(angle);
//  float c = cos(angle);
//  // 平移到原点
//  float origin_x = raw_x - mid_x;
//  float origin_y = raw_y - mid_y;
//  // 旋转
//  float new_x = origin_x * c - origin_y * s;
//  float new_y = origin_x * s + origin_y * c;
//  // 平移回去并更新原始坐标
//  *x = new_x + mid_x;
//  *y = new_y + mid_y;
//}

///**
// * @brief 按比例放大某点
// * 
// * @param x 存储放大后x的地址
// * @param y 存储放大后y的地址
// * @param raw_x 原始x的值
// * @param raw_y 原始y的值
// * @param mid_x 放大中心点x
// * @param mid_y 放大中心点y
// * @param scale 放大的比例
// */
//void scale_point(uint16_t *x, uint16_t *y, uint16_t raw_x, uint16_t raw_y, float mid_x, float mid_y, float scale) 
//{
//  // 平移到原点
//  float origin_x = raw_x - mid_x;
//  float origin_y = raw_y - mid_y;
//  // 按比例放大
//  float new_x = origin_x * scale;
//  float new_y = origin_y * scale;
//  // 平移回去并更新原始坐标
//  *x = new_x + mid_x;
//  *y = new_y + mid_y;
//}

///**
// * @brief 旋转圆弧
// * 
// * @param start_angle 起始角度 
// * @param end_angel 终止角度
// * @param rotation_angle 
// */
//void rotate_arc(uint16_t *start_angle, uint16_t *end_angel, float rotation_angle) 
//{
//  *start_angle = fmod(*start_angle + rotation_angle, 360);
//  *end_angel = fmod(*end_angel + rotation_angle, 360);
//}

///*其他******************************************/
//void Get_Hp(float *current_hp,float *max_hp,robot_type_e robot_type)
//{
//  //最大血量
//  switch (robot_type)
//  {
//  case HERO:
//    *max_hp = 500;
//    break;
//  case INFANRTY_3:
//  case INFANRTY_4:
//  case INFANRTY_5:
//    *max_hp = 400;
//    break;
//  case OUTPOST:
//    *max_hp = 1500;
//    break;
//  case SENTRY:
//    *max_hp = 400;
//    break;
//  case BASE:
//    *max_hp = 5000;
//    break;
//  default:
//    break;
//  }
//  //判断红蓝
//  if(judge.game_robot_status.robot_id <= 10)//红方
//  {
//    switch (robot_type)
//    {
//    case HERO:
//      *current_hp = judge.game_robot_HP.blue_1_robot_HP;
//      break;
//    case ENGINEER:
//      *current_hp = judge.game_robot_HP.blue_2_robot_HP;
//      break;
//    case INFANRTY_3:
//      *current_hp = judge.game_robot_HP.blue_3_robot_HP;
//      break;
//    case INFANRTY_4:
//      *current_hp = judge.game_robot_HP.blue_4_robot_HP;
//      break;
//    case INFANRTY_5:
//      *current_hp = judge.game_robot_HP.blue_5_robot_HP;
//      break;
//    case OUTPOST:
//      *current_hp = judge.game_robot_HP.blue_outpost_HP;
//      break;
//    case SENTRY:
//      *current_hp = judge.game_robot_HP.blue_7_robot_HP;
//      break;
//    case BASE:
//      *current_hp = judge.game_robot_HP.blue_base_HP;
//      break;
//    default:
//      break;
//    }
//  }
//  else//蓝方
//  {
//    switch (robot_type)
//    {
//    case HERO:
//      *current_hp = judge.game_robot_HP.red_1_robot_HP;
//      break;
//    case ENGINEER:
//      *current_hp = judge.game_robot_HP.red_2_robot_HP;
//      break;
//    case INFANRTY_3:
//      *current_hp = judge.game_robot_HP.red_3_robot_HP;
//      break;
//    case INFANRTY_4:
//      *current_hp = judge.game_robot_HP.red_4_robot_HP;
//      break;
//    case INFANRTY_5:
//      *current_hp = judge.game_robot_HP.red_5_robot_HP;
//      break;
//    case OUTPOST:
//      *current_hp = judge.game_robot_HP.red_outpost_HP;
//      break;
//    case SENTRY:
//      *current_hp = judge.game_robot_HP.red_7_robot_HP;
//      break;
//    case BASE:
//      *current_hp = judge.game_robot_HP.red_base_HP;
//      break;
//    default:
//      break;
//    }
//    
//  }
//}


