
#ifndef UNITREE_JOYSTICK_H
#define UNITREE_JOYSTICK_H

#include <stdint.h>

// 16位按键状态联合体（位域形式）
typedef union {
    struct {
        uint8_t R1          :1;  // R1肩键（右后侧）
        uint8_t L1          :1;  // L1肩键（左后侧）
        uint8_t start       :1;  // 开始/菜单键
        uint8_t select      :1;  // 选择/返回键
        uint8_t R2          :1;  // R2扳机键（右前侧）
        uint8_t L2          :1;  // L2扳机键（左前侧）
        uint8_t F1          :1;  // 功能键1（左侧）
        uint8_t F2          :1;  // 功能键2（左侧）
        uint8_t A           :1;  // A键（右下方）
        uint8_t B           :1;  // B键（右下方）
        uint8_t X           :1;  // X键（左下方）
        uint8_t Y           :1;  // Y键（左下方）
        uint8_t up          :1;  // 方向键上
        uint8_t right       :1;  // 方向键右
        uint8_t down        :1;  // 方向键下
        uint8_t left        :1;  // 方向键左
    } components;
    uint16_t value;  // 整型形式访问（按位操作）
} xKeySwitchUnion;

// 40 Byte (now used 24B)
// 手柄完整数据结构（40字节协议）
typedef struct {
    uint8_t head[2];            // 协议头（0x55 0xAA）
    xKeySwitchUnion btn;        // 按键状态组合
    float lx;                   // 左摇杆X轴 [-1.0, 1.0]
    float rx;                   // 右摇杆X轴 [-1.0, 1.0]
    float ry;                   // 右摇杆Y轴 [-1.0, 1.0]
    float L2;                   // L2扳机压力值 [0.0, 1.0]
    float ly;                   // 左摇杆Y轴 [-1.0, 1.0]

    uint8_t idle[16];           // 预留空间（协议对齐）
} xRockerBtnDataStruct;

#endif  // UNITREE_JOYSTICK_H