#pragma once
#include <cstddef>
#include <cstdint>

namespace smotor::cmd::rmd {
constexpr size_t IDX_FRAME_HEAD = 0;
constexpr size_t IDX_CMD = 1;
constexpr size_t IDX_ID = 2;
constexpr size_t IDX_DATA_LEN = 3;
constexpr size_t IDX_HEAD_CHK = 4;
constexpr size_t IDX_DATA_START = 5;
constexpr size_t FRAME_LEN = 5;
constexpr uint8_t FRAME_HEAD = 0x3E;

constexpr double MAX_SPEED = 20000.0;

// Control
constexpr uint8_t MOTOR_START = 0x88;
constexpr uint8_t MOTOR_PAUSE = 0x81;
constexpr uint8_t MOTOR_SHUTDOWN = 0x80;

// R: Read, W: Write
// [R/W]_[Target]_[Dst]
// PID
constexpr uint8_t R_PID = 0x30;
constexpr uint8_t W_PID_RAM = 0x31;
constexpr uint8_t W_PID_ROM = 0x32;

// ACC
constexpr uint8_t R_ACC = 0x33;
constexpr uint8_t W_ACC = 0x34;

// Encoder
constexpr uint8_t R_ENCODER = 0x90;
constexpr uint8_t W_ENCODER_ROM = 0x91;
constexpr uint8_t W_CURPOS_ROM = 0x19;

// Angle
constexpr uint8_t R_MULTILOOPANG = 0x92; // 当前电机的多圈绝对值角度
constexpr uint8_t R_SINGALLOOPANG = 0x94; // 当前电机的单圈绝对值角度
constexpr uint8_t W_DEFANG = 0x95;

// Error & State
constexpr uint8_t R_ERROR = 0x9A;
constexpr uint8_t W_ERROR = 0x9B;
constexpr uint8_t R_STATE1 = 0x9A;
constexpr uint8_t R_STATE2 = 0x9C;
constexpr uint8_t R_STATE3 = 0x9D;

// Close Loop Control
// 输出指定功率
constexpr uint8_t W_POWER = 0xA0;
// 输出指定速度
constexpr uint8_t W_SPEED = 0xA2;

// Angle Control (1：默认速度，2：指定速度)
// 旋转到特定角度，这里有考虑从上电开始电机全部旋转角度
// 比如电机上电后旋转四圈后使用下面指令旋转到 0°，会让电机往回转四圈
// -360 ~ 360
constexpr uint8_t W_MLANG1 = 0xA3;
constexpr uint8_t W_MLANG2 = 0xA4;
// 旋转到特定角度，不考虑电机已经旋转了多少圈
// 比如电机上电后旋转了四圈多 10°
// 在不指定倒转的情况下会旋转 350° 变成第五圈的 0 °
// 0 ~ 360 (允许倒转)
constexpr uint8_t W_SLANG1 = 0xA5;
constexpr uint8_t W_SLANG2 = 0xA6;
// 从当前位置开始再选转指定角度
// -360 ~ 360
constexpr uint8_t W_ADDANG1 = 0xA7; // 默认速度旋转指定角度 0 ~ 360
constexpr uint8_t W_ADDANG2 = 0xA8; // 指定速度旋转指定角度 0 ~ 360

constexpr uint8_t R_INFO = 0x12;
};