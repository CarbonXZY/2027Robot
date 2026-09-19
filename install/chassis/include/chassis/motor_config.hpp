// motor_config.hpp
#pragma once

namespace chassis
{

// 电机数据结构体模板（Send_Interface 线格式：每轮一帧 [id][MotorData]，共 8 字节）
// tx（下发指令）：只用 velocity，position 填 0
// rx（电机回传）：velocity + position 均由 MCU 填充
struct MotorData
{
    float velocity = 0.0f;  // 速度 (rad/s)
    float position = 0.0f;  // 位置 (rad)
};

}  // namespace chassis
