// motor_base.hpp
#pragma once

#include <cstdint>

namespace Device
{

#pragma pack(push, 1)
struct Struct_Motor_Base
{
    float velocity = 0.0f;  // 速度 (rad/s)
    float position = 0.0f;  // 位置 (rad)
    uint8_t alive_flag = 0;
};
#pragma pack(pop)

}  // namespace Device
