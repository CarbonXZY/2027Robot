// motor_base.hpp
#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

namespace Control_Frame
{

#pragma pack(push, 1)
struct Struct_Motor_Base
{
    float velocity = 0.0f;  // 速度 (rad/s)
    float position = 0.0f;  // 位置 (rad)
    uint8_t alive_flag = 0;
};
#pragma pack(pop)

// 通用模板：从参数表里读一个参数，找不到返回默认值
template <typename T>
inline T ReadParam(const std::unordered_map<std::string, std::string> & params,
                   const char * key, T def)
{
    const auto it = params.find(key);
    if (it == params.end())
    {
        return def;
    }
    return static_cast<T>(std::stod(it->second));
}

}  // namespace Control_Frame
