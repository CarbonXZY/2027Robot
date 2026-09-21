// sim_slave.cpp
#include "sim_slave.hpp"

#include <cstring>

namespace
{

// —— 独立实现一遍线格式 ——
// 一包 = 一批帧：[id][data...][crc16_lo][crc16_hi]
// CRC16/CCITT-FALSE：poly 0x1021，init 0xFFFF，不反转，xorout 0
// 覆盖范围：[id][data...]

constexpr size_t kIdSize  = 1;
constexpr size_t kCrcSize = 2;

// 下行（主机 → 从机）：目标速度
constexpr size_t kDownSize = sizeof(float);

// 上行（从机 → 主机）：速度 + 位置。
// 故意不 include motor_base.hpp 的 Struct_Motor_Base，自己按一样的布局写一遍，
// 用来验证两边对 struct 布局的理解是否真的一致。
struct Motor_Value
{
    float velocity;
    float position;
};
constexpr size_t kUpSize = sizeof(Motor_Value);

constexpr uint8_t kIds[] = {1, 2, 3, 4};
constexpr size_t  kCount = sizeof(kIds) / sizeof(kIds[0]);

uint16_t Crc16Ccitt(const uint8_t * data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

// 从机模拟的电机行为：速度原样回显（这样主机可以做位精确比对），
// 位置按 id 给一个固定值（这样主机能验证每一帧落到了正确的槽位）。
float Sim_Position(uint8_t id)
{
    return static_cast<float>(id) * 1000.0f + 0.25f;
}

}  // namespace

size_t Sim_Slave_Handle(const uint8_t * in, size_t in_len, uint8_t * out, size_t out_cap)
{
    (void)out_cap;

    // 1) 解主机下发的一包：帧长在这里是固定的（1 + 4 + 2），逐帧往前走
    float target[kCount] = {};
    bool  got[kCount]    = {};

    const size_t down_frame = kIdSize + kDownSize + kCrcSize;
    for (size_t i = 0; i + down_frame <= in_len; i += down_frame)
    {
        const uint8_t id = in[i];
        for (size_t k = 0; k < kCount; ++k)
        {
            if (kIds[k] != id)
            {
                continue;
            }
            const uint16_t expect = static_cast<uint16_t>(in[i + kIdSize + kDownSize] |
                                                          (static_cast<uint16_t>(in[i + kIdSize + kDownSize + 1]) << 8));
            if (Crc16Ccitt(in + i, kIdSize + kDownSize) == expect)
            {
                std::memcpy(&target[k], in + i + kIdSize, kDownSize);
                got[k] = true;
            }
        }
    }

    // 2) 构造回包：每个 id 一帧 [id][velocity][position][crc16]
    size_t p = 0;
    for (size_t k = 0; k < kCount; ++k)
    {
        Motor_Value value;
        value.velocity = got[k] ? target[k] : 0.0f;
        value.position = Sim_Position(kIds[k]);

        const size_t start = p;
        out[p++] = kIds[k];
        std::memcpy(out + p, &value, kUpSize);
        p += kUpSize;

        const uint16_t crc = Crc16Ccitt(out + start, kIdSize + kUpSize);
        out[p++] = static_cast<uint8_t>(crc & 0xFF);
        out[p++] = static_cast<uint8_t>(crc >> 8);
    }
    return p;
}
