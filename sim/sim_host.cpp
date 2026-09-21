// sim_host.cpp
//
// 主机侧仿真：用真的 Class_Communication_Interface（全局实例 USB_Communication_Interface）
// 收发结构体，另一端是 sim_slave.cpp 里的模拟从机。中间没有真 USB，
// 由 Send_To_Slave() 这一层「假传输」把两边的字节接起来：
//
//   主机 Send() --打包--> 字节 --> 模拟从机解析/回包 --> 字节 --> 主机 Rx_RptlCallback()
//
#include <cstdio>
#include <cstring>

#include <Communication_Interface.hpp>
#include <motor_base.hpp>

#include "sim_slave.hpp"

namespace
{

using Control_Frame::Struct_Motor_Base;
using Control_Frame::USB_Communication_Interface;

constexpr size_t kWheels  = 4;
constexpr size_t kNoChange = static_cast<size_t>(-1);

// 下行：4 个轮子的目标速度
struct Struct_Tx
{
    float velocity[kWheels];
};

// 上行：4 个轮子的回传
struct Struct_Rx
{
    Struct_Motor_Base motor[kWheels];
};

Struct_Tx Tx;
Struct_Rx Rx;

// 人工注入线路错误：把从机回包的第 n 个字节改掉，模拟线上被打坏
size_t  g_rx_corrupt_at    = kNoChange;
uint8_t g_rx_corrupt_value = 0;

// 线格式（主机侧视角，仅用于算偏移）：
//   下行一帧 [id][float(4)][crc16(2)] = 7 字节
//   上行一帧 [id][velocity(4)][position(4)][crc16(2)] = 11 字节
constexpr size_t kUpFrame = 1 + sizeof(Struct_Motor_Base) + 2;

size_t Up_Offset(size_t wheel) { return wheel * kUpFrame; }

void Dump_Hex(const char * tag, const uint8_t * data, size_t len)
{
    std::printf("      %s (%zu 字节):", tag, len);
    for (size_t i = 0; i < len; ++i)
    {
        std::printf(" %02X", data[i]);
    }
    std::printf("\n");
}

// 中间件的发送回调。不做真 USB：把打包好的字节直接喂给模拟从机，
// 再把从机的回包喂回 Rx_RptlCallback()，模拟一次「发出去 → 设备回话」。
int64_t Send_To_Slave(uint8_t * buf, size_t len)
{
    Dump_Hex("下行", buf, len);

    uint8_t reply[256];
    size_t reply_len = Sim_Slave_Handle(buf, len, reply, sizeof(reply));

    if (g_rx_corrupt_at < reply_len)
    {
        reply[g_rx_corrupt_at] = g_rx_corrupt_value;
    }
    Dump_Hex("上行", reply, reply_len);

    USB_Communication_Interface.Rx_RptlCallback(reply, static_cast<uint16_t>(reply_len));
    return static_cast<int64_t>(len);
}

int g_failed = 0;

void Check(const char * name, bool ok)
{
    std::printf("      %-30s %s\n", name, ok ? "ok" : "FAIL");
    if (!ok)
    {
        ++g_failed;
    }
}

bool Same_Bits(float a, float b)
{
    return std::memcmp(&a, &b, sizeof(float)) == 0;
}

float Rx_Velocity(size_t wheel) { return Rx.motor[wheel].velocity; }

void Capture(float * dst)
{
    for (size_t k = 0; k < kWheels; ++k)
    {
        dst[k] = Rx_Velocity(k);
    }
}

void Set_Targets(float value)
{
    for (size_t k = 0; k < kWheels; ++k)
    {
        Tx.velocity[k] = value;
    }
}

// 期望：每帧的 velocity 位精确回到对应槽位，position 按 id 落位
bool Velocity_Is(const float * expect)
{
    for (size_t k = 0; k < kWheels; ++k)
    {
        if (!Same_Bits(Rx_Velocity(k), expect[k]))
        {
            return false;
        }
    }
    return true;
}

}  // namespace

int main()
{
    USB_Communication_Interface.Init(&Send_To_Slave);

    for (size_t k = 0; k < kWheels; ++k)
    {
        const uint8_t id = static_cast<uint8_t>(k + 1);
        if (!USB_Communication_Interface.Register(id, &Tx.velocity[k], &Rx.motor[k],
                                                  sizeof(float), sizeof(Struct_Motor_Base)))
        {
            std::printf("Register id=%u 失败\n", id);
            return 1;
        }
    }

    // 重复注册应当被拒绝
    std::printf("[0] 重复 id 注册被拒\n");
    Check("id=2 再注册一次返回 false",
          !USB_Communication_Interface.Register(2, &Tx.velocity[1], &Rx.motor[1],
                                                sizeof(float), sizeof(Struct_Motor_Base)));

    std::printf("\n[1] 位精确往返（含 -0.0 / 大数 / 小数）\n");
    {
        const float values[kWheels] = {0.0f, -0.0f, 123456.789f, -1.5e-7f};
        for (size_t k = 0; k < kWheels; ++k)
        {
            Tx.velocity[k] = values[k];
        }
        USB_Communication_Interface.Send();
        Check("velocity 逐位原样回来", Velocity_Is(values));
    }

    std::printf("\n[2] position 按 id 落到正确槽位\n");
    {
        const float expect[kWheels] = {1000.25f, 2000.25f, 3000.25f, 4000.25f};
        bool ok = true;
        for (size_t k = 0; k < kWheels; ++k)
        {
            ok = ok && Same_Bits(Rx.motor[k].position, expect[k]);
        }
        Check("4 个槽位互不串位", ok);
    }

    std::printf("\n[3] 回包里坏一个数据字节\n");
    {
        const float before[kWheels] = {0.0f, -0.0f, 123456.789f, -1.5e-7f};

        Set_Targets(9.0f);
        g_rx_corrupt_at    = Up_Offset(1) + 1;  // 第 2 个轮子回包的第一个数据字节
        g_rx_corrupt_value = 0xFF;
        USB_Communication_Interface.Send();
        g_rx_corrupt_at = kNoChange;

        const float expect[kWheels] = {9.0f, before[1], 9.0f, 9.0f};
        Check("坏帧被丢、同包其它帧照收", Velocity_Is(expect));
    }

    std::printf("\n[4] 回包里坏一个 id 字节（改成没注册的 id）\n");
    {
        float before[kWheels];
        Capture(before);

        Set_Targets(7.0f);
        g_rx_corrupt_at    = Up_Offset(1);  // 第 2 个轮子回包的 id 字节
        g_rx_corrupt_value = 0x7F;          // 不是任何已注册的 id
        USB_Communication_Interface.Send();
        g_rx_corrupt_at = kNoChange;

        const float expect[kWheels] = {7.0f, before[1], before[2], before[3]};
        Check("坏 id 之后的帧全丢（预期行为）", Velocity_Is(expect));
    }

    std::printf("\n[5] 回包里坏一个 id 字节（改成另一个已注册的 id）\n");
    {
        float before[kWheels];
        Capture(before);

        Set_Targets(8.0f);
        g_rx_corrupt_at    = Up_Offset(1);  // 第 2 个轮子回包的 id 字节
        g_rx_corrupt_value = 3;             // 已注册，且 rx_size 与被顶掉的那帧相同
        USB_Communication_Interface.Send();
        g_rx_corrupt_at = kNoChange;

        // 帧长照样对，只有这一帧 CRC 不过被丢，后面的帧不受影响
        const float expect[kWheels] = {8.0f, before[1], 8.0f, 8.0f};
        Check("只丢这一帧，后面照常", Velocity_Is(expect));
    }

    std::printf("\n%s（失败 %d 项）\n", g_failed == 0 ? "全部通过" : "有失败", g_failed);
    return g_failed == 0 ? 0 : 1;
}
