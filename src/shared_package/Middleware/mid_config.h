/**
 * @file    mid_config.h
 * @author  Carbon
 * @date    2026-10-04
 * @brief   通信帧 id 总表 —— R2_PC 侧
 *
 * 线格式见 Communication_Interface.hpp：[id][data...][crc16_lo][crc16_hi]，id 全局唯一。
 * id 的唯一来源是本表，各设备在 on_init() 里用下面的枚举自注册。
 * 与下位机固件（R2_Embedded）各持一份，改 id 时两份要一起动。
 */
#pragma once

#include <cstdint>

namespace Middleware
{

enum class CommFrameId : uint8_t
{
    kChassis     = 1,  // 底盘：下行目标轮速 16B / 上行轮反馈 32B，双向
    kLeg         = 3,  // 腿：  下行目标位置 16B / 上行关节反馈 32B，双向
    kTelecontrol = 5,  // 遥控：仅上行，CRSF 摇杆/开关/链路状态
};

} // namespace Middleware
