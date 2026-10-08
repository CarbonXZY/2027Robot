/**
 * @file mod_chassis.cpp
 * @brief 麦克纳姆底盘（4 轮）控制，与上位机 R2_PC 的 chassis_system 配对
 *
 * 职责：
 *   下行：把上位机下发的目标轮速（tx_buffer_.velocity[i]）写进各轮电机的
 *         速度环目标，走速度模式。
 *   上行：把各轮电机的反馈（速度 / 位置）填进 rx_buffer_，由通信中间件打包回传。
 *
 * 底盘帧（id 见 Middleware/mid_config.h 的 CommFrameId::kChassis）在本类 Init() 里自注册，
 * 注意中间件 Register() 的 tx/rx 是站在本板视角，与结构体名的电机视角是反的，别接错：
 *   本板发（tx）= rx_buffer_（反馈，32B）
 *   本板收（rx）= tx_buffer_（指令，16B）
 */

#include "mod_chassis.h"

#include <cstddef>
#include <cstdint>

#include "Communication_Interface.hpp"
#include "mid_config.h"

namespace Module
{

Chassis::Chassis(std::array<Device::MotorBase *, kChassisCount> chassis_motor)
    : chassis_motor_(chassis_motor)
{
}

void Chassis::Init(const Parameters &parameters)
{
    param_ = parameters;

    for (uint8_t i = 0; i < kChassisCount; ++i)
    {
        // 方向归一到 ±1，0 视为正向
        param_.direction_sign[i] = (param_.direction_sign[i] < 0) ? -1 : 1;

        if (chassis_motor_[i] == nullptr)
        {
            continue;
        }

        chassis_motor_[i]->SetControlMethod(Device::MotorControlMethod::kSpeed);
    }

    Middleware::g_usb_communication_interface.Register(
        static_cast<uint8_t>(Middleware::CommFrameId::kChassis),
        &rx_buffer_, &tx_buffer_,
        static_cast<uint8_t>(sizeof(rx_buffer_)), static_cast<uint8_t>(sizeof(tx_buffer_)));
}

void Chassis::SetState(ChassisStatus status)
{
    status_ = status;
}

void Chassis::TimCalculatePeriodElapsedCallback()
{
    for (uint8_t i = 0; i < kChassisCount; ++i)
    {
        if (chassis_motor_[i] == nullptr)
        {
            continue;
        }

        // 下行：上位机目标轮速 → 电机侧（乘方向）
        const float direction = (param_.direction_sign[i] < 0) ? -1.0f : 1.0f;

        // 失能时给 0（PC 掉线保护）
        if (status_ == ChassisStatus::kDisable)
        {
            chassis_motor_[i]->SetTargetSpeed(0.0f);
        }
        else
        {
            chassis_motor_[i]->SetTargetSpeed(direction * tx_buffer_.velocity[i]);
        }

        // 反馈更新 → PID → 写 CAN 发送缓冲
        // 实际发帧由 Driver::Tim1msCanPeriodElapsedCallback() 统一做，不在这里发
        chassis_motor_[i]->Calculate();

        // 上行：电机反馈 → 回传结构体
        rx_buffer_.motor[i].position = direction * chassis_motor_[i]->GetPosition();
        rx_buffer_.motor[i].velocity = direction * chassis_motor_[i]->GetSpeed();
    }
}

} // namespace Module
