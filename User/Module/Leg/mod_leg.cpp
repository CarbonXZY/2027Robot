/**
 * @file mod_leg.cpp
 * @brief 腿机构（4 关节）控制，与上位机 R2_PC 的 leg_system 配对
 *
 * 职责：
 *   下行：把上位机下发的目标位置（tx_buffer_.position[i]）写进各腿电机的
 *         位置环目标，走位置模式。
 *   上行：把各腿电机的反馈（输出轴角度 rad / 角速度 rad/s）填进 rx_buffer_，
 *         由通信中间件打包回传上位机。
 *
 * id=3 的两帧在本类 Init() 里自注册，注意中间件 Register() 的 tx/rx 是站在
 * 本板视角，与结构体名的电机视角是反的，别接错：
 *   本板发（tx）= rx_buffer_（反馈，32B）
 *   本板收（rx）= tx_buffer_（指令，16B）
 */

#include "mod_leg.h"

#include <cstddef>
#include <cstdint>

#include "Communication_Interface.hpp"

namespace Module
{
namespace
{
// 腿帧 id，须与上位机 URDF 的 leg_id 一致（robot.urdf: <param name="leg_id">3</param>）。
// 同一 id 只能注册一次，别处若也注册 id=3 会被 first-match-wins 遮住。
constexpr uint8_t kIdLeg = 3;
} // namespace

Leg::Leg(std::array<Device::MotorBase *, kLegCount> leg_motor)
    : leg_motor_(leg_motor)
{
}

void Leg::Init(const Parameters &parameters)
{
    param_ = parameters;

    for (uint8_t i = 0; i < kLegCount; ++i)
    {
        // 方向归一到 ±1，0 视为正向
        param_.direction_sign[i] = (param_.direction_sign[i] < 0) ? -1 : 1;

        if (leg_motor_[i] == nullptr)
        {
            continue;
        }

        leg_motor_[i]->SetControlMethod(Device::MotorControlMethod::kPosition);
    }

    Middleware::g_usb_communication_interface.Register(
        kIdLeg,
        &rx_buffer_, &tx_buffer_,
        static_cast<uint8_t>(sizeof(rx_buffer_)), static_cast<uint8_t>(sizeof(tx_buffer_)));
}

void Leg::MoveToPosition()
{
    for (uint8_t i = 0; i < kLegCount; ++i)
    {
        if (leg_motor_[i] == nullptr)
        {
            continue;
        }

        // 上位机下发的是关节侧目标位置 (rad)
        target_position_[i] = tx_buffer_.position[i];

        const float direction = (param_.direction_sign[i] < 0) ? -1.0f : 1.0f;

        // 关节侧位置误差
        const float error = target_position_[i] - feedback_position_[i];

        if (Driver::MathAbs(error) > param_.position_lock_threshold)
        {
            // 远距离：速度模式开环逼近
            // 电机侧速度 = 方向 × 关节侧目标速度
            leg_motor_[i]->SetControlMethod(Device::MotorControlMethod::kSpeed);
            leg_motor_[i]->SetTargetSpeed(direction * ((error > 0.0f) ? param_.max_velocity : -param_.max_velocity));
        }
        else
        {
            // 近距离：切位置环，锁定到目标
            // 关节角 → 电机侧原始角: raw = 方向 × 关节角 + 零点
            leg_motor_[i]->SetControlMethod(Device::MotorControlMethod::kPosition);
            leg_motor_[i]->SetTargetPosition(direction * target_position_[i] + offset_[i]);
        }
    }
}

void Leg::TimCalculatePeriodElapsedCallback()
{
    if (!calibrated_)
    {
        Calibrate();
    }
    else
    {
        MoveToPosition();
    }

    for (uint8_t i = 0; i < kLegCount; ++i)
    {
        if (leg_motor_[i] == nullptr)
        {
            continue;
        }

        // 反馈更新 → PID → 写 CAN 发送缓冲
        // 实际发帧由 Driver::Tim1msCanPeriodElapsedCallback() 统一做，不在这里发
        leg_motor_[i]->Calculate();

        // 上行：电机反馈 → 关节侧（乘方向）→ 回传结构体
        const float direction = (param_.direction_sign[i] < 0) ? -1.0f : 1.0f;
        feedback_position_[i] = direction * (leg_motor_[i]->GetPosition() - offset_[i]);
        feedback_velocity_[i] = direction * leg_motor_[i]->GetSpeed();
        rx_buffer_.motor[i].position = feedback_position_[i];
        rx_buffer_.motor[i].velocity = feedback_velocity_[i];
    }
}

void Leg::Calibrate()
{
    bool all_done = true;

    for (uint8_t i = 0; i < kLegCount; ++i)
    {
        if (leg_motor_[i] == nullptr || calibrated_motor_[i])
        {
            continue;
        }

        Device::CalibrateParams calib = param_.calibrate;
        calib.motion_value *= static_cast<float>(param_.direction_sign[i]);

        float offset = 0.0f;
        if (leg_motor_[i]->Calibrate(calib, offset))
        {
            offset_[i] = offset;
            calibrated_motor_[i] = true;
        }
        else
        {
            all_done = false;
        }
    }

    calibrated_ = all_done;
}

} // namespace Module
