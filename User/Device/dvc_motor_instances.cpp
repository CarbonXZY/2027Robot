/**
 * @file dvc_motor_instances.cpp
 * @brief 全车电机实例集中处
 */

#include "dvc_motor_instances.h"

#include "drv_can.h"

namespace Device
{
namespace
{
// PID 参数空壳：整定时直接改这里，腿走位置环，底盘走速度环
constexpr MotorDjiC620::Parameters kLegParameters{};
constexpr MotorDjiC620::Parameters kChassisParameters
{
    .pid_omega = {.k_p = 2.0f}
};
} // namespace

MotorInstances g_motor;

void MotorInstances::Init()
{
    leg_front_left.Init(MotorControlMethod::kPosition, kLegParameters);
    leg_front_right.Init(MotorControlMethod::kPosition, kLegParameters);
    leg_rear_left.Init(MotorControlMethod::kPosition, kLegParameters);
    leg_rear_right.Init(MotorControlMethod::kPosition, kLegParameters);

    chassis_front_left.Init(MotorControlMethod::kSpeed, kChassisParameters);
    chassis_front_right.Init(MotorControlMethod::kSpeed, kChassisParameters);
    chassis_rear_left.Init(MotorControlMethod::kSpeed, kChassisParameters);
    chassis_rear_right.Init(MotorControlMethod::kSpeed, kChassisParameters);
}
} // namespace Device
