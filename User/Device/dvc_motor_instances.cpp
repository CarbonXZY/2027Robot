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
// 找限位阶段: 只用速度环
constexpr MotorDjiC620::Parameters kLegParameters
{
    .pid_omega = {.k_p = 2.0f}
};
constexpr MotorDjiC620::Parameters kChassisParameters
{
    .pid_omega = {.k_p = 2.0f}
};
} // namespace

MotorInstances g_motor;

void MotorInstances::Init()
{
    leg_front_left.Init(MotorControlMethod::kSpeed, kLegParameters);
    leg_front_right.Init(MotorControlMethod::kSpeed, kLegParameters);
    leg_rear_left.Init(MotorControlMethod::kSpeed, kLegParameters);
    leg_rear_right.Init(MotorControlMethod::kSpeed, kLegParameters);

    chassis_front_left.Init(MotorControlMethod::kSpeed, kChassisParameters);
    chassis_front_right.Init(MotorControlMethod::kSpeed, kChassisParameters);
    chassis_rear_left.Init(MotorControlMethod::kSpeed, kChassisParameters);
    chassis_rear_right.Init(MotorControlMethod::kSpeed, kChassisParameters);
}
} // namespace Device
