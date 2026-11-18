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

// Earth Block 导轨: 位置环 + 速度环; 法一的大误差定速接近由 Module 层切换控制模式
constexpr MotorDjiC610::Parameters kEarthBlockParameters
{
    .pid_position = {.k_p = 10.0f, .k_i = 0.0f, .k_d = 0.0f, .out_max = 20.0f},
    .pid_omega    = {.k_p = 3.5f, .k_i = 0.0f, .k_d = 0.0f, .out_max = 10.0f},
};
} // namespace

MotorInstances g_motor;

void MotorInstances::Init()
{
    // leg_front_left.Init(MotorControlMethod::kSpeed, kLegParameters);
    // leg_front_right.Init(MotorControlMethod::kSpeed, kLegParameters);
    // leg_rear_left.Init(MotorControlMethod::kSpeed, kLegParameters);
    // leg_rear_right.Init(MotorControlMethod::kSpeed, kLegParameters);

    chassis_front_left.Init(MotorControlMethod::kSpeed, kChassisParameters);
    chassis_front_right.Init(MotorControlMethod::kSpeed, kChassisParameters);
    chassis_rear_left.Init(MotorControlMethod::kSpeed, kChassisParameters);
    chassis_rear_right.Init(MotorControlMethod::kSpeed, kChassisParameters);

    // Earth Block 导轨: 先配置位置环参数, 接线后由 Module::EarthBlock 接管
    earth_block.Init(MotorControlMethod::kPosition, kEarthBlockParameters);
}
} // namespace Device
