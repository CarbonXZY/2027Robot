/**
 * @file dvc_motor_instances.cpp
 * @brief 全车电机实例集中处
 *
 * 4 个腿关节电机在定义处直接构造，外设句柄在构造函数里绑定，
 * 构造只做指针比较，不解引用句柄，因此静态初始化期安全。
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

MotorDjiC620 g_leg_front_left(&hfdcan1, MotorDjiId::kId0x201);
MotorDjiC620 g_leg_front_right(&hfdcan1, MotorDjiId::kId0x202);
MotorDjiC620 g_leg_rear_left(&hfdcan1, MotorDjiId::kId0x203);
MotorDjiC620 g_leg_rear_right(&hfdcan1, MotorDjiId::kId0x204);

MotorDjiC610 g_motor_clamp(&hfdcan2, MotorDjiId::kId0x201);

// 底盘 4 轮，接在腿后面（M3508 标准 ID 段 0x201~0x208）
MotorDjiC620 g_chassis_front_left(&hfdcan1, MotorDjiId::kId0x205);
MotorDjiC620 g_chassis_front_right(&hfdcan1, MotorDjiId::kId0x206);
MotorDjiC620 g_chassis_rear_left(&hfdcan1, MotorDjiId::kId0x207);
MotorDjiC620 g_chassis_rear_right(&hfdcan1, MotorDjiId::kId0x208);

void InitMotorInstances()
{
    g_leg_front_left.Init(MotorControlMethod::kPosition, kLegParameters);
    g_leg_front_right.Init(MotorControlMethod::kPosition, kLegParameters);
    g_leg_rear_left.Init(MotorControlMethod::kPosition, kLegParameters);
    g_leg_rear_right.Init(MotorControlMethod::kPosition, kLegParameters);

    g_chassis_front_left.Init(MotorControlMethod::kSpeed, kChassisParameters);
    g_chassis_front_right.Init(MotorControlMethod::kSpeed, kChassisParameters);
    g_chassis_rear_left.Init(MotorControlMethod::kSpeed, kChassisParameters);
    g_chassis_rear_right.Init(MotorControlMethod::kSpeed, kChassisParameters);
}
} // namespace Device
