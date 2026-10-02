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
// 腿关节串级位置环，两环都只给 P
constexpr MotorDjiC620::Parameters kLegMotorParameters{
    .pid_position = {.k_p = 1.0f, .out_max = 3.0f},
    .pid_omega = {.k_p = 0.1f, .out_max = 0.5f},
};
} // namespace

MotorDjiC620 g_leg_front_left(&hfdcan1, MotorDjiId::kId0x201);
MotorDjiC620 g_leg_front_right(&hfdcan1, MotorDjiId::kId0x202);
MotorDjiC620 g_leg_rear_left(&hfdcan1, MotorDjiId::kId0x203);
MotorDjiC620 g_leg_rear_right(&hfdcan1, MotorDjiId::kId0x204);
} // namespace Device
