/**
 * @file dvc_motor_instances.h
 * @brief 全车电机实例集中处
 */

#ifndef DVC_MOTOR_INSTANCES_H
#define DVC_MOTOR_INSTANCES_H

#include "dvc_motor_dji.h"

namespace Device
{

/**
 * @brief 全车电机实例集中结构体
 *
 * 成员保持具体类型，watch 一个 g_motor 即可展开全部电机状态。
 */
struct MotorInstances
{
    // 腿 4 关节，与上位机 joints 一一对应
    MotorDjiC620 leg_front_left {&hfdcan1, MotorDjiId::kId0x201};
    MotorDjiC620 leg_front_right{&hfdcan1, MotorDjiId::kId0x202};
    MotorDjiC620 leg_rear_left  {&hfdcan1, MotorDjiId::kId0x203};
    MotorDjiC620 leg_rear_right {&hfdcan1, MotorDjiId::kId0x204};

    // 底盘 4 轮，与上位机 joints 一一对应（前左/前右/后左/后右）
    MotorDjiC620 chassis_front_left {&hfdcan1, MotorDjiId::kId0x205};
    MotorDjiC620 chassis_front_right{&hfdcan1, MotorDjiId::kId0x206};
    MotorDjiC620 chassis_rear_left  {&hfdcan1, MotorDjiId::kId0x207};
    MotorDjiC620 chassis_rear_right {&hfdcan1, MotorDjiId::kId0x208};

    // 夹爪（M2006），独立挂在 CAN2
    MotorDjiC610 clamp{&hfdcan2, MotorDjiId::kId0x201};

    // Earth Block 导轨（M2006+C610），挂在 CAN2；CAN2 0x201 已被夹爪占用
    MotorDjiC610 earth_block{&hfdcan2, MotorDjiId::kId0x202};

    // Sky Block：大臂 C610、小臂 C620、齿条夹爪 C610，统一挂在 CAN2
    MotorDjiC610 sky_block_big_arm{&hfdcan2, MotorDjiId::kId0x203};
    MotorDjiC620 sky_block_small_arm{&hfdcan2, MotorDjiId::kId0x204};
    MotorDjiC610 sky_block_gripper{&hfdcan2, MotorDjiId::kId0x205};

    void Init();
};

extern MotorInstances g_motor;

} // namespace Device

#endif
