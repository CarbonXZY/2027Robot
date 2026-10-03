/**
 * @file dvc_motor_instances.h
 * @brief 全车电机实例集中处
 */

#ifndef DVC_MOTOR_INSTANCES_H
#define DVC_MOTOR_INSTANCES_H

#include "dvc_motor_dji.h"

namespace Device
{

// 腿 4 关节，与上位机 joints 一一对应
extern MotorDjiC620 g_leg_front_left;
extern MotorDjiC620 g_leg_front_right;
extern MotorDjiC620 g_leg_rear_left;
extern MotorDjiC620 g_leg_rear_right;

extern MotorDjiC610 g_motor_clamp;

// 底盘 4 轮，与上位机 joints 一一对应（前左/前右/后左/后右）
extern MotorDjiC620 g_chassis_front_left;
extern MotorDjiC620 g_chassis_front_right;
extern MotorDjiC620 g_chassis_rear_left;
extern MotorDjiC620 g_chassis_rear_right;

/**
 * @brief 写入各电机的控制模式与 PID 参数
 *
 * 放在设备层是为了让 Module::Leg / Module::Chassis 只依赖 MotorBase，
 * 不感知 C620 的具体参数结构，保持解耦。
 */
void InitMotorInstances();

} // namespace Device

#endif
