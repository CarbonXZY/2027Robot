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

} // namespace Device

#endif
