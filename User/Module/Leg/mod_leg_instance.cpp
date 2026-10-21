/**
* @file mod_leg_instance.cpp
* @author lyh
* @date 2026-10-20
* @brief 
*
*/
#include "mod_leg_instance.h"
#include "dvc_motor_instances.h"


namespace
{
//
using namespace Module;

const Leg::Mechanical g_leg_mechanical = {
    .equivalent_radius = {0.025f, 0.025f, 0.025f, 0.025f},
    .direction_sign = {1, -1, -1, 1},
    .max_height = {0.3f, 0.3f},
};

} // namespace

namespace Module
{
//
using namespace Device;

// 顺序固定: lf, rf, lr, rr
Leg g_legs(Leg::MotorGroup{
    .lf = &g_motor.leg_front_left,
    .rf = &g_motor.leg_front_right,
    .lr = &g_motor.leg_rear_left,
    .rr = &g_motor.leg_rear_right,
    }, g_leg_mechanical);

Leg::Homing g_legs_homing = {
    .speed = 0.05f,
    .speed_threshold = 0.001f,
    .debounce_ms = 20,
};

} // namespace Module

void LegTest()
{
    using namespace Module;
    
}

