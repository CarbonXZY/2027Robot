/**
 * @file    motor_dji.cpp
 * @author  lyh
 * @date    2026-10-02
 * @brief   
 */
#include "motor_dji.h"

using namespace Device;

namespace motor_test
{
MotorDjiC620 m3508_test(&hfdcan1, MotorDjiId::kId0x201);

void Init()
{
    MotorDjiC620::Parameters params = {
        .pid_position = {.k_p = 0, .k_i = 0, .k_d = 0},
        .pid_omega = {.k_p = 3, .k_i = 0, .k_d = 0}
    };
    m3508_test.Init(MotorControlMethod::kSpeed, params);
    m3508_test.SetTargetSpeed(0);
}

void Timer1msCalculateCallback()
{
    m3508_test.Calculate();
}

void Timer100msAliveCallback()
{
    m3508_test.Tim100msAlivePeriodElapsedCallback();
}

void CanRxCallBack()
{
    m3508_test.FdcanRxCpltCallback(nullptr);
}
}
