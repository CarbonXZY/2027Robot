/**
 * @file    motor_dji.h
 * @author  lyh
 * @date    2026-10-02
 * @brief   
 */
#ifndef MOTOR_DJI_H
#define MOTOR_DJI_H

#include "dvc_motor_dji.h"

namespace motor_test
{
void Init();
void Timer1msCalculateCallback();
void Timer100msAliveCallback();
void CanRxCallBack();

extern Device::MotorDjiC620 g_motor_test;
}

#endif // MOTOR_DJI_H
