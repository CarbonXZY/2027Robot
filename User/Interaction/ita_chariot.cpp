/**
 * @file ita_chariot.cpp
 * @brief 整车组合：把各机构绑到一起，对上层只暴露几个回调
 */

#include "ita_chariot.h"

#include "Communication_Interface.hpp"
#include "drv_can.h"
#include "dvc_motor_instances.h"

Chariot::Chariot()
    : leg_({&Device::g_leg_front_left, &Device::g_leg_front_right, &Device::g_leg_rear_left, &Device::g_leg_rear_right}),
      chassis_({&Device::g_chassis_front_left, &Device::g_chassis_front_right, &Device::g_chassis_rear_left, &Device::g_chassis_rear_right})
{
}

void Chariot::Init()
{
    // 各机构电机的硬件初始化已在 Device::g_* 的构造函数里完成
    // 中间件的发送通道由 tsk 绑定

    // 电机控制模式与 PID 参数在设备层统一写入
    Device::InitMotorInstances();

    // id=3 的腿帧在 leg_.Init() 里注册
    Module::Leg::Parameters leg_param;
    leg_.Init(leg_param);

    // id=1 的底盘帧在 chassis_.Init() 里注册
    Module::Chassis::Parameters chassis_param;
    chassis_.Init(chassis_param);

    // id=5 的遥控帧在 telecontrol_.Init() 里注册
    Module::Telecontrol::Parameters telecontrol_param;
    telecontrol_.Init(telecontrol_param);
}

void Chariot::McuRxCallback(uint8_t *data, uint16_t length)
{
    // 滑动窗口，判断上位机是否在线
    alive_flag_ += 1;

    Middleware::g_usb_communication_interface.RxRptlCallback(data, length);
}

void Chariot::CrsfRxCallback(uint8_t *data, uint16_t length)
{
    telecontrol_.UartRxCallback(data, length);
}

void Chariot::TimCalculatePeriodElapsedCallback()
{
    leg_.TimCalculatePeriodElapsedCallback();

    calibration_finished_ = leg_.IsCalibrated();

    // 校准期间整机不动：底盘不跟上位机下发的轮速，只有腿自己去找零点
    if (calibration_finished_)
    {
        chassis_.TimCalculatePeriodElapsedCallback();
    }

    // 实际发帧由 Driver::Tim1msCanPeriodElapsedCallback() 统一做
    Driver::Tim1msCanPeriodElapsedCallback();

    // 校准没完成前不回传：腿反馈是没意义的，连带遥控帧也先不发，
    // 免得上位机拿到的是一堆假的关节位置
    if (!calibration_finished_)
    {
        return;
    }

    telecontrol_.TimCalculatePeriodElapsedCallback();

    Middleware::g_usb_communication_interface.Send();
}

void Chariot::Tim100msAlivePeriodElapsedCallback()
{
    telecontrol_.Tim100msAlivePeriodElapsedCallback();

    Device::g_leg_front_left.Tim100msAlivePeriodElapsedCallback();
    Device::g_leg_front_right.Tim100msAlivePeriodElapsedCallback();
    Device::g_leg_rear_left.Tim100msAlivePeriodElapsedCallback();
    Device::g_leg_rear_right.Tim100msAlivePeriodElapsedCallback();

    Device::g_chassis_front_left.Tim100msAlivePeriodElapsedCallback();
    Device::g_chassis_front_right.Tim100msAlivePeriodElapsedCallback();
    Device::g_chassis_rear_left.Tim100msAlivePeriodElapsedCallback();
    Device::g_chassis_rear_right.Tim100msAlivePeriodElapsedCallback();
}

void Chariot::Tim1000msAlivePeriodElapsedCallback()
{
    pc_is_alive_ = (alive_flag_ != pre_alive_flag_);
    pre_alive_flag_ = alive_flag_;
}
