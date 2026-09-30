/**
 * @file ita_chariot.cpp
 * @brief 整车组合：把各机构绑到一起，对上层只暴露几个回调
 */

#include "ita_chariot.h"

#include <cstddef>

#include "Communication_Interface.hpp"
#include "drv_can.h"
#include "drv_usb.h"
#include "dvc_motor_instances.h"

namespace
{

// 中间件的发送回调是裸函数指针，只能绑无捕获函数
int64_t SendCallback(uint8_t *data, size_t length)
{
    return Driver::UsbTransmitAsync(data, static_cast<uint16_t>(length)) == Driver::USB_Status::kOk
               ? static_cast<int64_t>(length)
               : -1;
}

} // namespace

Chariot::Chariot()
    : leg_({&Device::g_leg_front_left, &Device::g_leg_front_right, &Device::g_leg_rear_left, &Device::g_leg_rear_right})
{
}

void Chariot::Init()
{
    // 腿关节的硬件初始化已在 Device::g_leg_* 的构造函数里完成
    Middleware::g_usb_communication_interface.Init(SendCallback);

    // id=3 的两帧在 leg_.Init() 里注册
    Module::Leg<4>::Parameters leg_param;
    leg_.Init(leg_param);
}

void Chariot::McuRxCallback(uint8_t *data, uint16_t length)
{
    // 滑动窗口，判断上位机是否在线
    alive_flag_ += 1;

    Middleware::g_usb_communication_interface.RxRptlCallback(data, length);
}

void Chariot::TimCalculatePeriodElapsedCallback()
{
    leg_.TimCalculatePeriodElapsedCallback();

    // 实际发帧由 Driver::Tim1msCanPeriodElapsedCallback() 统一做
    Driver::Tim1msCanPeriodElapsedCallback();

    calibration_finished_ = leg_.IsCalibrated();

    // 校准没完成前不回传，免得上位机拿到的是没意义的反馈
    if (!calibration_finished_)
    {
        return;
    }

    Middleware::g_usb_communication_interface.Send();
}

void Chariot::Tim100msAlivePeriodElapsedCallback()
{
    Device::g_leg_front_left.Tim100msAlivePeriodElapsedCallback();
    Device::g_leg_front_right.Tim100msAlivePeriodElapsedCallback();
    Device::g_leg_rear_left.Tim100msAlivePeriodElapsedCallback();
    Device::g_leg_rear_right.Tim100msAlivePeriodElapsedCallback();
}

void Chariot::Tim1000msAlivePeriodElapsedCallback()
{
    pc_is_alive_ = (alive_flag_ != pre_alive_flag_);
    pre_alive_flag_ = alive_flag_;
}
