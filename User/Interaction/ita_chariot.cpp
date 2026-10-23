/**
 * @file ita_chariot.cpp
 * @brief 整车组合：把各机构绑到一起，对上层只暴露几个回调
 */

#include "ita_chariot.h"

#include "Communication_Interface.hpp"
#include "drv_can.h"
#include "dvc_motor_instances.h"

namespace
{
// 齿轮齿条机械参数, 下标顺序 lf, rf, lr, rr
constexpr Module::Leg::Mechanical kLegMechanical = {
    .equivalent_radius = {0.025f, 0.025f, 0.025f, 0.025f},
    .direction_sign = {1, -1, -1, 1},
    .max_height = {0.3f, 0.3f},
};

// 主动找限位参数
constexpr Module::Leg::Homing kLegHoming = {
    .speed = 0.05f,
    .speed_threshold = 0.001f,
    .debounce_ms = 20,
};
} // namespace

Chariot::Chariot()
    : leg_(Module::Leg::MotorGroup{
          .lf = &Device::g_motor.leg_front_left,
          .rf = &Device::g_motor.leg_front_right,
          .lr = &Device::g_motor.leg_rear_left,
          .rr = &Device::g_motor.leg_rear_right,
          }, kLegMechanical),
    // : leg_(&Module::g_legs),
      chassis_({&Device::g_motor.chassis_front_left, &Device::g_motor.chassis_front_right, &Device::g_motor.chassis_rear_left, &Device::g_motor.chassis_rear_right})
{
}

void Chariot::Init()
{
    // 各机构电机的硬件初始化已在 Device::g_* 的构造函数里完成
    // 中间件的发送通道由 tsk 绑定

    // 电机控制模式与 PID 参数在设备层统一写入
    Device::g_motor.Init();

    // id=3 的腿帧在 leg_.Init() 里注册
    leg_.Init(0.1f, kLegHoming);

    // id=1 的底盘帧在 chassis_.Init() 里注册
    Module::Chassis::Parameters chassis_param = {
        .direction_sign = {1, -1, 1, -1},
    };
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
    // leg_.TimCalculatePeriodElapsedCallback();

    // 校准完成标志：腿状态机进入 kRunning 即认为全车校准完成
    calibration_finished_ = (leg_.GetStatus() == Module::Leg::Status::kRunning);

    chassis_.TimCalculatePeriodElapsedCallback();

    telecontrol_.TimCalculatePeriodElapsedCallback();
}

void Chariot::Tim100msAlivePeriodElapsedCallback()
{
    telecontrol_.Tim100msAlivePeriodElapsedCallback();

    leg_.Tim100msAlivePeriodElapsedCallback();

    Device::g_motor.chassis_front_left.Tim100msAlivePeriodElapsedCallback();
    Device::g_motor.chassis_front_right.Tim100msAlivePeriodElapsedCallback();
    Device::g_motor.chassis_rear_left.Tim100msAlivePeriodElapsedCallback();
    Device::g_motor.chassis_rear_right.Tim100msAlivePeriodElapsedCallback();
}

void Chariot::Tim1000msAlivePeriodElapsedCallback()
{
    pc_is_alive_ = (alive_flag_ != pre_alive_flag_);
    pre_alive_flag_ = alive_flag_;

    // PC 掉线保护：腿稳定在最后一帧目标位置，底盘轮速清零
    if (pc_is_alive_)
    {
        leg_.SetState(Module::LegStatus::kEnable);
        chassis_.SetState(Module::ChassisStatus::kEnable);
    }
    else
    {
        leg_.SetState(Module::LegStatus::kDisable);
        chassis_.SetState(Module::ChassisStatus::kDisable);
    }
}
