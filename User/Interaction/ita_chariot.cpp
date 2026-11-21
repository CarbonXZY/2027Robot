/**
 * @file ita_chariot.cpp
 * @brief 整车组合：把各机构绑到一起，对上层只暴露几个回调
 */

#include "ita_chariot.h"

#include "Communication_Interface.hpp"
#include "drv_can.h"
#include "dvc_motor_instances.h"
#include "main.h"

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

// Earth Block 气动件引脚：沿用板载两路 DC24 输出（PC14/PC13）
// 右吸盘暂不接线；气泵为高电平动作
const Module::EarthBlock::HardwareConfig kEarthBlockHardware = {
    .suction_0_port = Power_OUT1_EN_GPIO_Port,
    .suction_0_pin = Power_OUT1_EN_Pin,
    .suction_1_port = nullptr,
    .suction_1_pin = 0,
    .air_rail_port = Power_OUT2_EN_GPIO_Port,
    .air_rail_pin = Power_OUT2_EN_Pin,
};

const Module::SkyBlock::HardwareConfig kSkyBlockHardware = {
    .Wrist_Timer = &htim1,
    .Wrist_Channel = TIM_CHANNEL_3,
    .Wrist_Min_Pulse_Us = 500,
    .Wrist_Max_Pulse_Us = 2500,
    .Wrist_Safe_Position = 0.5f,
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
      chassis_({&Device::g_motor.chassis_front_left, &Device::g_motor.chassis_front_right, &Device::g_motor.chassis_rear_left, &Device::g_motor.chassis_rear_right}),
      earth_block_(Device::g_motor.earth_block),
      sky_block_(
          Device::g_motor.sky_block_big_arm,
          Device::g_motor.sky_block_small_arm,
          Device::g_motor.sky_block_gripper)
{
}

void Chariot::Init()
{
    // 各机构电机的硬件初始化已在 Device::g_* 的构造函数里完成
    // 中间件的发送通道由 tsk 绑定

    // 电机控制模式与 PID 参数在设备层统一写入
    Device::g_motor.Init();

    // 各模块按通信帧 id 顺序注册: 1 底盘 → 2 Earth Block → 3 腿 → 4 Sky Block → 5 遥控。
    // 中间件按注册顺序打包，id 3/4 两帧合计 64B，正好落在同一个 USB 包里。

    // id=1 的底盘帧在 chassis_.Init() 里注册
    Module::Chassis::Parameters chassis_param = {
        .direction_sign = {1, -1, 1, -1},
    };
    chassis_.Init(chassis_param);

    // Earth Block：绑定气动件引脚，并注册 id=2 的通信帧
    if (earth_block_.Init(kEarthBlockHardware))
    {
        // 整车没有单独的地块调试命令，上电后启动一次电机导轨校准。
        earth_block_.CalibrateMotorRail();
    }

    // id=3 的腿帧在 leg_.Init() 里注册
    leg_.Init(0.1f, kLegHoming);

    // Sky Block：绑定舵机 PWM，注册 id=4 的通信帧；电机对象已由 Device::g_motor 初始化
    sky_block_.Init(kSkyBlockHardware);

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

    // 全车校准完成标志：腿进入 kRunning，且 Earth Block 导轨、Sky Block 三轴都完成校准
    calibration_finished_ =
        (leg_.GetStatus() == Module::Leg::Status::kRunning) &&
        earth_block_.IsMotorRailCalibrated() &&
        sky_block_.Is_Calibrated();

    chassis_.TimCalculatePeriodElapsedCallback();

    telecontrol_.TimCalculatePeriodElapsedCallback();

    // Earth Block：通信目标 → 状态机 → 上行反馈
    earth_block_.TimCalculate1msCallback();

    sky_block_.TimCalculate1msCallback();
}

void Chariot::Tim100msAlivePeriodElapsedCallback()
{
    telecontrol_.Tim100msAlivePeriodElapsedCallback();

    leg_.Tim100msAlivePeriodElapsedCallback();

    earth_block_.TimAlive100msCallback();

    sky_block_.TimAlive100msCallback();

    Device::g_motor.chassis_front_left.Tim100msAlivePeriodElapsedCallback();
    Device::g_motor.chassis_front_right.Tim100msAlivePeriodElapsedCallback();
    Device::g_motor.chassis_rear_left.Tim100msAlivePeriodElapsedCallback();
    Device::g_motor.chassis_rear_right.Tim100msAlivePeriodElapsedCallback();
}

void Chariot::Tim100msPcAlivePeriodElapsedCallback()
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

    // Earth Block 不做掉线保护：上位机掉线后 tx_buffer_ 保持最后一帧目标，
    // 电机继续跟随最后位置，吸盘/气动导轨保持最后状态，不主动停止。
}
