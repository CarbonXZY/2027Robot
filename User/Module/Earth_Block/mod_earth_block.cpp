/**
 * @file mod_earth_block.cpp
 * @brief Earth Block 机构实现
 *
 * 由 KFC 机构移植，气动件改用 dvc_airtool 的 Class_AIRPUMP。
 * 法一（大误差定速接近、小误差位置闭环）在本模块的
 * CalculateMotorRail() 内实现，不改动 Device 层。
 */

#include "mod_earth_block.h"

#include <cstddef>
#include <cstdint>

#include "Communication_Interface.hpp"
#include "mid_config.h"

namespace Module
{

/**
 * @brief 构造 Earth Block，绑定 Device::g_motor 中的 C610
 * @param[in] motor 已由 MotorInstances::Init() 初始化的导轨电机
 */
EarthBlock::EarthBlock(Device::MotorDjiC610 &motor)
    : motor_(&motor)
{
}

/**
 * @brief 初始化吸盘、气动导轨，并将 C610 机械参数绑定到模块
 *
 * 成功后绑定通信层并注册 id=2 的收发帧。
 *
 * @param[in] hardware 引脚配置
 * @return true 初始化成功
 * @return false 引脚或导轨参数非法、C610 未初始化
 */
bool EarthBlock::Init(const HardwareConfig &hardware)
{
    initialized_ = false;
    status_ = Status::kDisable;
    current_command_ = {};
    ClearCommandQueue();

    const bool suction_1_port_valid = hardware.suction_1_port != nullptr;
    const bool suction_1_pin_valid = hardware.suction_1_pin != 0;

    if (hardware.suction_0_port == nullptr ||
        hardware.suction_0_pin == 0 ||
        hardware.air_rail_port == nullptr ||
        hardware.air_rail_pin == 0 ||
        suction_1_port_valid != suction_1_pin_valid)
    {
        return false;
    }

    const MotorRailConfig &motor_rail = motion_.motor_rail;
    if (!std::isfinite(motor_rail.mm_per_motor_rad) ||
        !std::isfinite(motor_rail.direction_sign) ||
        !std::isfinite(motor_rail.min_mm) ||
        !std::isfinite(motor_rail.max_mm) ||
        !std::isfinite(motor_rail.arrive_position_mm) ||
        !std::isfinite(motor_rail.arrive_speed_mmps) ||
        !std::isfinite(motor_rail.approach_speed_radps) ||
        !std::isfinite(motor_rail.switch_error_rad) ||
        motor_rail.mm_per_motor_rad <= 0.0f ||
        motor_rail.direction_sign == 0.0f ||
        motor_rail.max_mm < motor_rail.min_mm ||
        motor_rail.arrive_position_mm <= 0.0f ||
        motor_rail.arrive_speed_mmps <= 0.0f ||
        motor_rail.approach_speed_radps <= 0.0f ||
        motor_rail.switch_error_rad <= 0.0f)
    {
        return false;
    }

    if (motor_ == nullptr || !motor_->IsInitialized())
    {
        return false;
    }

    hardware_ = hardware;
    motor_rail_param_ = motor_rail;
    motor_rail_param_.direction_sign =
        motor_rail.direction_sign > 0.0f ? 1.0f : -1.0f;
    motor_rail_offset_rad_ = motor_->GetPosition();
    motor_rail_current_mm_ = 0.0f;
    motor_rail_current_speed_mmps_ = 0.0f;
    motor_rail_target_mm_ = 0.0f;
    motor_rail_calibrated_ = false;
    motor_rail_calibration_active_ = false;
    motor_rail_control_enabled_ = false;
    motor_rail_initialized_ = true;

    suction_enabled_[0] = true;
    suction_cup_[0].Init(
        hardware.suction_0_port,
        hardware.suction_0_pin);

    suction_enabled_[1] = suction_1_port_valid && suction_1_pin_valid;
    if (suction_enabled_[1])
    {
        suction_cup_[1].Init(
            hardware.suction_1_port,
            hardware.suction_1_pin);
    }

    air_rail_.Init(
        hardware.air_rail_port,
        hardware.air_rail_pin);

    motor_->SetControlMethod(Device::MotorControlMethod::kCurrent);
    motor_->SetTargetCurrent(0.0f);
    motor_->Calculate();

    initialized_ = true;
    status_ = Status::kIdle;

    // 硬件就绪后绑定通信层，并注册 id=2 的收发帧
    interact_.Init(this);

    return true;
}

/**
 * @brief 获取队列中尚未执行的命令数量
 * @return 待执行命令数量
 */
uint8_t EarthBlock::GetCommandCount() const
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const uint8_t count =
        (command_queue_head_ + kCommandQueueSize - command_queue_tail_) %
        kCommandQueueSize;

    if (primask == 0U)
    {
        __enable_irq();
    }
    return count;
}

/**
 * @brief 判断命令队列是否已满
 * @return true 队列已满
 * @return false 队列未满
 */
bool EarthBlock::IsCommandQueueFull() const
{
    return GetCommandCount() >= kCommandQueueSize - 1U;
}

/**
 * @brief 清空所有尚未执行的命令
 */
void EarthBlock::ClearCommandQueue()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    command_queue_head_ = 0;
    command_queue_tail_ = 0;

    if (primask == 0U)
    {
        __enable_irq();
    }
}

/**
 * @brief 将命令写入环形队列
 * @param[in] command 待写入命令
 * @return true 入队成功
 * @return false 未初始化、处于 Fault 或队列已满
 */
bool EarthBlock::PushCommand(const Command &command)
{
    if (!initialized_)
    {
        return false;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (status_ == Status::kFault)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return false;
    }

    const uint8_t next_head =
        (command_queue_head_ + 1U) % kCommandQueueSize;
    if (next_head == command_queue_tail_)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return false;
    }

    command_queue_[command_queue_head_] = command;
    command_queue_head_ = next_head;

    if (primask == 0U)
    {
        __enable_irq();
    }
    return true;
}

/**
 * @brief 从环形队列取出最早的一条命令
 * @param[out] command 取出的命令
 * @return true 队列非空，取出成功
 * @return false 队列为空
 */
bool EarthBlock::PopCommand(Command &command)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (command_queue_tail_ == command_queue_head_)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return false;
    }

    command = command_queue_[command_queue_tail_];
    command_queue_tail_ =
        (command_queue_tail_ + 1U) % kCommandQueueSize;

    if (primask == 0U)
    {
        __enable_irq();
    }
    return true;
}

/**
 * @brief 将队列命令装载为当前动作
 * @param[in] command 要装载的命令
 * @param[in] now_us 当前时刻，单位 us
 */
void EarthBlock::LoadCommand(
    const Command &command,
    uint32_t now_us)
{
    current_command_ = command;
    action_start_time_us_ = now_us;
    status_ = Status::kQueued;
}

/**
 * @brief 吸取指定吸盘
 * @param[in] cup_index 吸盘编号：0=左，1=右
 * @return true 命令已入队
 * @return false 未初始化、编号非法或队列已满
 */
bool EarthBlock::SuckCup(uint8_t cup_index)
{
    if (!initialized_ || cup_index >= 2 || !suction_enabled_[cup_index])
    {
        return false;
    }

    const Command command
    {
        .action = Action::kSuctionOn,
        .suction_index = cup_index,
    };
    return PushCommand(command);
}

/**
 * @brief 松开指定吸盘
 * @param[in] cup_index 吸盘编号：0=左，1=右
 * @return true 命令已入队
 * @return false 未初始化、编号非法或队列已满
 */
bool EarthBlock::ReleaseCup(uint8_t cup_index)
{
    if (!initialized_ || cup_index >= 2 || !suction_enabled_[cup_index])
    {
        return false;
    }

    const Command command
    {
        .action = Action::kSuctionOff,
        .suction_index = cup_index,
    };
    return PushCommand(command);
}

/**
 * @brief 气动导轨切换到顶部
 * @return true 命令已入队
 * @return false 未初始化或队列已满
 */
bool EarthBlock::AirRailUp()
{
    const Command command
    {
        .action = Action::kAirRailUp,
    };
    return PushCommand(command);
}

/**
 * @brief 气动导轨切换到底部
 * @return true 命令已入队
 * @return false 未初始化或队列已满
 */
bool EarthBlock::AirRailDown()
{
    const Command command
    {
        .action = Action::kAirRailDown,
    };
    return PushCommand(command);
}

/**
 * @brief 把毫米目标交给内部电机并排队执行
 * @param[in] position_mm 目标位置，单位 mm，自动限幅到 [min_mm, max_mm]
 * @return true 命令已入队
 * @return false 未初始化、参数非法或队列已满
 */
bool EarthBlock::MoveMotorRailTo(float position_mm)
{
    if (!initialized_ ||
        !motor_rail_initialized_ ||
        !std::isfinite(position_mm))
    {
        return false;
    }

    if (position_mm < motor_rail_param_.min_mm)
    {
        position_mm = motor_rail_param_.min_mm;
    }
    else if (position_mm > motor_rail_param_.max_mm)
    {
        position_mm = motor_rail_param_.max_mm;
    }

    const Command command
    {
        .action = Action::kMotorRailMove,
        .target_mm = position_mm,
    };
    return PushCommand(command);
}

/**
 * @brief 启动 C610 堵转校准
 * @return true 命令已入队
 * @return false 未初始化或队列已满
 */
bool EarthBlock::CalibrateMotorRail()
{
    if (!initialized_)
    {
        return false;
    }

    const Command command
    {
        .action = Action::kMotorRailCalibrate,
    };
    return PushCommand(command);
}

/**
 * @brief 将当前 C610 位置设为零点
 * @return true 命令已入队
 * @return false 未初始化或队列已满
 */
bool EarthBlock::SetCurrentMotorRailPositionAsZero()
{
    if (!initialized_)
    {
        return false;
    }

    const Command command
    {
        .action = Action::kMotorRailSetZero,
    };
    return PushCommand(command);
}

/**
 * @brief 立即停止 C610 导轨并清空命令队列
 */
void EarthBlock::StopMotorRail()
{
    if (!initialized_)
    {
        return;
    }

    StopMotorRailControl();
    ClearCommandQueue();
    current_command_ = {};
    status_ = Status::kIdle;
}

/**
 * @brief 1ms 周期入口：先通信，再推进状态机，最后回填反馈
 * @note 由 Task 每 1 ms 调用一次
 */
void EarthBlock::TimCalculate1msCallback()
{
    interact_.TimCalculate1msCallback();
}

/**
 * @brief 周期推进 Earth Block 状态机
 *
 * 吸盘和气动导轨使用稳压时间完成，C610 导轨通过
 * 到位反馈完成。
 *
 * @note 由 EarthBlockInteract 每 1 ms 调用一次
 */
void EarthBlock::Calculate()
{
    if (!initialized_)
    {
        return;
    }

    const uint32_t now_us = Device::DwtGetCurrentTimeUs();

    // 空闲或上一条动作完成后，自动取下一条排队命令。
    if (status_ == Status::kIdle ||
        status_ == Status::kFinished)
    {
        Command command;
        if (PopCommand(command))
        {
            LoadCommand(command, now_us);
        }
    }

    switch (status_)
    {
    case Status::kQueued:
        ExecuteQueuedAction();
        break;

    case Status::kSuctionWait:
    {
        const uint32_t settle_us =
            current_command_.action == Action::kSuctionOn
                ? motion_.suction_settle_us
                : motion_.release_settle_us;

        if (now_us - action_start_time_us_ >= settle_us)
        {
            FinishAction();
        }
        break;
    }

    case Status::kAirRailWait:
        if (now_us - action_start_time_us_ >= motion_.air_rail_settle_us)
        {
            FinishAction();
        }
        break;

    case Status::kMotorRailMoving:
        if (IsActionTimeout(now_us))
        {
            SetFault();
        }
        else if (IsMotorRailArrived())
        {
            FinishAction();
        }
        break;

    case Status::kMotorRailCalibrating:
        if (UpdateMotorRailCalibration())
        {
            FinishAction();
        }
        else if (IsActionTimeout(now_us))
        {
            SetFault();
        }
        break;

    case Status::kDisable:
    case Status::kIdle:
    case Status::kFinished:
    case Status::kFault:
        break;
    }

    CalculateMotorRail();
}

/**
 * @brief 把 C610 CAN 反馈转交给内部电机
 * @param[in] rx_data 反馈帧数据指针
 */
void EarthBlock::FdcanRxCpltCallback(uint8_t *rx_data)
{
    motor_->FdcanRxCpltCallback(rx_data);
}

/**
 * @brief 执行 C610 在线检测
 * @note 由 Task 每 100 ms 调用一次
 */
void EarthBlock::TimAlive100msCallback()
{
    motor_->Tim100msAlivePeriodElapsedCallback();
}

/**
 * @brief 启动导轨回零状态
 * @note 回零期间失能位置目标，直到 UpdateMotorRailCalibration() 成功
 */
void EarthBlock::StartMotorRailCalibration()
{
    if (!motor_rail_initialized_)
    {
        return;
    }

    motor_rail_calibrated_ = false;
    motor_rail_calibration_active_ = true;
    motor_rail_control_enabled_ = false;
}

/**
 * @brief 推进回零，成功后建立毫米坐标
 * @return true 回零完成
 * @return false 仍在回零
 */
bool EarthBlock::UpdateMotorRailCalibration()
{
    if (!motor_rail_initialized_ ||
        !motor_rail_calibration_active_)
    {
        return motor_rail_calibrated_;
    }

    float offset = 0.0f;
    if (!motor_->Calibrate(motor_rail_param_.calibration, offset))
    {
        return false;
    }

    motor_rail_offset_rad_ = offset;
    motor_rail_target_mm_ = 0.0f;
    motor_rail_calibrated_ = true;
    motor_rail_calibration_active_ = false;
    motor_rail_control_enabled_ = true;
    UpdateMotorRailFeedback();
    return true;
}

/**
 * @brief 将当前电机角度直接设为机械零点
 * @return true 设置成功
 */
bool EarthBlock::SetMotorRailCurrentPositionAsZero()
{
    if (!motor_rail_initialized_)
    {
        return false;
    }

    motor_rail_offset_rad_ = motor_->GetPosition();
    motor_rail_current_mm_ = 0.0f;
    motor_rail_current_speed_mmps_ = 0.0f;
    motor_rail_target_mm_ = 0.0f;
    motor_rail_calibrated_ = true;
    motor_rail_calibration_active_ = false;
    motor_rail_control_enabled_ = true;
    return true;
}

/**
 * @brief 将电机反馈换算成导轨毫米位置和速度
 */
void EarthBlock::UpdateMotorRailFeedback()
{
    if (!motor_rail_initialized_)
    {
        return;
    }

    motor_->UpdateFeedback();

    motor_rail_current_mm_ =
        (motor_->GetPosition() - motor_rail_offset_rad_) *
        motor_rail_param_.mm_per_motor_rad *
        motor_rail_param_.direction_sign;

    motor_rail_current_speed_mmps_ =
        motor_->GetSpeed() *
        motor_rail_param_.mm_per_motor_rad *
        motor_rail_param_.direction_sign;
}

/**
 * @brief 执行导轨控制周期（含法一：大误差定速、小误差位置闭环）
 */
void EarthBlock::CalculateMotorRail()
{
    if (!motor_rail_initialized_)
    {
        return;
    }

    UpdateMotorRailFeedback();

    if (motor_rail_calibration_active_)
    {
        motor_->Calculate();
        return;
    }

    if (motor_rail_control_enabled_ && motor_rail_calibrated_)
    {
        const float target_rad =
            motor_rail_offset_rad_ +
            motor_rail_param_.direction_sign *
                motor_rail_target_mm_ /
                motor_rail_param_.mm_per_motor_rad;

        const float position_error_rad = target_rad - motor_->GetPosition();

        if (std::fabs(position_error_rad) >
            motor_rail_param_.switch_error_rad)
        {
            // 大误差：位置环不参与，固定速度朝目标接近。
            motor_->pid_position.SetIntegralError(0.0f);

            motor_->SetControlMethod(Device::MotorControlMethod::kSpeed);
            motor_->SetTargetSpeed(
                position_error_rad > 0.0f
                    ? motor_rail_param_.approach_speed_radps
                    : -motor_rail_param_.approach_speed_radps);
        }
        else
        {
            // 小误差：位置外环 + 速度内环串级。
            motor_->SetControlMethod(Device::MotorControlMethod::kPosition);
            motor_->SetTargetPosition(target_rad);
        }
    }
    else
    {
        motor_->SetControlMethod(Device::MotorControlMethod::kCurrent);
        motor_->SetTargetCurrent(0.0f);
    }

    motor_->Calculate();
}

/**
 * @brief 停止导轨控制并输出零电流
 */
void EarthBlock::StopMotorRailControl()
{
    if (!motor_rail_initialized_)
    {
        return;
    }

    motor_rail_calibration_active_ = false;
    motor_rail_control_enabled_ = false;
    motor_->SetControlMethod(Device::MotorControlMethod::kCurrent);
    motor_->SetTargetCurrent(0.0f);
    motor_->Calculate();
}

/**
 * @brief 判断毫米位置和速度是否同时进入到位阈值
 * @return true 已到位
 * @return false 未到位
 */
bool EarthBlock::IsMotorRailArrived() const
{
    return std::fabs(motor_rail_target_mm_ - motor_rail_current_mm_) <=
               motor_rail_param_.arrive_position_mm &&
           std::fabs(motor_rail_current_speed_mmps_) <=
               motor_rail_param_.arrive_speed_mmps;
}

/**
 * @brief 执行刚排队的机构动作
 */
void EarthBlock::ExecuteQueuedAction()
{
    switch (current_command_.action)
    {
    case Action::kSuctionOn:
        ApplySuction(current_command_.suction_index, true);
        status_ = Status::kSuctionWait;
        break;

    case Action::kSuctionOff:
        ApplySuction(current_command_.suction_index, false);
        status_ = Status::kSuctionWait;
        break;

    case Action::kAirRailUp:
        air_rail_.AIRPUMP_Open();
        status_ = Status::kAirRailWait;
        break;

    case Action::kAirRailDown:
        air_rail_.AIRPUMP_Close();
        status_ = Status::kAirRailWait;
        break;

    case Action::kMotorRailMove:
        if (!motor_rail_calibrated_)
        {
            SetFault();
            break;
        }
        motor_rail_target_mm_ = current_command_.target_mm;
        motor_rail_control_enabled_ = true;
        status_ = Status::kMotorRailMoving;
        break;

    case Action::kMotorRailCalibrate:
        StartMotorRailCalibration();
        status_ = Status::kMotorRailCalibrating;
        break;

    case Action::kMotorRailSetZero:
        if (!SetMotorRailCurrentPositionAsZero())
        {
            SetFault();
            break;
        }
        FinishAction();
        break;

    case Action::kNone:
    default:
        SetFault();
        break;
    }
}

/**
 * @brief 操作指定索引的吸盘
 * @param[in] cup_index 吸盘编号：0=左，1=右
 * @param[in] enable true=吸附，false=释放
 */
void EarthBlock::ApplySuction(uint8_t cup_index, bool enable)
{
    if (cup_index >= 2 || !suction_enabled_[cup_index])
    {
        return;
    }

    enable
        ? suction_cup_[cup_index].AIRPUMP_Open()
        : suction_cup_[cup_index].AIRPUMP_Close();
}

/**
 * @brief 判断电机导轨动作是否超时
 * @param[in] now_us 当前时刻，单位 us
 * @return true 已超时
 * @return false 未超时
 */
bool EarthBlock::IsActionTimeout(uint32_t now_us) const
{
    return motion_.motor_rail_move_timeout_us != 0 &&
           now_us - action_start_time_us_ >= motion_.motor_rail_move_timeout_us;
}

/**
 * @brief 将机构状态切换为 Finished
 */
void EarthBlock::FinishAction()
{
    status_ = Status::kFinished;
}

/**
 * @brief 进入 Fault 并停止电机导轨
 */
void EarthBlock::SetFault()
{
    status_ = Status::kFault;
    ClearCommandQueue();
    StopMotorRailControl();
}

/**
 * @brief 显式清除故障并回到空闲状态
 */
void EarthBlock::ClearFault()
{
    if (!initialized_ || status_ != Status::kFault)
    {
        return;
    }

    StopMotorRailControl();
    ClearCommandQueue();
    current_command_ = {};
    status_ = Status::kIdle;
}

/* ====================================================================== */
/*  EarthBlockInteract 实现                                                  */
/* ====================================================================== */

/**
 * @brief 绑定 EarthBlock 并注册 id=2 的收发帧
 * @param[in] earth_block 被控机构，由 EarthBlock::Init() 传入 this
 */
void EarthBlockInteract::Init(EarthBlock *earth_block)
{
    earth_block_ = earth_block;
    if (earth_block_ == nullptr)
    {
        return;
    }

    UsbRegister();

    // 以当前下行缓冲为基准，避免上电后重复下发
    last_pump_target_[0] = tx_buffer_.pump_target[0];
    last_pump_target_[1] = tx_buffer_.pump_target[1];
    last_motor_target_mm_ = tx_buffer_.motor_position_mm;
}

/**
 * @brief 1ms 通信周期：下发目标 → 推进 EarthBlock 状态机 → 回填反馈
 */
void EarthBlockInteract::TimCalculate1msCallback()
{
    if (earth_block_ == nullptr)
    {
        return;
    }

    UsbSetTarget();
    earth_block_->Calculate();
    UsbFeedback();
}

/**
 * @brief 把 id=2 的收发帧注册进通信中间件
 * @note Register() 的 tx/rx 是站在本板视角：本板发反馈，收指令
 */
void EarthBlockInteract::UsbRegister()
{
    // 注意 Register() 的 tx/rx 是站在本板视角：本板发反馈，收指令
    Middleware::g_usb_communication_interface.Register(
        static_cast<uint8_t>(Middleware::CommFrameId::kEarthBlock),
        &rx_buffer_, &tx_buffer_,
        static_cast<uint8_t>(sizeof(rx_buffer_)), static_cast<uint8_t>(sizeof(tx_buffer_)));
}

/**
 * @brief 把下行目标的变化翻译成 EarthBlock 动作
 *
 * 边沿触发：只有上位机改变目标才入队，避免每帧重复塞命令。
 * - pump_target[0]：0/1 → ReleaseCup(0) / SuckCup(0)
 * - pump_target[1]：0/1 → AirRailDown() / AirRailUp()
 * - motor_position_mm：变化超过 0.1 mm → MoveMotorRailTo()
 *
 * @note 上位机掉线时 tx_buffer_ 不再更新，目标保持最后一帧不变
 */
void EarthBlockInteract::UsbSetTarget()
{
    // 气泵目标是边沿触发：只有上位机改变目标才入队，避免每帧重复塞命令
    if (tx_buffer_.pump_target[0] != last_pump_target_[0])
    {
        const bool accepted = (tx_buffer_.pump_target[0] != 0)
            ? earth_block_->SuckCup(0)
            : earth_block_->ReleaseCup(0);
        if (accepted)
        {
            last_pump_target_[0] = tx_buffer_.pump_target[0];
        }
    }

    if (tx_buffer_.pump_target[1] != last_pump_target_[1])
    {
        const bool accepted = (tx_buffer_.pump_target[1] != 0)
            ? earth_block_->AirRailUp()
            : earth_block_->AirRailDown();
        if (accepted)
        {
            last_pump_target_[1] = tx_buffer_.pump_target[1];
        }
    }

    // 电机目标是边沿触发：变化超过 0.1 mm 才下发一次移动
    if (std::isfinite(tx_buffer_.motor_position_mm) &&
        std::fabs(tx_buffer_.motor_position_mm - last_motor_target_mm_) > 0.1f)
    {
        if (earth_block_->MoveMotorRailTo(tx_buffer_.motor_position_mm))
        {
            last_motor_target_mm_ = tx_buffer_.motor_position_mm;
        }
    }
}

/**
 * @brief 把 EarthBlock 的位置、速度、气泵状态填进上行帧
 */
void EarthBlockInteract::UsbFeedback()
{
    rx_buffer_.motor.position = earth_block_->GetMotorRailPositionMm();
    rx_buffer_.motor.velocity = earth_block_->GetMotorRailSpeedMmps();
    rx_buffer_.pump_status[0] = static_cast<uint8_t>(earth_block_->GetSuctionStatus(0));
    rx_buffer_.pump_status[1] = static_cast<uint8_t>(earth_block_->GetAirRailStatus());
}

} // namespace Module
