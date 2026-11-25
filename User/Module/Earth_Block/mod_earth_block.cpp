/**
 * @file mod_earth_block.cpp
 * @brief Earth Block 直接调用实现
 */

#include "mod_earth_block.h"

#include <cstddef>
#include <cstdint>

#include "Communication_Interface.hpp"
#include "mid_config.h"

namespace Module
{

EarthBlock::EarthBlock(Device::MotorDjiC610 &motor)
    : motor_(&motor)
{
}

bool EarthBlock::Init(const HardwareConfig &hardware)
{
    initialized_ = false;

    const bool suction_1_port_valid = hardware.suction_1_port != nullptr;
    const bool suction_1_pin_valid = hardware.suction_1_pin != 0;

    if (hardware.suction_0_port == nullptr ||
        hardware.suction_0_pin == 0 ||
        hardware.air_rail_port == nullptr ||
        hardware.air_rail_pin == 0 ||
        suction_1_port_valid != suction_1_pin_valid ||
        motor_ == nullptr ||
        !motor_->IsInitialized())
    {
        return false;
    }

    hardware_ = hardware;
    motor_rail_param_.direction_sign =
        motor_rail_param_.direction_sign > 0.0f ? 1.0f : -1.0f;
    motor_rail_offset_rad_ = motor_->GetPosition();
    motor_rail_current_mm_ = 0.0f;
    motor_rail_current_speed_mmps_ = 0.0f;
    motor_rail_target_mm_ = 0.0f;
    motor_rail_calibrated_ = false;
    motor_rail_calibration_active_ = false;
    motor_rail_control_enabled_ = false;

    suction_enabled_[0] = true;
    suction_cup_[0].Init(hardware.suction_0_port, hardware.suction_0_pin);

    suction_enabled_[1] = suction_1_port_valid && suction_1_pin_valid;
    if (suction_enabled_[1])
    {
        suction_cup_[1].Init(hardware.suction_1_port, hardware.suction_1_pin);
    }

    air_rail_.Init(hardware.air_rail_port, hardware.air_rail_pin);

    motor_->SetControlMethod(Device::MotorControlMethod::kCurrent);
    motor_->SetTargetCurrent(0.0f);
    motor_->Calculate();

    initialized_ = true;
    interact_.Init(this);
    return true;
}

bool EarthBlock::SuckAllCups()
{
    if (!initialized_)
    {
        return false;
    }

    for (uint8_t i = 0; i < 2; ++i)
    {
        if (suction_enabled_[i])
        {
            suction_cup_[i].AIRPUMP_Open();
        }
    }
    return true;
}

bool EarthBlock::ReleaseAllCups()
{
    if (!initialized_)
    {
        return false;
    }

    for (uint8_t i = 0; i < 2; ++i)
    {
        if (suction_enabled_[i])
        {
            suction_cup_[i].AIRPUMP_Close();
        }
    }
    return true;
}

bool EarthBlock::AirRailUp()
{
    if (!initialized_)
    {
        return false;
    }

    air_rail_.AIRPUMP_Open();
    return true;
}

bool EarthBlock::AirRailDown()
{
    if (!initialized_)
    {
        return false;
    }

    air_rail_.AIRPUMP_Close();
    return true;
}

bool EarthBlock::MoveMotorRailTo(float position_mm)
{
    if (!initialized_ || !std::isfinite(position_mm))
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

    motor_rail_target_mm_ = position_mm;
    motor_rail_control_enabled_ = true;
    return true;
}

bool EarthBlock::CalibrateMotorRail()
{
    if (!initialized_)
    {
        return false;
    }

    motor_rail_calibrated_ = false;
    motor_rail_calibration_active_ = true;
    motor_rail_control_enabled_ = false;
    return true;
}

void EarthBlock::StopMotorRail()
{
    if (initialized_)
    {
        StopMotorRailControl();
    }
}

void EarthBlock::TimCalculate1msCallback()
{
    interact_.TimCalculate1msCallback();
}

void EarthBlock::TimAlive100msCallback()
{
    motor_->Tim100msAlivePeriodElapsedCallback();
}

void EarthBlock::Calculate()
{
    if (!initialized_)
    {
        return;
    }

    UpdateMotorRailFeedback();

    if (motor_rail_calibration_active_)
    {
        float offset = 0.0f;
        if (motor_->Calibrate(motor_rail_param_.calibration, offset))
        {
            motor_rail_offset_rad_ = offset;
            motor_rail_target_mm_ = 0.0f;
            motor_rail_calibrated_ = true;
            motor_rail_calibration_active_ = false;
            motor_rail_control_enabled_ = true;
            UpdateMotorRailFeedback();
        }
        motor_->Calculate();
        return;
    }

    CalculateMotorRail();
}

void EarthBlock::UpdateMotorRailFeedback()
{
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

void EarthBlock::CalculateMotorRail()
{
    if (motor_rail_control_enabled_ && motor_rail_calibrated_)
    {
        const float target_rad =
            motor_rail_offset_rad_ +
            motor_rail_param_.direction_sign *
                motor_rail_target_mm_ /
                motor_rail_param_.mm_per_motor_rad;

        const float position_error_rad = target_rad - motor_->GetPosition();
        if (std::fabs(position_error_rad) > motor_rail_param_.switch_error_rad)
        {
            // KFC 已验证的法一接近阶段不主动清位置积分。
            motor_->SetControlMethod(Device::MotorControlMethod::kSpeed);
            motor_->SetTargetSpeed(
                position_error_rad > 0.0f
                    ? motor_rail_param_.approach_speed_radps
                    : -motor_rail_param_.approach_speed_radps);
        }
        else
        {
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

void EarthBlock::StopMotorRailControl()
{
    motor_rail_calibration_active_ = false;
    motor_rail_control_enabled_ = false;
    motor_->SetControlMethod(Device::MotorControlMethod::kCurrent);
    motor_->SetTargetCurrent(0.0f);
    motor_->Calculate();
}

Enum_AIRPUMP_Status EarthBlock::GetAllSuctionStatus()
{
    bool has_sucking = false;
    bool has_released = false;

    for (uint8_t i = 0; i < 2; ++i)
    {
        if (!suction_enabled_[i])
        {
            continue;
        }

        const Enum_AIRPUMP_Status status = suction_cup_[i].Get_Status();
        has_sucking |= status == AIRPUMP_Status_Open;
        has_released |= status == AIRPUMP_Status_Close;
    }

    if (has_sucking && !has_released)
    {
        return AIRPUMP_Status_Open;
    }
    if (has_released && !has_sucking)
    {
        return AIRPUMP_Status_Close;
    }
    return AIRPUMP_Status_None;
}

Enum_AIRPUMP_Status EarthBlock::GetAirRailStatus()
{
    return air_rail_.Get_Status();
}

void EarthBlockInteract::Init(EarthBlock *earth_block)
{
    earth_block_ = earth_block;
    if (earth_block_ == nullptr)
    {
        return;
    }

    UsbRegister();
    last_pump_target_[0] = tx_buffer_.pump_target[0];
    last_pump_target_[1] = tx_buffer_.pump_target[1];
}

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

void EarthBlockInteract::UsbRegister()
{
    Middleware::g_usb_communication_interface.Register(
        static_cast<uint8_t>(Middleware::CommFrameId::kEarthBlock),
        &rx_buffer_,
        &tx_buffer_,
        static_cast<uint8_t>(sizeof(rx_buffer_)),
        static_cast<uint8_t>(sizeof(tx_buffer_)));
}

void EarthBlockInteract::UsbSetTarget()
{
    if (tx_buffer_.pump_target[0] != last_pump_target_[0])
    {
        const bool accepted = (tx_buffer_.pump_target[0] != 0)
            ? earth_block_->SuckAllCups()
            : earth_block_->ReleaseAllCups();
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

    // 导轨目标每拍直接跟随上位机最新值。
    // 这里不能做成"与上次下发值比较"的死区：校准完成时内部目标会被清 0，
    // 若上位机目标一直不变，死区判断会让这个目标永远不再下发。
    if (std::isfinite(tx_buffer_.motor_position_mm))
    {
        earth_block_->MoveMotorRailTo(tx_buffer_.motor_position_mm);
    }
}

void EarthBlockInteract::UsbFeedback()
{
    // 导轨校准完成前，位置/速度是相对上电时随意读的编码器零点算出来的，没有物理意义；
    // 这里不填，免得校准期间把假位置发给上位机。
    if (earth_block_->IsMotorRailCalibrated())
    {
        rx_buffer_.motor.position = earth_block_->GetMotorRailPositionMm();
        rx_buffer_.motor.velocity = earth_block_->GetMotorRailSpeedMmps();
    }
    rx_buffer_.pump_status[0] =
        static_cast<uint8_t>(earth_block_->GetAllSuctionStatus());
    rx_buffer_.pump_status[1] =
        static_cast<uint8_t>(earth_block_->GetAirRailStatus());
}

} // namespace Module
