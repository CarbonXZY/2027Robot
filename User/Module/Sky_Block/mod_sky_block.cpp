/**
 * @file mod_sky_block.cpp
 * @brief Sky Block 直接调用实现
 */

#include "mod_sky_block.h"

#include <cstddef>
#include <cstdint>

#include "Communication_Interface.hpp"
#include "mid_config.h"

namespace Module
{

SkyBlock::SkyBlock(
    Device::MotorDjiC610 &big_arm,
    Device::MotorDjiC620 &small_arm,
    Device::MotorDjiC610 &gripper)
    : big_arm_(&big_arm),
      small_arm_(&small_arm),
      gripper_(&gripper)
{
}

bool SkyBlock::Init(const HardwareConfig &hardware)
{
    Initialized = false;

    if (hardware.Wrist_Timer == nullptr ||
        hardware.Wrist_Channel == 0 ||
        hardware.Wrist_Min_Pulse_Us >= hardware.Wrist_Max_Pulse_Us ||
        big_arm_ == nullptr ||
        small_arm_ == nullptr ||
        gripper_ == nullptr ||
        !big_arm_->IsInitialized() ||
        !small_arm_->IsInitialized() ||
        !gripper_->IsInitialized())
    {
        return false;
    }

    Hardware = hardware;
    BigArm.Direction_Sign = BigArm.Direction_Sign >= 0.0f ? 1.0f : -1.0f;
    SmallArm.Direction_Sign = SmallArm.Direction_Sign >= 0.0f ? 1.0f : -1.0f;
    Gripper.Direction_Sign = Gripper.Direction_Sign >= 0.0f ? 1.0f : -1.0f;

    BigArm_State = {};
    SmallArm_State = {};
    Gripper_State = {};

    BigArm_State.Offset_Rad = big_arm_->GetPosition();
    SmallArm_State.Offset_Rad = small_arm_->GetPosition();
    Gripper_State.Offset_Rad = gripper_->GetPosition();

    big_arm_->SetControlMethod(Device::MotorControlMethod::kCurrent);
    big_arm_->SetTargetCurrent(0.0f);
    small_arm_->SetControlMethod(Device::MotorControlMethod::kCurrent);
    small_arm_->SetTargetCurrent(0.0f);
    gripper_->SetControlMethod(Device::MotorControlMethod::kCurrent);
    gripper_->SetTargetCurrent(0.0f);

    Wrist_Servo.Init(
        hardware.Wrist_Timer,
        hardware.Wrist_Channel,
        hardware.Wrist_Min_Pulse_Us,
        hardware.Wrist_Max_Pulse_Us);

    if (Wrist_Servo.Get_State() !=
            Class_Servo_Base::Enum_Servo_State::Servo_State_Enable ||
        HAL_TIM_PWM_Start(hardware.Wrist_Timer, hardware.Wrist_Channel) != HAL_OK)
    {
        return false;
    }

    Wrist_Servo.Set_Normalized_Position(hardware.Wrist_Safe_Position);

    Initialized = true;
    interact_.Init(this);
    Calibrate_All();
    return true;
}

bool SkyBlock::Move_BigArm_To(float height_mm)
{
    if (!Initialized || !std::isfinite(height_mm))
    {
        return false;
    }

    height_mm = height_mm < BigArm.Min_Mm
        ? BigArm.Min_Mm
        : (height_mm > BigArm.Max_Mm ? BigArm.Max_Mm : height_mm);

    BigArm_State.Target_Output = height_mm;
    BigArm_State.Control_Enabled = true;
    return true;
}

bool SkyBlock::Move_SmallArm_To(float angle_deg)
{
    if (!Initialized || !std::isfinite(angle_deg))
    {
        return false;
    }

    angle_deg = angle_deg < SmallArm.Min_Angle_Deg
        ? SmallArm.Min_Angle_Deg
        : (angle_deg > SmallArm.Max_Angle_Deg ? SmallArm.Max_Angle_Deg : angle_deg);

    SmallArm_State.Target_Output = angle_deg * PI / 180.0f;
    SmallArm_State.Control_Enabled = true;
    return true;
}

bool SkyBlock::Set_Wrist_Position(float normalized_position)
{
    if (!Initialized || !std::isfinite(normalized_position))
    {
        return false;
    }

    normalized_position = normalized_position < 0.0f
        ? 0.0f
        : (normalized_position > 1.0f ? 1.0f : normalized_position);
    Wrist_Servo.Set_Normalized_Position(normalized_position);
    return true;
}

bool SkyBlock::Move_Gripper_To(float position_mm)
{
    if (!Initialized || !std::isfinite(position_mm))
    {
        return false;
    }

    position_mm = position_mm < Gripper.Min_Mm
        ? Gripper.Min_Mm
        : (position_mm > Gripper.Max_Mm ? Gripper.Max_Mm : position_mm);

    Gripper_State.Target_Output = position_mm;
    Gripper_State.Control_Enabled = true;
    return true;
}

bool SkyBlock::Calibrate_All()
{
    if (!Initialized)
    {
        return false;
    }

    BigArm_State.Calibrated = false;
    BigArm_State.Calibration_Active = true;
    BigArm_State.Control_Enabled = false;

    SmallArm_State.Calibrated = false;
    SmallArm_State.Calibration_Active = true;
    SmallArm_State.Control_Enabled = false;

    Gripper_State.Calibrated = false;
    Gripper_State.Calibration_Active = true;
    Gripper_State.Control_Enabled = false;
    return true;
}
void SkyBlock::Stop_All()
{
    if (!Initialized)
    {
        return;
    }

    BigArm_State.Calibration_Active = false;
    SmallArm_State.Calibration_Active = false;
    Gripper_State.Calibration_Active = false;
    BigArm_State.Control_Enabled = false;
    SmallArm_State.Control_Enabled = false;
    Gripper_State.Control_Enabled = false;

    big_arm_->SetControlMethod(Device::MotorControlMethod::kCurrent);
    big_arm_->SetTargetCurrent(0.0f);
    small_arm_->SetControlMethod(Device::MotorControlMethod::kCurrent);
    small_arm_->SetTargetCurrent(0.0f);
    gripper_->SetControlMethod(Device::MotorControlMethod::kCurrent);
    gripper_->SetTargetCurrent(0.0f);

    Wrist_Servo.Set_Normalized_Position(Hardware.Wrist_Safe_Position);
}

void SkyBlock::TimCalculate1msCallback()
{
    interact_.TimCalculate1msCallback();
}

void SkyBlock::TimAlive100msCallback()
{
    big_arm_->Tim100msAlivePeriodElapsedCallback();
    small_arm_->Tim100msAlivePeriodElapsedCallback();
    gripper_->Tim100msAlivePeriodElapsedCallback();
}

void SkyBlock::Calculate()
{
    if (!Initialized)
    {
        return;
    }

    Update_Feedback();

    if (BigArm_State.Calibration_Active)
    {
        Update_BigArm_Calibration();
    }
    if (SmallArm_State.Calibration_Active)
    {
        Update_SmallArm_Calibration();
    }
    if (Gripper_State.Calibration_Active)
    {
        Update_Gripper_Calibration();
    }

    Apply_Gravity_Compensation();
    Calculate_Motors();
}

void SkyBlock::Update_Feedback()
{
    big_arm_->UpdateFeedback();
    small_arm_->UpdateFeedback();
    gripper_->UpdateFeedback();
    Update_BigArm_Feedback();
    Update_SmallArm_Feedback();
    Update_Gripper_Feedback();
}

void SkyBlock::Update_BigArm_Feedback()
{
    BigArm_State.Current_Rad =
        (big_arm_->GetPosition() - BigArm_State.Offset_Rad) *
        BigArm.Direction_Sign;
    BigArm_State.Current_Speed_Radps =
        big_arm_->GetSpeed() * BigArm.Direction_Sign;
    BigArm_State.Current_Output =
        BigArm_State.Current_Rad * BigArm.Mm_Per_Motor_Rad;
    BigArm_State.Current_Speed_Output =
        BigArm_State.Current_Speed_Radps * BigArm.Mm_Per_Motor_Rad;
}

void SkyBlock::Update_SmallArm_Feedback()
{
    const float raw_position = small_arm_->GetPosition();
    const float wrapped_motor_rad =
        std::remainder(
            raw_position - SmallArm.Coordinate_Zero_Raw_Rad,
            2.0f * PI);
    const float motor_relative_rad =
        wrapped_motor_rad * SmallArm.Direction_Sign;
    const float motor_relative_speed_radps =
        small_arm_->GetSpeed() * SmallArm.Direction_Sign;
    const float arm_rad_per_motor_rad =
        SmallArm.Arm_Deg_Per_Motor_Rad * PI / 180.0f;

    SmallArm_State.Current_Rad = motor_relative_rad;
    SmallArm_State.Current_Speed_Radps = motor_relative_speed_radps;
    SmallArm_State.Current_Output =
        motor_relative_rad * arm_rad_per_motor_rad;
    SmallArm_State.Current_Speed_Output =
        motor_relative_speed_radps * arm_rad_per_motor_rad;
}

void SkyBlock::Update_Gripper_Feedback()
{
    Gripper_State.Current_Rad =
        (gripper_->GetPosition() - Gripper_State.Offset_Rad) *
        Gripper.Direction_Sign;
    Gripper_State.Current_Speed_Radps =
        gripper_->GetSpeed() * Gripper.Direction_Sign;
    Gripper_State.Current_Output =
        Gripper_State.Current_Rad * Gripper.Mm_Per_Motor_Rad;
    Gripper_State.Current_Speed_Output =
        Gripper_State.Current_Speed_Radps * Gripper.Mm_Per_Motor_Rad;
}

void SkyBlock::Apply_Gravity_Compensation()
{
    const float big_arm_current =
        Gravity.BigArm_Enabled &&
                BigArm_State.Calibrated &&
                !BigArm_State.Calibration_Active
            ? Gravity.BigArm_Offset_A
            : 0.0f;
    big_arm_->SetFeedforwardCurrent(big_arm_current);

    const float small_arm_current =
        Gravity.SmallArm_Enabled &&
                SmallArm_State.Calibrated &&
                !SmallArm_State.Calibration_Active
            ? Gravity.SmallArm_Cos_Gain_A *
                std::cos(SmallArm_State.Current_Output)
            : 0.0f;
    small_arm_->SetFeedforwardCurrent(small_arm_current);
}

void SkyBlock::Calculate_Motors()
{
    if (!BigArm_State.Calibration_Active)
    {
        if (BigArm_State.Control_Enabled && BigArm_State.Calibrated)
        {
            const float target_rad =
                BigArm_State.Offset_Rad +
                BigArm.Direction_Sign * BigArm_State.Target_Output /
                    BigArm.Mm_Per_Motor_Rad;
            const float error = target_rad - big_arm_->GetPosition();
            if (std::fabs(error) > BigArm.Switch_Error_Rad)
            {
                big_arm_->pid_position.SetIntegralError(0.0f);
                big_arm_->SetControlMethod(Device::MotorControlMethod::kSpeed);
                big_arm_->SetTargetSpeed(
                    error > 0.0f
                        ? BigArm.Approach_Speed_Radps
                        : -BigArm.Approach_Speed_Radps);
            }
            else
            {
                big_arm_->SetControlMethod(Device::MotorControlMethod::kPosition);
                big_arm_->SetTargetPosition(target_rad);
            }
        }
        else
        {
            big_arm_->SetControlMethod(Device::MotorControlMethod::kCurrent);
            big_arm_->SetTargetCurrent(0.0f);
        }
    }

    if (!SmallArm_State.Calibration_Active)
    {
        if (SmallArm_State.Control_Enabled && SmallArm_State.Calibrated)
        {
            const float arm_rad_per_motor_rad =
                SmallArm.Arm_Deg_Per_Motor_Rad * PI / 180.0f;
            const float current_raw_position = small_arm_->GetPosition();
            const float current_wrapped_motor_rad =
                std::remainder(
                    current_raw_position - SmallArm.Coordinate_Zero_Raw_Rad,
                    2.0f * PI);
            const float zero_raw_near_current =
                current_raw_position - current_wrapped_motor_rad;
            const float target_rad =
                zero_raw_near_current +
                SmallArm.Direction_Sign * SmallArm_State.Target_Output /
                    arm_rad_per_motor_rad;
            const float error =
                std::remainder(target_rad - current_raw_position, 2.0f * PI);
            if (std::fabs(error) > SmallArm.Switch_Error_Rad)
            {
                small_arm_->pid_position.SetIntegralError(0.0f);
                small_arm_->SetControlMethod(Device::MotorControlMethod::kSpeed);
                small_arm_->SetTargetSpeed(
                    error > 0.0f
                        ? SmallArm.Approach_Speed_Radps
                        : -SmallArm.Approach_Speed_Radps);
            }
            else
            {
                small_arm_->SetControlMethod(Device::MotorControlMethod::kPosition);
                small_arm_->SetTargetPosition(target_rad);
            }
        }
        else
        {
            small_arm_->SetControlMethod(Device::MotorControlMethod::kCurrent);
            small_arm_->SetTargetCurrent(0.0f);
        }
    }

    if (!Gripper_State.Calibration_Active)
    {
        if (Gripper_State.Control_Enabled && Gripper_State.Calibrated)
        {
            const float target_rad =
                Gripper_State.Offset_Rad +
                Gripper.Direction_Sign * Gripper_State.Target_Output /
                    Gripper.Mm_Per_Motor_Rad;
            const float error = target_rad - gripper_->GetPosition();
            if (std::fabs(error) > Gripper.Switch_Error_Rad)
            {
                gripper_->pid_position.SetIntegralError(0.0f);
                gripper_->SetControlMethod(Device::MotorControlMethod::kSpeed);
                gripper_->SetTargetSpeed(
                    error > 0.0f
                        ? Gripper.Approach_Speed_Radps
                        : -Gripper.Approach_Speed_Radps);
            }
            else
            {
                gripper_->SetControlMethod(Device::MotorControlMethod::kPosition);
                gripper_->SetTargetPosition(target_rad);
            }
        }
        else
        {
            gripper_->SetControlMethod(Device::MotorControlMethod::kCurrent);
            gripper_->SetTargetCurrent(0.0f);
        }
    }

    big_arm_->Calculate();
    small_arm_->Calculate();
    gripper_->Calculate();
}

bool SkyBlock::Update_BigArm_Calibration()
{
    if (!big_arm_->Calibrate(BigArm.Calibration, BigArm_State.Offset_Rad))
    {
        return false;
    }
    BigArm_State.Target_Output = 0.0f;
    BigArm_State.Calibrated = true;
    BigArm_State.Calibration_Active = false;
    BigArm_State.Control_Enabled = true;
    Update_BigArm_Feedback();
    return true;
}

bool SkyBlock::Update_SmallArm_Calibration()
{
    if (!small_arm_->Calibrate(SmallArm.Calibration, SmallArm_State.Offset_Rad))
    {
        return false;
    }
    SmallArm_State.Calibrated = true;
    SmallArm_State.Calibration_Active = false;
    Update_SmallArm_Feedback();
    // 水平坐标零点与校准端点独立，校准结束后原地保持。
    SmallArm_State.Target_Output = SmallArm_State.Current_Output;
    SmallArm_State.Control_Enabled = true;
    return true;
}

bool SkyBlock::Update_Gripper_Calibration()
{
    if (!gripper_->Calibrate(Gripper.Calibration, Gripper_State.Offset_Rad))
    {
        return false;
    }
    Gripper_State.Target_Output = 0.0f;
    Gripper_State.Calibrated = true;
    Gripper_State.Calibration_Active = false;
    Gripper_State.Control_Enabled = true;
    Update_Gripper_Feedback();
    return true;
}

void SkyBlockInteract::Init(SkyBlock *sky_block)
{
    sky_block_ = sky_block;
    if (sky_block_ == nullptr)
    {
        return;
    }

    UsbRegister();
    last_big_arm_target_mm_ = tx_buffer_.big_arm_target_mm;
    last_small_arm_target_deg_ = tx_buffer_.small_arm_target_deg;
    last_wrist_target_ = tx_buffer_.wrist_target;
    last_gripper_target_mm_ = tx_buffer_.gripper_target_mm;
}

void SkyBlockInteract::TimCalculate1msCallback()
{
    if (sky_block_ == nullptr)
    {
        return;
    }

    UsbSetTarget();
    sky_block_->Calculate();
    UsbFeedback();
}

void SkyBlockInteract::UsbRegister()
{
    Middleware::g_usb_communication_interface.Register(
        static_cast<uint8_t>(Middleware::CommFrameId::kSkyBlock),
        &rx_buffer_,
        &tx_buffer_,
        static_cast<uint8_t>(sizeof(rx_buffer_)),
        static_cast<uint8_t>(sizeof(tx_buffer_)));
}

void SkyBlockInteract::UsbSetTarget()
{
    if (std::isfinite(tx_buffer_.big_arm_target_mm) &&
        std::fabs(tx_buffer_.big_arm_target_mm - last_big_arm_target_mm_) > 0.1f)
    {
        if (sky_block_->Move_BigArm_To(tx_buffer_.big_arm_target_mm))
        {
            last_big_arm_target_mm_ = tx_buffer_.big_arm_target_mm;
        }
    }

    if (std::isfinite(tx_buffer_.small_arm_target_deg) &&
        std::fabs(tx_buffer_.small_arm_target_deg - last_small_arm_target_deg_) > 0.1f)
    {
        if (sky_block_->Move_SmallArm_To(tx_buffer_.small_arm_target_deg))
        {
            last_small_arm_target_deg_ = tx_buffer_.small_arm_target_deg;
        }
    }

    if (std::isfinite(tx_buffer_.wrist_target) &&
        std::fabs(tx_buffer_.wrist_target - last_wrist_target_) > 0.01f)
    {
        if (sky_block_->Set_Wrist_Position(tx_buffer_.wrist_target))
        {
            last_wrist_target_ = tx_buffer_.wrist_target;
        }
    }

    if (std::isfinite(tx_buffer_.gripper_target_mm) &&
        std::fabs(tx_buffer_.gripper_target_mm - last_gripper_target_mm_) > 0.1f)
    {
        if (sky_block_->Move_Gripper_To(tx_buffer_.gripper_target_mm))
        {
            last_gripper_target_mm_ = tx_buffer_.gripper_target_mm;
        }
    }
}

void SkyBlockInteract::UsbFeedback()
{
    // 校准完成前反馈零点还会被改写，位置没有意义；先不填，上位机看到的就是 0。
    if (!sky_block_->Is_Calibrated())
    {
        return;
    }

    rx_buffer_.big_arm.position = sky_block_->Get_BigArm_Position_Mm();
    rx_buffer_.big_arm.velocity = sky_block_->Get_BigArm_Speed_Mmps();
    rx_buffer_.small_arm.position = sky_block_->Get_SmallArm_Angle_Deg();
    rx_buffer_.small_arm.velocity = sky_block_->Get_SmallArm_Speed_Degps();
    rx_buffer_.gripper.position = sky_block_->Get_Gripper_Position_Mm();
    rx_buffer_.gripper.velocity = sky_block_->Get_Gripper_Speed_Mmps();
}

} // namespace Module
