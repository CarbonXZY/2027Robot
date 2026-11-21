/**
 * @file mod_sky_block.h
 * @brief Sky Block：大臂、小臂、手腕、夹爪
 */

#ifndef MOD_SKY_BLOCK_H
#define MOD_SKY_BLOCK_H

#include <cmath>
#include <cstdint>

#include "main.h"
#include "dvc_ds_servo.h"
#include "dvc_motor_dji.h"

namespace Module
{

class SkyBlock;

#pragma pack(push, 1)
struct SkyBlockTx
{
    float big_arm_target_mm;
    float small_arm_target_deg;
    float wrist_target;
    float gripper_target_mm;
};

struct SkyBlockRx
{
    Device::MotorFeedbackFrame big_arm;
    Device::MotorFeedbackFrame small_arm;
    Device::MotorFeedbackFrame gripper;
};
#pragma pack(pop)

class SkyBlockInteract
{
public:
    SkyBlockInteract() = default;
    SkyBlockInteract(const SkyBlockInteract &) = delete;
    SkyBlockInteract &operator=(const SkyBlockInteract &) = delete;

    void Init(SkyBlock *sky_block);
    void TimCalculate1msCallback();

    SkyBlockTx tx_buffer_{};
    SkyBlockRx rx_buffer_{};

private:
    SkyBlock *sky_block_ = nullptr;

    float last_big_arm_target_mm_ = 0.0f;
    float last_small_arm_target_deg_ = 0.0f;
    float last_wrist_target_ = 0.5f;
    float last_gripper_target_mm_ = 0.0f;

    void UsbRegister();
    void UsbSetTarget();
    void UsbFeedback();
};

class SkyBlock
{
public:
    struct HardwareConfig
    {
        TIM_HandleTypeDef *Wrist_Timer = nullptr;
        uint32_t Wrist_Channel = 0;
        uint16_t Wrist_Min_Pulse_Us = 500;
        uint16_t Wrist_Max_Pulse_Us = 2500;
        float Wrist_Safe_Position = 0.5f;
    };

    SkyBlock(
        Device::MotorDjiC610 &big_arm,
        Device::MotorDjiC620 &small_arm,
        Device::MotorDjiC610 &gripper);

    bool Init(const HardwareConfig &hardware);

    void TimCalculate1msCallback();
    void TimAlive100msCallback();

    bool Move_BigArm_To(float height_mm);
    bool Move_SmallArm_To(float angle_deg);
    bool Set_Wrist_Position(float normalized_position);
    bool Move_Gripper_To(float position_mm);
    bool Calibrate_All();

    void Stop_All();

    bool Is_Initialized() const { return Initialized; }

    // 三轴全部校准完成
    bool Is_Calibrated() const;

    float Get_BigArm_Position_Mm() const;
    float Get_BigArm_Speed_Mmps() const;
    float Get_BigArm_Target_Mm() const;
    float Get_SmallArm_Angle_Deg() const;
    float Get_SmallArm_Speed_Degps() const;
    float Get_SmallArm_Target_Deg() const;
    float Get_Gripper_Position_Mm() const;
    float Get_Gripper_Speed_Mmps() const;
    float Get_Gripper_Target_Mm() const;

private:
    friend class SkyBlockInteract;

    bool Is_BigArm_Calibrated() const { return BigArm_State.Calibrated; }
    bool Is_SmallArm_Calibrated() const { return SmallArm_State.Calibrated; }
    bool Is_Gripper_Calibrated() const { return Gripper_State.Calibrated; }

    struct BigArmConfig
    {
        float Mm_Per_Motor_Rad = 22.941176f;
        float Direction_Sign = -1.0f;
        float Min_Mm = 0.0f;
        float Max_Mm = 390.0f;
        float Approach_Speed_Radps = 20.0f;
        float Switch_Error_Rad = 0.1f;
        Device::CalibrateParams Calibration = {
            .motion_mode = Device::CalibrateMotionMode::kSpeed,
            .motion_value = 3.0f,
            .detect_mode = Device::CalibrateDetectMode::kSpeed,
            .detect_threshold = 0.1f,
            .debounce_us = 20000,
        };
    };

    struct SmallArmConfig
    {
        float Arm_Deg_Per_Motor_Rad = 56.25f;
        float Direction_Sign = -1.0f;
        float Coordinate_Zero_Raw_Rad = -0.450891048f; // 水平位置对应的 C620 原始输出轴角度
        float Min_Angle_Deg = -60.0f;
        float Max_Angle_Deg = 150.0f;
        float Approach_Speed_Radps = 15.0f;
        float Switch_Error_Rad = 0.1f;
        Device::CalibrateParams Calibration = {
            .motion_mode = Device::CalibrateMotionMode::kSpeed,
            .motion_value = -3.0f,
            .detect_mode = Device::CalibrateDetectMode::kSpeed,
            .detect_threshold = 0.1f,
            .debounce_us = 20000,
        };
    };

    struct GripperConfig
    {
        float Mm_Per_Motor_Rad = 24.0f;
        float Direction_Sign = -1.0f;
        float Min_Mm = 0.0f;
        float Max_Mm = 60.0f;
        float Approach_Speed_Radps = 20.0f;
        float Switch_Error_Rad = 0.1f;
        Device::CalibrateParams Calibration = {
            .motion_mode = Device::CalibrateMotionMode::kSpeed,
            .motion_value = 3.0f,
            .detect_mode = Device::CalibrateDetectMode::kSpeed,
            .detect_threshold = 0.1f,
            .debounce_us = 20000,
        };
    };

    struct GravityCompensationConfig
    {
        bool BigArm_Enabled = true;
        bool SmallArm_Enabled = true;
        float BigArm_Offset_A = -0.76f;
        float SmallArm_Cos_Gain_A = -2.4f;
    };

    struct AxisState
    {
        float Offset_Rad = 0.0f;
        float Current_Rad = 0.0f;
        float Current_Speed_Radps = 0.0f;
        float Current_Output = 0.0f;
        float Current_Speed_Output = 0.0f;
        float Target_Output = 0.0f;
        bool Calibrated = false;
        bool Calibration_Active = false;
        bool Control_Enabled = false;
    };

    Device::MotorDjiC610 *big_arm_ = nullptr;
    Device::MotorDjiC620 *small_arm_ = nullptr;
    Device::MotorDjiC610 *gripper_ = nullptr;

    Class_DS_Servo Wrist_Servo;
    SkyBlockInteract interact_;

    HardwareConfig Hardware = {};
    BigArmConfig BigArm = {};
    SmallArmConfig SmallArm = {};
    GripperConfig Gripper = {};
    GravityCompensationConfig Gravity = {};

    AxisState BigArm_State = {};
    AxisState SmallArm_State = {};
    AxisState Gripper_State = {};
    bool Initialized = false;

    void Calculate();
    void Update_Feedback();
    void Update_BigArm_Feedback();
    void Update_SmallArm_Feedback();
    void Update_Gripper_Feedback();
    void Apply_Gravity_Compensation();
    void Calculate_Motors();
    bool Update_BigArm_Calibration();
    bool Update_SmallArm_Calibration();
    bool Update_Gripper_Calibration();
};

/**
 * @brief 三轴是否全部校准完成
 */
inline bool SkyBlock::Is_Calibrated() const
{
    return Is_BigArm_Calibrated() &&
           Is_SmallArm_Calibrated() &&
           Is_Gripper_Calibrated();
}

/**
 * @brief 获取大臂当前高度
 */
inline float SkyBlock::Get_BigArm_Position_Mm() const
{
    return BigArm_State.Current_Output;
}

/**
 * @brief 获取大臂当前速度
 */
inline float SkyBlock::Get_BigArm_Speed_Mmps() const
{
    return BigArm_State.Current_Speed_Output;
}

/**
 * @brief 获取大臂目标高度
 */
inline float SkyBlock::Get_BigArm_Target_Mm() const
{
    return BigArm_State.Target_Output;
}

/**
 * @brief 获取小臂当前角度
 */
inline float SkyBlock::Get_SmallArm_Angle_Deg() const
{
    return SmallArm_State.Current_Output * 180.0f / PI;
}

/**
 * @brief 获取小臂当前角速度
 */
inline float SkyBlock::Get_SmallArm_Speed_Degps() const
{
    return SmallArm_State.Current_Speed_Output * 180.0f / PI;
}

/**
 * @brief 获取小臂目标角度
 */
inline float SkyBlock::Get_SmallArm_Target_Deg() const
{
    return SmallArm_State.Target_Output * 180.0f / PI;
}

/**
 * @brief 获取夹爪当前位置
 */
inline float SkyBlock::Get_Gripper_Position_Mm() const
{
    return Gripper_State.Current_Output;
}

/**
 * @brief 获取夹爪当前速度
 */
inline float SkyBlock::Get_Gripper_Speed_Mmps() const
{
    return Gripper_State.Current_Speed_Output;
}

/**
 * @brief 获取夹爪目标位置
 */
inline float SkyBlock::Get_Gripper_Target_Mm() const
{
    return Gripper_State.Target_Output;
}

} // namespace Module

#endif
