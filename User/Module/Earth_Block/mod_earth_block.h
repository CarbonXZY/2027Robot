/**
 * @file mod_earth_block.h
 * @brief Earth Block：两路吸盘 + 气动导轨 + C610 电机导轨
 */

#ifndef MOD_EARTH_BLOCK_H
#define MOD_EARTH_BLOCK_H

#include <cmath>
#include <cstdint>

#include "dvc_airtool.h"
#include "dvc_motor_dji.h"

namespace Module
{

class EarthBlock;

#pragma pack(push, 1)
struct EarthBlockTx
{
    float motor_position_mm;
    uint8_t pump_target[2];
};

struct EarthBlockRx
{
    Device::MotorFeedbackFrame motor;
    uint8_t pump_status[2];
};
#pragma pack(pop)

/** @brief USB 通信适配层 */
class EarthBlockInteract
{
public:
    EarthBlockInteract() = default;
    EarthBlockInteract(const EarthBlockInteract &) = delete;
    EarthBlockInteract &operator=(const EarthBlockInteract &) = delete;

    void Init(EarthBlock *earth_block);
    void TimCalculate1msCallback();

    EarthBlockTx tx_buffer_{};
    EarthBlockRx rx_buffer_{};

private:
    EarthBlock *earth_block_ = nullptr;

    uint8_t last_pump_target_[2] = {0, 0};

    void UsbRegister();
    void UsbSetTarget();
    void UsbFeedback();
};

/** @brief Earth Block 机构 */
class EarthBlock
{
public:
    struct HardwareConfig
    {
        GPIO_TypeDef *suction_0_port = nullptr;
        uint16_t suction_0_pin = 0;
        GPIO_TypeDef *suction_1_port = nullptr;
        uint16_t suction_1_pin = 0;
        GPIO_TypeDef *air_rail_port = nullptr;
        uint16_t air_rail_pin = 0;
    };

    explicit EarthBlock(Device::MotorDjiC610 &motor);

    EarthBlock() = delete;
    EarthBlock(const EarthBlock &) = delete;
    EarthBlock &operator=(const EarthBlock &) = delete;

    bool Init(const HardwareConfig &hardware);

    void TimCalculate1msCallback();
    void TimAlive100msCallback();

    bool SuckAllCups();
    bool ReleaseAllCups();
    bool AirRailUp();
    bool AirRailDown();
    bool MoveMotorRailTo(float position_mm);
    bool CalibrateMotorRail();
    void StopMotorRail();

    bool IsInitialized() const;
    bool IsMotorRailCalibrated() const;
    float GetMotorRailPositionMm() const;
    float GetMotorRailSpeedMmps() const;
    float GetMotorRailTargetMm() const;
    float GetMotorRailRawPosition() const;
    Enum_AIRPUMP_Status GetAllSuctionStatus();
    Enum_AIRPUMP_Status GetAirRailStatus();

private:
    friend class EarthBlockInteract;

    struct MotorRailConfig
    {
        float mm_per_motor_rad = 20.0f;
        float direction_sign = 1.0f;
        float min_mm = 0.0f;
        float max_mm = 140.0f;
        float approach_speed_radps = 20.0f;
        float switch_error_rad = 0.1f;

        Device::CalibrateParams calibration = {
            .motion_mode = Device::CalibrateMotionMode::kSpeed,
            .motion_value = -3.0f,
            .detect_mode = Device::CalibrateDetectMode::kSpeed,
            .detect_threshold = 0.1f,
            .debounce_us = 20000,
        };
    };

    Device::MotorDjiC610 *motor_ = nullptr;
    EarthBlockInteract interact_;

    Class_AIRPUMP suction_cup_[2];
    Class_AIRPUMP air_rail_;

    HardwareConfig hardware_ = {};
    MotorRailConfig motor_rail_param_ = {};

    bool suction_enabled_[2] = {false, false};
    bool initialized_ = false;

    float motor_rail_offset_rad_ = 0.0f;
    float motor_rail_current_mm_ = 0.0f;
    float motor_rail_current_speed_mmps_ = 0.0f;
    float motor_rail_target_mm_ = 0.0f;

    bool motor_rail_calibrated_ = false;
    bool motor_rail_calibration_active_ = false;
    bool motor_rail_control_enabled_ = false;

    void Calculate();
    void UpdateMotorRailFeedback();
    void CalculateMotorRail();
    void StopMotorRailControl();
};

/**
 * @brief 是否已初始化
 */
inline bool EarthBlock::IsInitialized() const
{
    return initialized_;
}

/**
 * @brief 导轨是否校准完成
 */
inline bool EarthBlock::IsMotorRailCalibrated() const
{
    return motor_rail_calibrated_;
}

/**
 * @brief 获取导轨当前位置
 */
inline float EarthBlock::GetMotorRailPositionMm() const
{
    return motor_rail_current_mm_;
}

/**
 * @brief 获取导轨当前速度
 */
inline float EarthBlock::GetMotorRailSpeedMmps() const
{
    return motor_rail_current_speed_mmps_;
}

/**
 * @brief 获取导轨目标位置
 */
inline float EarthBlock::GetMotorRailTargetMm() const
{
    return motor_rail_target_mm_;
}

/**
 * @brief 获取导轨电机原始位置
 */
inline float EarthBlock::GetMotorRailRawPosition() const
{
    return motor_->GetPosition();
}

} // namespace Module

#endif
