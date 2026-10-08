/**
 * @file mod_leg.h
 * @brief 腿机构（4 关节）控制，与上位机 R2_PC 的 leg_system 配对
 *
 * 一帧四条腿，id=3，与上位机 URDF 的 leg_id 默认值一致：
 *   下行（上位机 → 本板）[id=3][4×float position]          16B
 *   上行（本板 → 上位机）[id=3][4×{velocity,position}]     32B
 * 上下行结构体名按「电机视角」命名：Tx = 下发给电机的指令，Rx = 电机回传的反馈。
 */

#ifndef MOD_LEG_H
#define MOD_LEG_H

#include <array>
#include <cstdint>

#include "dvc_motor_base.h"

namespace Module
{

/**
 * @brief 腿的数量：四条腿，每条腿一个关节电机
 * 与上位机 leg 的腿数一致，一帧装下四条腿
 */
constexpr uint8_t kLegCount = 4;

/**
 * @brief 腿机构使能状态
 */
enum class LegStatus
{
    kDisable = 0,  // 失能：不再接受新指令，稳定在最后一帧目标位置
    kEnable,       // 使能：跟随上位机指令
};

#pragma pack(push, 1)
/**
 * @brief 腿发送结构体（下发指令）：每条腿的目标位置
 *
 * 线上方向：上位机 → 本板（上位机 leg 的 Tx 结构体，16B）
 */
struct LegTx
{
    float position[kLegCount];
};

/**
 * @brief 腿接收结构体（电机回传）：每条腿速度 + 位置
 *
 * 线上方向：本板 → 上位机（上位机 leg 的 Rx 结构体，32B）
 * Device::MotorFeedbackFrame 为 {velocity, position}，两端布局必须一致
 */
struct LegRx
{
    Device::MotorFeedbackFrame motor[kLegCount];
};
#pragma pack(pop)

class Leg
{
public:
    /**
     * @brief 腿控制参数（照 R1 crt_multi_motor_sync 的做法集中配置）
     */
    struct Parameters
    {
        // 远距离开环逼近速度 (rad/s)
        float max_velocity = 5.0f;

        // 短距离位置环锁定阈值 (rad)：
        // |目标 - 反馈| 大于它以速度模式逼近，进入以内切位置环锁定
        float position_lock_threshold = 0.1f;

        // ±1：电机安装方向，1=与关节正向同向，-1=镜像反向
        // Init 里归一到 ±1，0 视为 1
        int8_t direction_sign[kLegCount] = {};

        Device::CalibrateParams calibrate{};
    };

    /**
     * @brief 绑定各腿电机指针，需已接好反馈回调
     */
    explicit Leg(std::array<Device::MotorBase *, kLegCount> leg_motor);

    /**
     * @brief 写入控制参数，并把 id=3 的两帧注册进通信中间件
     */
    void Init(const Parameters &parameters);

    /**
     * @brief 1ms 控制周期：目标位置下发 + 反馈回传
     */
    void TimCalculatePeriodElapsedCallback();

    /**
     * @brief kDisable 时不再接受新指令，稳定在最后一帧目标位置（PC 掉线保护）
     */
    void SetState(LegStatus status);

    /**
     * @brief 四条腿是否都标定完成
     */
    bool IsCalibrated() const
    {
        return calibrated_;
    }

    // 下行：上位机下发的目标位置，通信中间件直接写入
    LegTx tx_buffer_{};
    // 上行：回传给上位机的电机反馈
    LegRx rx_buffer_{};

private:
    std::array<Device::MotorBase *, kLegCount> leg_motor_{}; // 各腿电机

    float target_position_[kLegCount]{};   // 目标位置 (rad)
    float feedback_position_[kLegCount]{}; // 反馈位置 (rad)
    float feedback_velocity_[kLegCount]{}; // 反馈速度 (rad/s)

    float offset_[kLegCount]{};          // 各腿机械零点, 电机侧原始角 (rad)
    bool calibrated_motor_[kLegCount]{}; // 单腿标定完成标记

    Parameters param_{}; // 腿控制参数

    bool calibrated_ = false; // 校准标志位

    LegStatus status_ = LegStatus::kDisable;

    void MoveToPosition();

    void Calibrate();
};

} // namespace Module

#endif
