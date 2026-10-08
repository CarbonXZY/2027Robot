/**
 * @file mod_chassis.h
 * @brief 麦克纳姆底盘（4 轮）控制，与上位机 R2_PC 的 chassis_system 配对
 *
 * 一帧四个轮，id=1，与上位机 URDF 的 chassis_id 默认值一致：
 *   下行（上位机 → 本板）[id=1][4×float velocity]              16B
 *   上行（本板 → 上位机）[id=1][4×{velocity,position}]         32B
 * 上下行结构体名按「电机视角」命名：Tx = 下发给电机的指令，Rx = 电机回传的反馈。
 *
 * 麦轮运动学解算在上位机的 mecanum_drive_controller 里做，本模块只做执行：
 * 把下发的目标轮速写给 4 个 M3508，并回传反馈，不做任何逻辑判断。
 */

#ifndef MOD_CHASSIS_H
#define MOD_CHASSIS_H

#include <array>
#include <cstdint>

#include "dvc_motor_base.h"

namespace Module
{

/**
 * @brief 轮子数量：麦克纳姆底盘 4 个电机，与上位机 chassis 的轮数一致
 */
constexpr uint8_t kChassisCount = 4;

/**
 * @brief 底盘使能状态
 */
enum class ChassisStatus
{
    kDisable = 0,  // 失能：目标轮速给 0
    kEnable,       // 使能：跟随上位机轮速
};

#pragma pack(push, 1)
/**
 * @brief 底盘发送结构体（下发指令）：每个轮的目标速度
 *
 * 线上方向：上位机 → 本板（上位机 chassis 的 Tx 结构体，16B）
 */
struct ChassisTx
{
    float velocity[kChassisCount];
};

/**
 * @brief 底盘接收结构体（电机回传）：每个轮速度 + 位置
 *
 * 线上方向：本板 → 上位机（上位机 chassis 的 Rx 结构体，32B）
 * Device::MotorFeedbackFrame 为 {velocity, position}，两端布局必须一致
 */
struct ChassisRx
{
    Device::MotorFeedbackFrame motor[kChassisCount];
};
#pragma pack(pop)

class Chassis
{
public:
    /**
     * @brief 底盘控制参数
     */
    struct Parameters
    {
        // ±1：电机安装方向，1=与麦轮正向同向，-1=镜像反向
        // Init 里归一到 ±1，0 视为 1
        int8_t direction_sign[kChassisCount] = {};
    };

    /**
     * @brief 绑定各轮电机指针，需已接好反馈回调
     */
    explicit Chassis(std::array<Device::MotorBase *, kChassisCount> chassis_motor);

    /**
     * @brief 写入控制参数，并把 id=1 的两帧注册进通信中间件
     */
    void Init(const Parameters &parameters);

    /**
     * @brief 使能 / 失能底盘（失能时目标轮速给 0，PC 掉线保护用）
     */
    void SetState(ChassisStatus status);

    /**
     * @brief 1ms 控制周期：目标轮速下发 + 反馈回传
     */
    void TimCalculatePeriodElapsedCallback();

    // 下行：上位机下发的目标轮速，通信中间件直接写入
    ChassisTx tx_buffer_{};
    // 上行：回传给上位机的电机反馈
    ChassisRx rx_buffer_{};

private:
    std::array<Device::MotorBase *, kChassisCount> chassis_motor_{}; // 各轮电机

    Parameters param_{}; // 底盘控制参数

    ChassisStatus status_ = ChassisStatus::kDisable;
};

} // namespace Module

#endif
