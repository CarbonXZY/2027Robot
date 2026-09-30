/**
 * @file dvc_motor_base.h
 * @author hzy
 * @brief 通用电机配置与操作
 * @version 0.1
 * @date 2026-05-28
 *
 * @copyright NEUQ (cpp) 2026
 */


#ifndef DVC_MOTOR_BASE_H
#define DVC_MOTOR_BASE_H

#include <stdint.h>
#include "drv_math.h"
#include "dvc_dwt.h"

namespace Device
{

/**
 * @brief 通用电机状态
 */
enum class MotorStatus
{
    kDisable = 0,  // 电机离线 / 失能
    kEnable,       // 电机在线 / 使能
};

#pragma pack(push, 1)
struct MotorFeedbackFrame
{
    float velocity = 0.0f;  // 速度 (rad/s)
    float position = 0.0f;  // 位置 (rad)
};
#pragma pack(pop)

/**
 * @brief 通用电机控制模式
 *
 * 这个枚举只描述上层希望电机处于什么控制模式，
 * 具体如何转换成 CAN 指令、PWM、ERPM、电流输出，由子类/适配器负责。
 */
enum class MotorControlMethod
{
    kDisable = 0,  // 失能 / 零输出
    kCurrent,      // 电流模式，单位 A
    kSpeed,        // 速度模式，单位由子类约定，舵轮工程中建议统一为 rad/s 或 m/s
    kPosition,     // 位置模式，单位 rad
    kMit,          // 预留 MIT 模式
    kDuty,         // 预留占空比模式，单位 %
    kBrake,        // 预留刹车模式，单位 A
};

/**
 * @brief 校准运动模式
 */
enum class CalibrateMotionMode
{
    kNone = 0,     // 不运动，直接以当前位置为 offset
    kSpeed,        // 恒速运动
    kCurrent,      // 恒流运动
};

/**
 * @brief 校准堵转检测模式
 */
enum class CalibrateDetectMode
{
    kCurrent = 0,  // 电流超过阈值判定堵转
    kSpeed,        // 速度低于阈值判定堵转
};

/**
 * @brief 校准参数
 *
 * 用法:
 *   CalibrateParams p;
 *   p.motion_mode = CalibrateMotionMode::kSpeed;
 *   p.motion_value = -0.3f;
 *   p.detect_mode = CalibrateDetectMode::kCurrent;
 *   p.detect_threshold = 5.0f;
 *   mot.Calibrate(p, offset);
 */
struct CalibrateParams
{
    CalibrateMotionMode motion_mode = CalibrateMotionMode::kSpeed;
    float motion_value = 0.3f;                              // 速度(rad/s)或电流(A), 正负决定方向

    CalibrateDetectMode detect_mode = CalibrateDetectMode::kCurrent;
    float detect_threshold = 5.0f;                          // 电流阈值(A)或速度阈值(rad/s)

    uint32_t debounce_us = 0;                               // 检测消抖时间(us), 条件持续满足此时间后判定堵转; 0=立即判定
};

/**
 * @brief 通用电机抽象基类
 *
 * 设计原则：
 * 1. 基类只定义统一接口，不关心具体电机协议。
 * 2. DJI、MKSESC、VESC、达妙等具体电机通过继承或适配器实现这些接口。
 * 3. 上层模块如 SwerveModule 只依赖 MotorBase。
 *
 * 单位建议：
 * - Current: A
 * - Position: rad
 * - Speed:
 *   - 舵向电机建议 rad/s
 *   - 轮向电机在舵轮模块中建议使用 m/s，由轮向适配器内部转换成 ERPM/rad/s
 */
class MotorBase
{
public:
    virtual ~MotorBase() {}

    /**
     * @brief 设置电机控制模式
     */
    virtual void SetControlMethod(MotorControlMethod method) = 0;

    /**
     * @brief 设置目标电流，单位 A
     */
    virtual void SetTargetCurrent(float target_current) = 0;

    /**
     * @brief 设置目标速度
     *
     * 对于舵向电机，建议单位 rad/s。
     * 对于轮向电机，在舵轮系统中建议单位 m/s，
     * 由具体轮向电机适配器换算为电机自身单位。
     */
    virtual void SetTargetSpeed(float target_speed) = 0;

    /**
     * @brief 设置目标位置，单位 rad
     */
    virtual void SetTargetPosition(float target_position) = 0;

    /**
     * @brief 写入外部反馈电流，单位 A
     *
     * 某些电机反馈来自自身 CAN；某些闭环反馈来自外部传感器。
     * 例如舵向电机位置反馈常来自绝对编码器。
     */
    virtual void SetFeedbackCurrent(float feedback_current) = 0;

    /**
     * @brief 写入外部反馈速度（单位由子类约定）
     */
    virtual void SetFeedbackSpeed(float feedback_speed) = 0;

    /**
     * @brief 写入外部反馈位置，单位 rad
     */
    virtual void SetFeedbackPosition(float feedback_position) = 0;

    /**
     * @brief 获取当前反馈电流，单位 A
     */
    virtual float GetCurrent() const = 0;

    /**
     * @brief 获取当前反馈速度（单位由子类约定）
     */
    virtual float GetSpeed() const = 0;

    /**
     * @brief 获取当前反馈位置，单位 rad
     */
    virtual float GetPosition() const = 0;

    /**
     * @brief 更新反馈 — 从底层读取最新数据到统一接口
     *
     * 对于 CAN 电机：从底层 CAN 帧缓存同步到 feedback_current_/speed/position
     * 对于外部传感器反馈：上层已通过 SetFeedback* 写入，此函数可作为空操作
     */
    virtual void UpdateFeedback() = 0;

    /**
     * @brief 计算控制量 — 根据当前模式和目标值计算输出
     *
     * 典型实现：
     * - CURRENT 模式：直接使用目标电流（可叠加前馈）
     * - SPEED 模式：速度 PID 输出电流目标
     * - POSITION 模式：位置外环 → 速度目标 → 速度内环 → 电流目标
     */
    virtual void Calculate() = 0;

    /**
     * @brief 输出控制量
     *
     * 例如：
     * - 写入 CAN 发送缓冲区
     * - 直接发送 CAN 指令
     * - 输出 PWM
     */
    virtual void Output() = 0;

    /**
     * @brief 电机堵转校准
     *
     * 每控制周期调用一次。支持恒速 / 恒流两种运动模式，
     * 支持电流阈值 / 速度阈值两种堵转判定方式，带 DWT 消抖。
     * 校准完成后将电机置零，通过 offset 导出机械零点绝对角度。
     *
     * @param params  校准参数 (运动/检测模式, 阈值, 消抖时间)
     * @param offset  输出，堵转时的机械零点绝对角度
     * @return true   校准完成 (堵转检测到)
     * @return false  仍在校准中
     */
    bool Calibrate(const CalibrateParams &params, float &offset);

protected:
    uint32_t stall_debounce_start_time_ = 0; // 堵转消抖计时起点
};

} // namespace Device

#endif
