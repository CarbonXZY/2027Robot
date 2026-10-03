/**
 * @file    mod_gripper.h
 * @author  lyh
 * @date    2026-10-01
 * @brief   夹爪机构测试, 不用上位机, 仅在MCU进行测试
 *
 * 机构描述: M2006驱动齿轮, 齿条带动夹爪动, 水平放置无需重力补偿
 *
 * 方案一:
 *      使用串级PID + 输出电流限幅
 *
 * 方案二:
 *      柔顺控制, 弹簧阻尼模型
 *      计算式: t_out = kp * (x_des - x) + kd * (v_des - v) + t_ff
 *          kp:     弹性系数, 待调
 *          x_des:  目标位置, 要限幅
 *          x:      实际位置, 要校准
 *          kd:     阻尼系数, 待调
 *          v_des:  目标速度, 要限幅
 *          v:      实际速度, 要校准
 *          t_ff:   扭矩前馈, 摩擦补偿 (含方向) + 重力补偿 (恒为0)
 *      明显更符合 "夹取" 的场景
 *
 */
#ifndef MOD_GRIPPER_H
#define MOD_GRIPPER_H

#include "dvc_motor_dji.h"
#include "alg_mit.h"

namespace Module
{
//

class Gripper
{
public:

    enum class Status
    {
        kInit,
        kCalibrate,
        kRunning,
    };

    enum class Event
    {
        kNone = 0,
        kClamp,
        kRelease,
    };

    struct TaskSpace
    {
        float distance;
        float speed;
    };

    struct JointSpace
    {
        float angle;
        float speed;
    };

    explicit Gripper(Device::MotorDjiC610& motor)
        : motor_(&motor)
    {}

    Gripper() = delete;
    Gripper& operator=(const Gripper&) = delete;

    void Init();

    void Clamp() { event_ = Event::kClamp; }
    void Release() { event_ = Event::kRelease; }

    Status GetStatus() const { return status_; }

    void TimCalculate1msCallback();
    void TimAlive100msCallback();


protected:

    // 电机实例
    Device::MotorDjiC610 *motor_;
    // 任务空间MIT控制器
    Algorithm::Mit mit_task_space_;
    // 事件
    Event event_ = Event::kNone;

    struct
    {
        float kp = 170;     // 调参
        float kd = 15;      // 调参
    } mit_param_;

    // 校准参数
    Device::CalibrateParams calibrate_params_ = {
        .motion_mode = Device::CalibrateMotionMode::kSpeed,
        .motion_value = 3.0f,
        .detect_mode = Device::CalibrateDetectMode::kCurrent,
        .detect_threshold = 5.0f,
        .debounce_us = 100*1000,
    };

    // 摩擦补偿
    struct
    {
        float forward = 0.0f;
        float backward = 0.0f;
    } friction_compensation_;

    // 机械参数
    struct
    {
        const float PITCH_RADIUS = 0.0225f;     // 分度圆半径, 单位米
        const int8_t DIRECTION_SIGN = -1;       // 方向符号
        const float CLAMP_DISTANCE = 0.073f;     // 夹取的行程, 单位米
    } mechanical_;

    // 编码器偏移量
    float encoder_offset_ = 0.0f;

    // 目标任务空间参数
    TaskSpace target_task_space_ = {};

    // 实际任务空间参数
    TaskSpace task_space_ = {};

    // 实际关节空间参数
    JointSpace joint_space_ = {};

    float target_current_ = 0.0f;
    float max_current_ = 4.0f;

    // 更新空间参数
    void UpdateSpaceParam();

    // 计算电流
    void CalculateCurrent();

    // 获取摩擦补偿
    float GetFrictionCompensation() const;

    // 输出到电机
    void OutputToMotor();

    // 处理事件输入
    void HandlerEvent();

    // 状态
    Status status_ = Status::kInit;
    void Fsm();
    void Running();

};

//
}

#endif // MOD_GRIPPER_H
