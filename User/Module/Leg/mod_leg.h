/**
* @file mod_leg.h
* @author lyh
* @date 2026-10-19
* @brief 腿控制: 4 个齿轮齿条, 前后两组各自同步
*
* 前左 + 前右为一组, 后左 + 后右为一组。
* 组内用交叉耦合强制同步, 两组之间相互独立, 各走各的虚拟主轴与目标。
* 目标与反馈均为腿的真实高度, 不是电机角度。
*/

#ifndef MOD_LEG_H
#define MOD_LEG_H

#include <array>
#include <cstdint>

#include "alg_cross_couple.h"
#include "alg_interpolator.h"
#include "dvc_motor_dji.h"

namespace Module
{

/**
 * @brief 腿帧结构, id=3, 与上位机 R2_PC 的 leg_system 配对
 *
 * 上下行结构体名按「电机视角」命名: Tx = 下发给电机的指令, Rx = 电机回传的反馈。
 * 下行: 上位机 → 本板 [id=3][4×float 腿高(m)]           16B
 * 上行: 本板 → 上位机 [id=3][4×{velocity,position}]      32B
 *   velocity: 线速度 (m/s), position: 腿高 (m), 均向上为正
 */
#pragma pack(push, 1)
struct LegTx
{
    // 腿高 (m), 向上为正
    float position[4];
};

struct LegRx
{
    Device::MotorFeedbackFrame motor[4];
};
#pragma pack(pop)

/**
 * @brief 上位机链路的使能状态 (掉线保护)
 */
enum class LegStatus
{
    kDisable = 0,  // 失能: 不再接受新目标, 保持最后一帧目标
    kEnable,       // 使能: 跟随上位机下发的目标
};

class Leg
{
public:
    //

    // 固定 4 台电机, 顺序 lf, rf, lr, rr
    static constexpr uint32_t kMotorNum = 4;
    // 每组参与同步的电机数: 前组 lf+rf, 后组 lr+rr
    static constexpr uint32_t kGroupMotorNum = 2;

    // 状态枚举
    enum class Status
    {
        kInit,       // 上电后等待 4 台电机全部在线, 期间不输出
        kCalibrate,  // 主动向下找机械限位, 记录各电机零点
        kRunning,    // 正常跟踪目标高度
        kError,      // 故障锁存, 停止更新目标
    };

    // 前/后两组的腿高，单位 m
    struct Height
    {
        float front = 0.0f;
        float rear = 0.0f;
    };

    // 齿轮齿条机械参数, 下标与电机顺序一致
    struct Mechanical
    {
        float equivalent_radius[kMotorNum]; // 电机转角到齿条直线位移的等效半径 (m/rad)
        int8_t direction_sign[kMotorNum];
        // 前/后组各自允许的最大高度 (m)
        Height max_height = {0.4f, 0.4f};
    };

    // 主动找限位的参数
    struct Homing
    {
        float speed = 0.02f;            // 下降速度, m/s
        float speed_threshold = 0.002f; // 堵转速度阈值, m/s
        uint32_t debounce_ms = 150;     // 堵转消抖时间, ms
    };

    // 4 台电机的统一管理者, 顺序 lf, rf, lr, rr
    struct MotorGroup
    {
        Device::MotorDjiC620* lf = nullptr;
        Device::MotorDjiC620* rf = nullptr;
        Device::MotorDjiC620* lr = nullptr;
        Device::MotorDjiC620* rr = nullptr;

        // 按下标访问, 方便 4 电机循环; 顺序 lf, rf, lr, rr
        Device::MotorDjiC620* operator[](uint32_t index) const
        {
            switch (index)
            {
                case 0: return lf;
                case 1: return rf;
                case 2: return lr;
                case 3: return rr;
                default: return nullptr;
            }
        }
    };

    // 构造顺序为 lf, rf, lr, rr; 机械参数在构造时固定
    explicit Leg(const std::array<Device::MotorDjiC620*, kMotorNum>& motors,
        const Mechanical& mechanical);
    explicit Leg(const MotorGroup& motor_group, const Mechanical& mechanical);

    Leg() = delete;
    Leg& operator=(const Leg&) = delete;

    void Init(float average_speed, const Homing& homing,
        Algorithm::Interpolator::Type interpolator_type = Algorithm::Interpolator::Type::kPolynomial,
        float cross_couple_kp = 40);

    // 目标高度: 前组/后组各自独立, 也可同时设置
    void SetTargetFront(float target) { target_.front = target; new_target_flag_ = true; }
    void SetTargetRear(float target) { target_.rear = target; new_target_flag_ = true; }
    void SetTarget(float front, float rear)
    {
        target_.front = front;
        target_.rear = rear;
        new_target_flag_ = true;
    }
    void SetTarget(const Height& target) { target_ = target; new_target_flag_ = true; }

    void SetInterpolatorType(Algorithm::Interpolator::Type type) { interpolator_type_ = type; }
    void SetAverageSpeed(float speed) { average_speed_ = speed; }
    void SetCrossCouple(bool enable) { cross_couple_enable_ = enable; }
    // 前/后两组刚度可分别整定
    void SetCrossCoupleKp(float kp)
    {
        cross_couple_front_.SetKp(kp);
        cross_couple_rear_.SetKp(kp);
    }
    void SetFrontCrossCoupleKp(float kp) { cross_couple_front_.SetKp(kp); }
    void SetRearCrossCoupleKp(float kp) { cross_couple_rear_.SetKp(kp); }
    void SetGravityComp(float current, uint32_t idx) { gravity_comp[idx] = current; }
    void SetGravityComp(const std::array<float, kMotorNum>& current);

    // 重力补偿读取
    float GetGravityComp(uint32_t idx) const { return gravity_comp[idx]; }
    const std::array<float, kMotorNum>& GetGravityComp() const { return gravity_comp; }

    Status GetStatus() const { return status_; }
    void SetHoming(const Homing& homing) { homing_ = homing; }
    const Homing& GetHoming() const { return homing_; }

    float GetTargetFront() const { return target_.front; }
    float GetTargetRear() const { return target_.rear; }
    const Height& GetTarget() const { return target_; }
    // 当前腿高 (前/后组平均高度)
    const Height& GetHeight() const { return now_height_; }
    const Mechanical& GetMechanical() const { return mechanical_; }
    bool GetCrossCoupeEnable() const { return cross_couple_enable_; }
    bool GetMotorAllEnabled() const;

    // 和 Chassis 相同的一拍入口
    void TimCalculatePeriodElapsedCallback();
    void Tim100msAlivePeriodElapsedCallback();

    // // 通信缓冲: 通信中间件直接读写
    // // 下行: 上位机下发的关节目标位置
    // LegTx tx_buffer_{};
    // // 上行: 回传给上位机的电机反馈
    // LegRx rx_buffer_{};

protected:
    //

    // 4 台电机统一由 MotorGroup 管理
    MotorGroup motor_group_;
    // 电机编码器偏移量, 下标 lf, rf, lr, rr
    float offset_[kMotorNum] = {};
    // 校准完成（下限位）时对应的高度值, 必须非正; 进入 kRunning 时零点按此平移
    float initial_height = 0.0f;
    // 电机电流重力补偿, 下标 lf, rf, lr, rr
    std::array<float, kMotorNum> gravity_comp = {};

    // 前组虚拟主轴插值规划器
    Algorithm::Interpolator front_master_axis_;
    // 后组虚拟主轴插值规划器
    Algorithm::Interpolator rear_master_axis_;
    // 插值规划器插值类型
    Algorithm::Interpolator::Type interpolator_type_ = Algorithm::Interpolator::Type::kPolynomial;

    // 交叉耦合: 两组互相独立, 各自只在组内求平均与补偿
    Algorithm::CrossCouple<kGroupMotorNum> cross_couple_front_;
    Algorithm::CrossCouple<kGroupMotorNum> cross_couple_rear_;

    // 运行时点到点的平均速度 (单位: m/s)
    float average_speed_ = 1.0f;

    // 机械参数
    Mechanical mechanical_;

    // 目标高度(前/后组独立) 与上一次目标高度
    Height target_, prev_target_;
    // 当前腿高(前/后组独立), 由组内电机位置换算并取平均得到
    Height now_height_;
    // debug: 为了调试方便一次性设置多个值
    bool new_target_flag_ = false;

    // 实际给 4 台电机的输出量
    struct
    {
        float speed_kff[kMotorNum] = {};
        float position_real[kMotorNum] = {};
    } output_;

    // 状态机
    Status status_ = Status::kInit;
    // 主动找限位参数
    Homing homing_ = {};
    // 各电机是否已完成找限位
    bool homing_done_[kMotorNum] = {};
    // 交叉耦合控制开关
    bool cross_couple_enable_ = false;

    void LimitOutput();

    void FiniteStateMachine();
    void EnterCalibrate();
    void EnterRunning();
    void EnterError();

    bool HomingStep();
    void RunningStep();
    void ErrorStep();
    void ResetMotion();

    void ParamValidation();
    void UpdateData();
    void NewTargetHandler();
    void CalculateOutput();
    void OutputToMotor();

    bool IsMasterAxisDone() const
    {
        return front_master_axis_.IsFinished() &&
            rear_master_axis_.IsFinished();
    }
};

class LegInteract : public Leg
{
public:
    //
    
    using Leg::Leg;

    LegInteract() = delete;
    LegInteract& operator=(const LegInteract&) = delete;

    void Init(float average_speed, const Homing& homing,
        Algorithm::Interpolator::Type interpolator_type = Algorithm::Interpolator::Type::kPolynomial,
        float cross_couple_kp = 0);

    // 上位机掉线保护: 失能后保持最后一帧目标, 不再接受新指令
    void SetState(LegStatus status) { enable_status_ = status; }

    void TimCalculatePeriodElapsedCallback();

protected:
    //
    

    // 通信缓冲: 通信中间件直接读写
    // 下行: 上位机下发的关节目标位置
    LegTx tx_buffer_{};
    // 上行: 回传给上位机的电机反馈
    LegRx rx_buffer_{};
    // 上位机链路使能状态
    LegStatus enable_status_ = LegStatus::kDisable;

    void UsbRegister();
    void UsbSetTarget();
    void UsbFeedback();

    
};

} // namespace Module


#endif
