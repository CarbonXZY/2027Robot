/**
* @file mod_leg.cpp
* @author lyh
* @date 2026-10-19
* @brief 腿控制: 4 个齿轮齿条, 前后两组各自同步
*
*/
#include "mod_leg.h"

#include <cmath>

#include "Communication_Interface.hpp"
#include "drv_math.h"
#include "mid_config.h"

namespace
{

// 控制周期
constexpr float kControlPeriod = 0.001F;

// 平均速度下限 (EPSILON)
constexpr float kMinAverageSpeed = 0.01F;

// 运行阶段: 位置环 + 速度环
constexpr Device::MotorDjiC620::Parameters kRunningParams = {
    .pid_position = {
        .k_p = 20.0F, .k_i = 0.0F, .k_d = 0.0F
    },
    .pid_omega = {
        .k_p = 2.5F, .k_i = 0.0F, .k_d = 0.0F
    },
};

} // namespace

namespace Module
{

Leg::Leg(const std::array<Device::MotorDjiC620*, kMotorNum>& motors,
    const Mechanical& mechanical)
    : mechanical_(mechanical)
{
    motor_group_.lf = motors[0];
    motor_group_.rf = motors[1];
    motor_group_.lr = motors[2];
    motor_group_.rr = motors[3];
}

Leg::Leg(const MotorGroup& motor_group, const Mechanical& mechanical)
    : motor_group_(motor_group), mechanical_(mechanical)
{
}

/**
 * @brief 初始化
 * @param average_speed 点到点的平均速度 (m/s)
 * @param homing 主动找限位参数
 * @param interpolator_type 虚拟主轴插值类型
 * @param cross_couple_kp 交叉耦合刚度, 前后两组相同
 *
 */
void Leg::Init(float average_speed, const Homing& homing,
    Algorithm::Interpolator::Type interpolator_type,
    float cross_couple_kp)
{
    // 速度只接受正数, 非正数统一退回下限
    average_speed_ = average_speed > kMinAverageSpeed ? average_speed : kMinAverageSpeed;
    interpolator_type_ = interpolator_type;
    homing_ = homing;

    // 这里不碰电机, 等进入 kCalibrate 后再切换为速度环参数
    for (uint32_t i = 0; i < kMotorNum; i++)
    {
        homing_done_[i] = false;
    }
    ResetMotion();

    // 前后两组各自独立, 刚度分别设置
    cross_couple_front_.SetKp(cross_couple_kp);
    cross_couple_rear_.SetKp(cross_couple_kp);
    cross_couple_front_.Init(kGroupMotorNum);
    cross_couple_rear_.Init(kGroupMotorNum);

    // 注意 Register() 的 tx/rx 是本板视角: 本板发反馈, 收指令
    Middleware::g_usb_communication_interface.Register(
        static_cast<uint8_t>(Middleware::CommFrameId::kLeg),
        &rx_buffer_, &tx_buffer_,
        static_cast<uint8_t>(sizeof(rx_buffer_)), static_cast<uint8_t>(sizeof(tx_buffer_)));

    status_ = Status::kInit;
}

void Leg::TimCalculatePeriodElapsedCallback()
{
    ParamValidation();
    FiniteStateMachine();

    // kInit 阶段不驱动电机, 也不发送任何控制量
    if (status_ == Status::kInit) return;

    for (uint32_t i = 0; i < kMotorNum; i++)
    {
        motor_group_[i]->Calculate();
    }
}

// 按当前状态分发, 每拍只执行本状态的动作
void Leg::FiniteStateMachine()
{
    switch (status_)
    {
    case Status::kInit:
        if (GetMotorAllEnabled()) 
        {
            EnterCalibrate();
        }
        break;

    case Status::kCalibrate:
        if (HomingStep())
        {
            EnterRunning();
        }
        break;

    case Status::kRunning:
        RunningStep();
        break;

    case Status::kError:
        ErrorStep();
        break;
    }
}

// 进入找限位: 只用速度环
void Leg::EnterCalibrate()
{
    for (uint32_t i = 0; i < kMotorNum; i++)
    {
        homing_done_[i] = false;
    }
    ResetMotion();

    status_ = Status::kCalibrate;
}

// 找限位: 逐台低速向下, 堵转后记录零点并切回位置环保持
bool Leg::HomingStep()
{
    if (!GetMotorAllEnabled())
    {
        EnterError();
        return false;
    }

    bool all_done = true;

    for (uint32_t i = 0; i < kMotorNum; i++)
    {
        // 已完成: 位置保持, 不再参与找限位
        if (homing_done_[i])
        {
            motor_group_[i]->SetFeedforwardOmega(0.0f);
            motor_group_[i]->SetFeedforwardCurrent(0.0f);
            motor_group_[i]->SetTargetPosition(offset_[i]);
            continue;
        }

        all_done = false;

        homing_.speed = std::abs(homing_.speed);
        // 高度速度换算成电机角速度, 方向与 direction_sign 一致
        Device::CalibrateParams params = {
            .motion_mode = Device::CalibrateMotionMode::kSpeed,
            .motion_value = -homing_.speed * static_cast<float>(mechanical_.direction_sign[i]) /
                mechanical_.equivalent_radius[i],
            // 速度判据: 实测转速持续低于阈值判定到底
            .detect_mode = Device::CalibrateDetectMode::kSpeed,
            // 高度阈值 (m/s) 换算成输出轴角速度 (rad/s)
            .detect_threshold = homing_.speed_threshold / mechanical_.equivalent_radius[i],
            // ms -> us
            .debounce_us = homing_.debounce_ms * 1000U,
        };

        // 找限位期间清掉前馈, 避免影响堵转判据
        motor_group_[i]->SetFeedforwardOmega(0.0f);
        motor_group_[i]->SetFeedforwardCurrent(0.0f);

        float offset = 0.0f;
        if (motor_group_[i]->Calibrate(params, offset))
        {
            offset_[i] = offset;
            homing_done_[i] = true;

            // 立即退出速度模式, 在等待其他腿时保持当前位置
            motor_group_[i]->Init(Device::MotorControlMethod::kPosition, kRunningParams);
            motor_group_[i]->SetTargetPosition(offset_[i]);
        }
    }

    return all_done;
}

// 进入运行: 零点已由下限位确定并平移到 initial_height, 两组目标都从 0 开始
void Leg::EnterRunning()
{

    // 切回运行时位置环, 同时清掉速度环积分与前馈
    for (uint32_t i = 0; i < kMotorNum; i++)
    {
        motor_group_[i]->Init(Device::MotorControlMethod::kPosition, kRunningParams);
    }

    // 校准位置对应高度 initial_height: 平移零点, 使下限位读数等于 initial_height。
    // 校准期间的保持指令用的仍是原始堵转角度, 因此不会被提前抬起。
    for (uint32_t i = 0; i < kMotorNum; i++)
    {
        offset_[i] -= initial_height * static_cast<float>(mechanical_.direction_sign[i]) /
            mechanical_.equivalent_radius[i];
    }

    // 重置运动状态 (目标 0, 即比校准点高 -initial_height) 并刷新当前高度
    ResetMotion();
    UpdateData();

    status_ = Status::kRunning;
}

// 正常运行: 更新反馈 -> 规划 -> 计算输出
void Leg::RunningStep()
{
    if (!GetMotorAllEnabled())
    {
        EnterError();
        return;
    }

    // 下行: 只有使能时才接受上位机目标, 失能时保持最后一帧目标
    if (enable_status_ == LegStatus::kEnable)
    {
        // 上位机直接下发腿高 (m), 向上为正; 组内两条腿取平均作为本组目标
        SetTarget(0.5f * (tx_buffer_.position[0] + tx_buffer_.position[1]),
                  0.5f * (tx_buffer_.position[2] + tx_buffer_.position[3]));
    }

    UpdateData();
    NewTargetHandler();
    CalculateOutput();
    LimitOutput();
    OutputToMotor();

    // 上行: 回传腿高 (m) 与线速度 (m/s), 向上为正, 由通信中间件打包
    for (uint32_t i = 0; i < kMotorNum; i++)
    {
        const float direction = static_cast<float>(mechanical_.direction_sign[i]);
        const float radius   = mechanical_.equivalent_radius[i];
        rx_buffer_.motor[i].position =
            direction * (motor_group_[i]->GetNowAngle() - offset_[i]) * radius;
        rx_buffer_.motor[i].velocity = direction * motor_group_[i]->GetNowOmega() * radius;
    }
}

// 进入故障状态: 先退出速度模式并以当前位置保持, 再锁存
void Leg::EnterError()
{
    if (status_ == Status::kError) return;

    for (uint32_t i = 0; i < kMotorNum; i++)
    {
        motor_group_[i]->Init(Device::MotorControlMethod::kPosition, kRunningParams);
        motor_group_[i]->SetTargetPosition(motor_group_[i]->GetNowAngle());
        motor_group_[i]->SetFeedforwardOmega(0.0f);
        motor_group_[i]->SetFeedforwardCurrent(gravity_comp[i]);
    }

    status_ = Status::kError;
}

// 故障锁存: 停止更新目标, 只清速度前馈
void Leg::ErrorStep()
{
    for (uint32_t i = 0; i < kMotorNum; i++)
    {
        motor_group_[i]->SetFeedforwardOmega(0.0f);
        motor_group_[i]->SetFeedforwardCurrent(gravity_comp[i]);
    }
}

// 4 台电机是否全部在线
bool Leg::GetMotorAllEnabled() const
{
    for (uint32_t i = 0; i < kMotorNum; i++)
    {
        if (motor_group_[i]->GetStatus() != Device::MotorStatus::kEnable)
        {
            return false;
        }
    }

    return true;
}

// 清空运动状态: 目标清零, 插值器回到 0, 输出清零
void Leg::ResetMotion()
{
    target_ = {};
    // 哨兵值保证任何合法目标(>=0)都会触发重新规划
    prev_target_.front = -1.0f;
    prev_target_.rear = -1.0f;
    new_target_flag_ = true;

    front_master_axis_.Reset(0.0f, 0.0f, 1, interpolator_type_);
    rear_master_axis_.Reset(0.0f, 0.0f, 1, interpolator_type_);

    output_ = {};
}

void Leg::UpdateData()
{
    for (uint32_t i = 0; i < kMotorNum; i++)
    {
        // 电机转角换算成齿条直线高度 (m)
        const float height = (motor_group_[i]->GetNowAngle() - offset_[i]) *
            mechanical_.direction_sign[i] * mechanical_.equivalent_radius[i];

        // 前组 0/1, 后组重新编号为 0/1
        if (i < kGroupMotorNum)
        {
            cross_couple_front_.SetNow(height, i);
        }
        else
        {
            cross_couple_rear_.SetNow(height, i - kGroupMotorNum);
        }
    }

    // 获取各组当前平均高度
    cross_couple_front_.CalculateNowAverage();
    cross_couple_rear_.CalculateNowAverage();
    now_height_.front = cross_couple_front_.GetNowAverage();
    now_height_.rear = cross_couple_rear_.GetNowAverage();
}

void Leg::NewTargetHandler()
{
    // 没有新目标, 本周期不规划
    if (!new_target_flag_) return;
    // 标志已消费, 避免同一目标重复规划
    new_target_flag_ = false;

    if (average_speed_ < kMinAverageSpeed) average_speed_ = kMinAverageSpeed;

    // 前组独立规划
    if (target_.front != prev_target_.front)
    {
        // 总步数 = 行程 / 每 tick 的行程
        uint32_t total_step = std::ceil(std::abs(now_height_.front - target_.front) /
            (average_speed_ * kControlPeriod));
        front_master_axis_.Reset(now_height_.front, target_.front, total_step, interpolator_type_);

        prev_target_.front = target_.front;
    }

    // 后组独立规划
    if (target_.rear != prev_target_.rear)
    {
        uint32_t total_step = std::ceil(std::abs(now_height_.rear - target_.rear) /
            (average_speed_ * kControlPeriod));
        rear_master_axis_.Reset(now_height_.rear, target_.rear, total_step, interpolator_type_);

        prev_target_.rear = target_.rear;
    }
}

void Leg::CalculateOutput()
{
    // 两组虚拟主轴各自更新
    front_master_axis_.Update();
    rear_master_axis_.Update();

    const float front_pos = front_master_axis_.GetNow();
    // 相邻两次输出之差 / 控制周期 即本组高度速度 (单位: m/s)
    const float front_speed = (front_pos - front_master_axis_.GetPrev()) / kControlPeriod;

    const float rear_pos = rear_master_axis_.GetNow();
    const float rear_speed = (rear_pos - rear_master_axis_.GetPrev()) / kControlPeriod;

    for (uint32_t i = 0; i < kGroupMotorNum; i++)
    {
        // 高度 -> 电机转角, direction_sign 的逆等于自身
        const float ratio = static_cast<float>(mechanical_.direction_sign[i]) /
            mechanical_.equivalent_radius[i];
        output_.position_real[i] = front_pos * ratio + offset_[i];
        output_.speed_kff[i] = front_speed * ratio;
    }

    for (uint32_t i = kGroupMotorNum; i < kMotorNum; i++)
    {
        const float ratio = static_cast<float>(mechanical_.direction_sign[i]) /
            mechanical_.equivalent_radius[i];
        output_.position_real[i] = rear_pos * ratio + offset_[i];
        output_.speed_kff[i] = rear_speed * ratio;
    }

    // 交叉耦合补偿, 两组各算各的
    if (cross_couple_enable_)
    {
        cross_couple_front_.Update();
        cross_couple_rear_.Update();

        for (uint32_t i = 0; i < kGroupMotorNum; i++)
        {
            const float ratio = static_cast<float>(mechanical_.direction_sign[i]) /
                mechanical_.equivalent_radius[i];
            output_.speed_kff[i] += cross_couple_front_.GetOutput()[i] * ratio;
        }

        for (uint32_t i = kGroupMotorNum; i < kMotorNum; i++)
        {
            const float ratio = static_cast<float>(mechanical_.direction_sign[i]) /
                mechanical_.equivalent_radius[i];
            output_.speed_kff[i] +=
                cross_couple_rear_.GetOutput()[i - kGroupMotorNum] * ratio;
        }
    }
}

// 参数合法化
void Leg::ParamValidation()
{
    using namespace Driver;

    // MathConstrain 不处理 NaN, 先挡掉非有限值
    if (!std::isfinite(target_.front)) target_.front = 0.0f;
    if (!std::isfinite(target_.rear)) target_.rear = 0.0f;
    MathConstrain(&target_.front, 0.0f, mechanical_.max_height.front);
    MathConstrain(&target_.rear, 0.0f, mechanical_.max_height.rear);

    average_speed_ = std::abs(average_speed_);
    if (!std::isfinite(average_speed_) || average_speed_ < kMinAverageSpeed)
    {
        average_speed_ = kMinAverageSpeed;
    }

    // 机械参数非法时锁存故障, 避免除零和符号错误
    for (uint32_t i = 0; i < kMotorNum; i++)
    {
        if (!std::isfinite(mechanical_.equivalent_radius[i]) ||
            mechanical_.equivalent_radius[i] <= 0.0f ||
            (mechanical_.direction_sign[i] != 1 && mechanical_.direction_sign[i] != -1) ||
            (mechanical_.max_height.front <= 0 || mechanical_.max_height.rear <= 0))
        {
            EnterError();
        }
    }

    // initial_height 必须非正: 正值会让进入 kRunning 后的目标 0 位于下限位以下
    if (!std::isfinite(initial_height) || initial_height > 0.0f)
    {
        EnterError();
    }

    // 找限位参数非法时锁存故障; 阈值必须小于下降速度, 否则正常下降也会被判成堵转
    if (!std::isfinite(homing_.speed) || homing_.speed <= 0.0f ||
        !std::isfinite(homing_.speed_threshold) || homing_.speed_threshold <= 0.0f ||
        homing_.speed_threshold >= homing_.speed)
    {
        EnterError();
    }
}

void Leg::OutputToMotor()
{
    for (uint32_t i = 0; i < kMotorNum; i++)
    {
        // 设置重力补偿
        motor_group_[i]->SetFeedforwardCurrent(gravity_comp[i]);

        // 设置速度前馈
        motor_group_[i]->SetFeedforwardOmega(output_.speed_kff[i]);

        // 设置位置目标
        motor_group_[i]->SetTargetPosition(output_.position_real[i]);
    }
}

// 限制实际速度: 位置环输出与速度前馈各自不超过 2 倍平均速度
void Leg::LimitOutput()
{
    using namespace Driver;

    // 实际高度速度上限 (m/s)
    const float max_height_speed = 2.0f * std::abs(average_speed_);

    for (uint32_t i = 0; i < kMotorNum; i++)
    {
        // 高度速度 -> 输出轴角速度 (rad/s), 上限与方向无关
        const float max_omega = max_height_speed / mechanical_.equivalent_radius[i];

        // 速度前馈 (含交叉耦合补偿) 限幅
        MathConstrain(&output_.speed_kff[i], -max_omega, max_omega);

        // 位置环输出即速度目标, 同样限幅; 这里恒为正值
        motor_group_[i]->pid_position.SetOutMax(max_omega);
    }
}

void Leg::Tim100msAlivePeriodElapsedCallback()
{
    for (uint32_t i = 0; i < kMotorNum; i++)
    {
        motor_group_[i]->Tim100msAlivePeriodElapsedCallback();
    }
}

void Leg::SetGravityComp(const std::array<float, kMotorNum>& current)
{
    for (uint32_t i = 0; i < kMotorNum; i++)
    {
        gravity_comp[i] = current[i];
    }

}

} // namespace Module
