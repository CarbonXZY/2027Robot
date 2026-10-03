/**
 * @file    mod_gripper.cpp
 * @author  lyh
 * @date    2026-10-01
 * @brief
 */
#include "mod_gripper.h"
#include "drv_math.h"

using namespace Device;

namespace Module
{

void Gripper::Init()
{
    motor_->Init(MotorControlMethod::kSpeed,
{.pid_omega = {.k_p = 2, .k_i = 0, .k_d = 0}});
    mit_task_space_.Init(mit_param_.kp, mit_param_.kd);
    status_ = Status::kCalibrate;
}

void Gripper::UpdateSpaceParam()
{
    // 获取关节空间参数
    joint_space_.angle = (motor_->GetNowAngle() - encoder_offset_) * mechanical_.DIRECTION_SIGN;
    joint_space_.speed = motor_->GetNowOmega() * mechanical_.DIRECTION_SIGN;

    // 转化为任务空间参数
    task_space_.distance = joint_space_.angle * mechanical_.PITCH_RADIUS;
    task_space_.speed = joint_space_.speed * mechanical_.PITCH_RADIUS;
}

void Gripper::CalculateCurrent()
{
    // 同步参数
    mit_task_space_.Init(mit_param_.kp, mit_param_.kd);

    // 计算弹簧阻尼系统力矩
    mit_task_space_.SetXdes(target_task_space_.distance);
    mit_task_space_.SetX(task_space_.distance);
    mit_task_space_.SetVdes(target_task_space_.speed);
    mit_task_space_.SetV(task_space_.speed);
    mit_task_space_.Calculate();

    // 把虚拟力映射到实际电流
    target_current_ = mit_task_space_.GetOutput() * mechanical_.DIRECTION_SIGN
        + GetFrictionCompensation();
}

float Gripper::GetFrictionCompensation() const
{
    // 得到实际补偿值
    return motor_->GetNowOmega() > 0 ?
        friction_compensation_.forward : friction_compensation_.backward;
}

void Gripper::Running()
{
    UpdateSpaceParam();
    HandlerEvent();
    CalculateCurrent();
    OutputToMotor();
}

void Gripper::TimCalculate1msCallback()
{
    Fsm();

    motor_->Calculate();
}

void Gripper::TimAlive100msCallback()
{
    motor_->Tim100msAlivePeriodElapsedCallback();
}

void Gripper::Fsm()
{
    switch (status_)
    {
    case Status::kInit:
        {
            Init();
            status_ = Status::kCalibrate;
        }
        break;

    case Status::kCalibrate:
        {
            float offset = 0;
            if (motor_->Calibrate(calibrate_params_, offset))
            {
                encoder_offset_ = offset;
                status_ = Status::kRunning;
                motor_->SetTargetSpeed(0);
                motor_->SetControlMethod(MotorControlMethod::kCurrent);
                UpdateSpaceParam();
            }
        }
        break;

    case Status::kRunning:
        {
            Running();
        }
        break;

    default:
        break;
    }
}

void Gripper::OutputToMotor()
{
    Driver::MathConstrain(&target_current_, -max_current_, max_current_);
    motor_->SetTargetCurrent(target_current_);
}

void Gripper::HandlerEvent()
{
    switch (event_)
    {
    case Event::kNone:
        return;

    case Event::kClamp:
        target_task_space_.distance = mechanical_.CLAMP_DISTANCE;
        break;

    case Event::kRelease:
        target_task_space_.distance = 0;
        break;
    }

    event_ = Event::kNone;
}

//
} // namespace Module
