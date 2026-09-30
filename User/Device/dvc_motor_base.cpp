#include "dvc_motor_base.h"
#include "drv_math.h"

namespace Device
{

bool MotorBase::Calibrate(const CalibrateParams &params, float &offset)
{
    UpdateFeedback();

    uint32_t now = DwtGetCurrentTimeUs();

    // 设置运动模式
    switch (params.motion_mode)
    {
        case CalibrateMotionMode::kNone:
            offset = GetPosition();
            stall_debounce_start_time_ = 0;
            return true;

        case CalibrateMotionMode::kSpeed:
            SetControlMethod(MotorControlMethod::kSpeed);
            SetTargetSpeed(params.motion_value);
            break;

        case CalibrateMotionMode::kCurrent:
            SetControlMethod(MotorControlMethod::kCurrent);
            SetTargetCurrent(params.motion_value);
            break;
    }

    // 堵转条件检测
    bool condition_met = false;
    switch (params.detect_mode)
    {
        case CalibrateDetectMode::kCurrent:
            condition_met = Driver::MathAbs(GetCurrent()) >= params.detect_threshold;
            break;
        case CalibrateDetectMode::kSpeed:
            condition_met = Driver::MathAbs(GetSpeed()) <= params.detect_threshold;
            break;
    }

    // 消抖: 条件需持续满足 debounce_us 时长
    if (condition_met)
    {
        if (stall_debounce_start_time_ == 0)
            stall_debounce_start_time_ = now;

        if (now - stall_debounce_start_time_ >= params.debounce_us)
        {
            offset = GetPosition();
            stall_debounce_start_time_ = 0;
            return true;
        }
    }
    else
    {
        stall_debounce_start_time_ = 0;
    }

    return false;
}

} // namespace Device
