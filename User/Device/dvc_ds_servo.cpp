#include "dvc_ds_servo.h"

/**
 * @brief 保存 PWM 配置并设置初始使能状态
 */
void Class_DS_Servo::Init(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t min_pulse, uint16_t max_pulse)
{
    this->htim = htim;
    this->Channel = Channel;

    Min_Pulse = min_pulse;
    Max_Pulse = max_pulse;

    State =
        (htim != nullptr && min_pulse < max_pulse)
            ? Class_Servo_Base::Enum_Servo_State::Servo_State_Enable
            : Class_Servo_Base::Enum_Servo_State::Servo_State_Disable;
}

/**
 * @brief 将归一化位置换算成脉宽并写入 PWM 比较寄存器
 */
void Class_DS_Servo::Set_Normalized_Position(float position)
{
    if (State != Class_Servo_Base::Enum_Servo_State::Servo_State_Enable ||
        htim == nullptr)
    {
        return;
    }

    if (position < 0.0f)
    {
        position = 0.0f;
    }
    else if (position > 1.0f)
    {
        position = 1.0f;
    }

    uint16_t pulse = Min_Pulse + static_cast<uint16_t>(position * (Max_Pulse - Min_Pulse));
    __HAL_TIM_SET_COMPARE(htim, Channel, pulse);
}
