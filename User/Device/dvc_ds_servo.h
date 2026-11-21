/**
 * @file dvc_ds_servo.h
 * @brief PWM 数字舵机设备接口
 */

#ifndef DVC_DS_SERVO_H
#define DVC_DS_SERVO_H

#include "main.h"
#include "dvc_servo_base.h"
#include "drv_tim.h"

/**
 * @brief 使用定时器 PWM 通道驱动的数字舵机
 */
class Class_DS_Servo : public Class_Servo_Base
{
public:
    /**
     * @brief 初始化 PWM 通道和有效脉宽范围
     * @param htim PWM 定时器
     * @param Channel PWM 通道
     * @param min_pulse 最小位置对应脉宽，单位 us
     * @param max_pulse 最大位置对应脉宽，单位 us
     */
    void Init(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t min_pulse, uint16_t max_pulse);


    /**
     * @brief 设置舵机软件使能状态
     */
    inline void Set_State(
        Class_Servo_Base::Enum_Servo_State state) override;

    /**
     * @brief 获取舵机软件使能状态
     */
    inline Class_Servo_Base::Enum_Servo_State Get_State() const override;

    /**
     * @brief 设置归一化位置，0 对应 min_pulse，1 对应 max_pulse
     */
    void Set_Normalized_Position(float position) override;

private:
    uint16_t Min_Pulse = 0; // 归一化位置 0 对应脉宽 (us)
    uint16_t Max_Pulse = 0; // 归一化位置 1 对应脉宽 (us)

    TIM_HandleTypeDef *htim = nullptr; // PWM 定时器句柄
    uint32_t Channel = 0;              // PWM 通道

    Class_Servo_Base::Enum_Servo_State State =
        Class_Servo_Base::Enum_Servo_State::Servo_State_Disable; // 软件使能状态
};

inline void Class_DS_Servo::Set_State(Class_Servo_Base::Enum_Servo_State state) 
{
    State = state;
}

inline Class_Servo_Base::Enum_Servo_State Class_DS_Servo::Get_State() const 
{
    return State;
}


#endif
