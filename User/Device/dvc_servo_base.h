#ifndef DVC_SERVO_BASE_H
#define DVC_SERVO_BASE_H

#include "stdint.h"

/**
 * @brief 舵机通用接口
 */
class Class_Servo_Base
{
public:
    /**
     * @brief 舵机使能状态
     */
    enum class Enum_Servo_State
    {
        Servo_State_Disable = 0, // 禁止更新舵机位置
        Servo_State_Enable,      // 允许更新舵机位置
    };

    virtual ~Class_Servo_Base() {}

    virtual void Set_State(Enum_Servo_State state) = 0; // 设置软件使能状态
    virtual Enum_Servo_State Get_State() const = 0;     // 获取软件使能状态

    /**
     * @brief 设置归一化位置
     * @param position 范围 [0, 1]
     */
    virtual void Set_Normalized_Position(float position) = 0;
};

#endif // DVC_SERVO_BASE_H
