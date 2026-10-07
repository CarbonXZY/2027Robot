/**
 * @file drv_bsp.cpp
 * @brief  板级支持包: 电源轨使能、按键读取、加热/蜂鸣 PWM
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_bsp.h"

namespace Driver
{

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化全部板级支持包引脚
 *
 * @param status 各个状态的按位或, 具体如何使用参考宏定义
 * @param imu_heater_rate IMU 加热电阻占空比
 * @param buzzer_rate 蜂鸣器响度占空比
 */
void BspInit(uint32_t status, float imu_heater_rate, float buzzer_rate)
{
    BspSetDc24L((status & kDc24LOn) == 0 ? Dc24Status::kDisabled : Dc24Status::kEnabled);
    BspSetDc24R((status & kDc24ROn) == 0 ? Dc24Status::kDisabled : Dc24Status::kEnabled);
    BspSetDc5((status & kDc5On) == 0 ? Dc5Status::kDisabled : Dc5Status::kEnabled);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

    BspSetPwmImuHeater(imu_heater_rate);
    BspSetPwmBuzzer(buzzer_rate);
}

/**
 * @brief 获取左上角 DC24
 *
 * @return Dc24Status 状态
 */
Dc24Status BspGetDc24L()
{
    return static_cast<Dc24Status>(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));
}

/**
 * @brief 获取右上角 DC24
 *
 * @return Dc24Status 状态
 */
Dc24Status BspGetDc24R()
{
    return static_cast<Dc24Status>(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14));
}

/**
 * @brief 获取 DC5
 *
 * @return Dc5Status 状态
 */
Dc5Status BspGetDc5()
{
    return static_cast<Dc5Status>(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15));
}

/**
 * @brief 获取按键
 *
 * @return KeyStatus 状态
 */
KeyStatus BspGetKey()
{
    static GPIO_PinState pre_key_status;
    GPIO_PinState key_status;
    KeyStatus return_value;

    key_status = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

    switch (pre_key_status)
    {
    case GPIO_PIN_RESET:
    {
        switch (key_status)
        {
        case GPIO_PIN_RESET:
        {
            pre_key_status = key_status;
            return_value = KeyStatus::kFree;

            break;
        }
        case GPIO_PIN_SET:
        {
            pre_key_status = key_status;
            return_value = KeyStatus::kTrigFreePressed;

            break;
        }
        }

        break;
    }
    case GPIO_PIN_SET:
    {
        switch (key_status)
        {
        case GPIO_PIN_RESET:
        {
            pre_key_status = key_status;
            return_value = KeyStatus::kTrigPressedFree;

            break;
        }
        case GPIO_PIN_SET:
        {
            pre_key_status = key_status;
            return_value = KeyStatus::kPressed;

            break;
        }
        }

        break;
    }
    }

    return return_value;
}

/**
 * @brief 设定左上角 DC24
 *
 * @param status 状态
 */
void BspSetDc24L(Dc24Status status)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, static_cast<GPIO_PinState>(status));
}

/**
 * @brief 设定右上角 DC24
 *
 * @param status 状态
 */
void BspSetDc24R(Dc24Status status)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, static_cast<GPIO_PinState>(status));
}

/**
 * @brief 设定 DC5
 *
 * @param status 状态
 */
void BspSetDc5(Dc5Status status)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, static_cast<GPIO_PinState>(status));
}

/**
 * @brief 设定 IMU 加热电阻
 *
 * @param rate IMU 加热电阻温度占空比
 */
void BspSetPwmImuHeater(float rate)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, rate * 255);
}

/**
 * @brief 设定蜂鸣器
 *
 * @param rate 蜂鸣器响度占空比
 */
void BspSetPwmBuzzer(float rate)
{
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, rate * 1000);
}

} // namespace Driver

/************************ COPYRIGHT(C) NEUQ-RoboPioneers **************************/
