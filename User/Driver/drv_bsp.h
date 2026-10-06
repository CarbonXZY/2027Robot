/**
 * @file drv_bsp.h
 * @brief  板级支持包: 电源轨使能、按键读取、加热/蜂鸣 PWM
 */

#ifndef DRV_BSP_H
#define DRV_BSP_H

/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx_hal.h"

/* CubeMX generated handles (global namespace) -------------------------------*/

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim12;

namespace Driver
{

/* Exported macros -----------------------------------------------------------*/

// BspInit 的按位或参数
constexpr uint32_t kDc24LOn = (1 << 0);
constexpr uint32_t kDc24ROn = (1 << 1);
constexpr uint32_t kDc5On = (1 << 2);
constexpr uint32_t kDc24LOff = (0 << 0);
constexpr uint32_t kDc24ROff = (0 << 1);
constexpr uint32_t kDc5Off = (0 << 2);

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 板上 DC24 工作状态
 *
 */
enum class Dc24Status
{
    kDisabled = 0,
    kEnabled,
};

/**
 * @brief 板上 DC5 工作状态
 *
 */
enum class Dc5Status
{
    kDisabled = 0,
    kEnabled,
};

/**
 * @brief 按键状态
 *
 */
enum class KeyStatus
{
    kFree = 0,
    kTrigFreePressed,
    kTrigPressedFree,
    kPressed,
};

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 初始化全部板级支持包引脚
 *
 * @param status 各个状态的按位或, 具体如何使用参考宏定义
 * @param imu_heater_rate IMU 加热电阻占空比
 * @param buzzer_rate 蜂鸣器响度占空比
 */
void BspInit(uint32_t status, float imu_heater_rate = 0, float buzzer_rate = 0);

Dc24Status BspGetDc24L();

Dc24Status BspGetDc24R();

Dc5Status BspGetDc5();

KeyStatus BspGetKey();

void BspSetDc24L(Dc24Status status);

void BspSetDc24R(Dc24Status status);

void BspSetDc5(Dc5Status status);

void BspSetPwmImuHeater(float rate);

void BspSetPwmBuzzer(float rate);

} // namespace Driver

#endif

/************************ COPYRIGHT(C) NEUQ-RoboPioneers **************************/
