/**
 * @file drv_math.h
 * @author yssickjgd 1345578933@qq.com
 * @brief 一些数学
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef DRV_MATH_H
#define DRV_MATH_H

/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx_hal.h"
#include "arm_math.h"
#include <cfloat>

namespace Driver
{

/* Exported macros -----------------------------------------------------------*/

// rpm换算到rad/s
constexpr float kRpmToRadps = (2.0f * PI / 60.0f);
// deg换算到rad
constexpr float kDegToRad = (PI / 180.0f);
// 摄氏度换算到开氏度
constexpr float kCelsiusToKelvin = (273.15f);

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

void MathBooleanLogicalNot(bool *value);

void MathEndianReverse16(void *address);

uint16_t MathEndianReverse16(void *source, void *destination);

void MathEndianReverse32(void *address);

uint32_t MathEndianReverse32(void *source, void *destination);

uint8_t MathSum8(uint8_t *address, uint32_t length);

uint16_t MathSum16(uint16_t *address, uint32_t length);

uint32_t MathSum32(uint32_t *address, uint32_t length);

float MathSinc(float x);

int32_t MathFloatToInt(float x, float float_1, float float_2, int32_t int_1, int32_t int_2);

float MathIntToFloat(int32_t x, int32_t int_1, int32_t int_2, float float_1, float float_2);

/**
 * @brief 限幅函数
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @param min 最小值
 * @param max 最大值
 */
template<typename Type>
Type MathConstrain(Type *x, Type min, Type max)
{
    if (*x < min)
    {
        *x = min;
    }
    else if (*x > max)
    {
        *x = max;
    }
    return (*x);
}

/**
 * @brief 求绝对值
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @return Type x的绝对值
 */
template<typename Type>
Type MathAbs(Type x)
{
    return ((x > 0) ? x : -x);
}

/**
 * @brief 求取模归化
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @param modulus 模数
 * @return Type 返回的归化数, 介于 ±modulus / 2 之间
 */
template<typename Type>
Type MathModulusNormalization(Type x, Type modulus)
{
    float tmp;

    tmp = fmod(x + modulus / 2.0f, modulus);

    if (tmp < 0.0f)
    {
        tmp += modulus;
    }

    return (tmp - modulus / 2.0f);
}

} // namespace Driver

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
