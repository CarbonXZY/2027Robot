/**
 * @file drv_math.cpp
 * @author yssickjgd 1345578933@qq.com
 * @brief 一些数学
 * @version 1.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-11-10 1.1 修改成cpp
 *
 * @copyright Copyright (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

namespace Driver
{

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 布尔值反转
 *
 * @param value 布尔值地址
 */
void MathBooleanLogicalNot(bool *value)
{
    if (*value == false)
    {
        *value = true;
    }
    else if (*value == true)
    {
        *value = false;
    }
}

/**
 * @brief 16位大小端转换
 *
 * @param address 地址
 */
void MathEndianReverse16(void *address)
{
    uint8_t *temp_address_8 = (uint8_t *) address;
    uint16_t *temp_address_16 = (uint16_t *) address;
    *temp_address_16 = temp_address_8[0] << 8 | temp_address_8[1];
}

/**
 * @brief 16位大小端转换
 *
 * @param source 源数据地址
 * @param destination 目标存储地址
 * @return uint16_t 结果
 */
uint16_t MathEndianReverse16(void *source, void *destination)
{
    uint8_t *temp_address_8 = (uint8_t *) source;
    uint16_t temp_address_16;
    temp_address_16 = temp_address_8[0] << 8 | temp_address_8[1];

    if (destination != nullptr)
    {
        uint8_t *temp_source, *temp_destination;
        temp_source = (uint8_t *) source;
        temp_destination = (uint8_t *) destination;

        temp_destination[0] = temp_source[1];
        temp_destination[1] = temp_source[0];
    }

    return temp_address_16;
}

/**
 * @brief 32位大小端转换
 *
 * @param address 地址
 */
void MathEndianReverse32(void *address)
{
    uint8_t *temp_address_8 = (uint8_t *) address;
    uint32_t *temp_address_32 = (uint32_t *) address;
    *temp_address_32 = temp_address_8[0] << 24 | temp_address_8[1] << 16 | temp_address_8[2] << 8 | temp_address_8[3];
}

/**
 * @brief 32位大小端转换
 *
 * @param source 源数据地址
 * @param destination 目标存储地址
 * @return uint32_t 结果
 */
uint32_t MathEndianReverse32(void *source, void *destination)
{
    uint8_t *temp_address_8 = (uint8_t *) source;
    uint32_t temp_address_32;
    temp_address_32 = temp_address_8[0] << 24 | temp_address_8[1] << 16 | temp_address_8[2] << 8 | temp_address_8[3];

    if (destination != nullptr)
    {
        uint8_t *temp_source, *temp_destination;
        temp_source = (uint8_t *) source;
        temp_destination = (uint8_t *) destination;

        temp_destination[0] = temp_source[3];
        temp_destination[1] = temp_source[2];
        temp_destination[2] = temp_source[1];
        temp_destination[3] = temp_source[0];
    }

    return temp_address_32;
}

/**
 * @brief 求和
 *
 * @param address 起始地址
 * @param length 被加的数据的数量, 注意不是字节数
 * @return uint8_t 结果
 */
uint8_t MathSum8(uint8_t *address, uint32_t length)
{
    uint8_t sum = 0;
    for (int i = 0; i < length; i++)
    {
        sum += address[i];
    }
    return (sum);
}

/**
 * @brief 求和
 *
 * @param address 起始地址
 * @param length 被加的数据的数量, 注意不是字节数
 * @return uint16_t 结果
 */
uint16_t MathSum16(uint16_t *address, uint32_t length)
{
    uint16_t sum = 0;
    for (int i = 0; i < length; i++)
    {
        sum += address[i];
    }
    return (sum);
}

/**
 * @brief 求和
 *
 * @param address 起始地址
 * @param length 被加的数据的数量, 注意不是字节数
 * @return uint32_t 结果
 */
uint32_t MathSum32(uint32_t *address, uint32_t length)
{
    uint32_t sum = 0;
    for (int i = 0; i < length; i++)
    {
        sum += address[i];
    }
    return (sum);
}

/**
 * @brief sinc函数的实现
 *
 * @param x 输入
 * @return float 输出
 */
float MathSinc(float x)
{
    // 分母为0则按极限求法
    if (MathAbs(x) <= 2.0f * FLT_EPSILON)
    {
        return (1.0f);
    }

    return (arm_sin_f32(x) / x);
}

/**
 * @brief 将浮点数映射到整型
 *
 * @param x 浮点数
 * @param float_1 浮点数1
 * @param float_2 浮点数2
 * @param int_1 整型1
 * @param int_2 整型2
 * @return int32_t 整型
 */
int32_t MathFloatToInt(float x, float float_1, float float_2, int32_t int_1, int32_t int_2)
{
    float tmp = (x - float_1) / (float_2 - float_1);
    int32_t out = tmp * (float) (int_2 - int_1) + int_1;
    return (out);
}

/**
 * @brief 将整型映射到浮点数
 *
 * @param x 整型
 * @param int_1 整型1
 * @param int_2 整型2
 * @param float_1 浮点数1
 * @param float_2 浮点数2
 * @return float 浮点数
 */
float MathIntToFloat(int32_t x, int32_t int_1, int32_t int_2, float float_1, float float_2)
{
    float tmp = (float) (x - int_1) / (float) (int_2 - int_1);
    float out = tmp * (float_2 - float_1) + float_1;
    return (out);
}

} // namespace Driver

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
