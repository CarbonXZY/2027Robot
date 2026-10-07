/**
 * @file drv_uart.cpp
 * @author Lucy (2478427315@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-10-04
 *
 * @copyright RoboPioneer (c) 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_uart.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

namespace Driver
{

UartManageObject g_uart1_manage_object = {0};
UartManageObject g_uart2_manage_object = {0};
UartManageObject g_uart3_manage_object = {0};
UartManageObject g_uart4_manage_object = {0};
UartManageObject g_uart5_manage_object = {0};
UartManageObject g_uart6_manage_object = {0};
UartManageObject g_uart7_manage_object = {0};
UartManageObject g_uart8_manage_object = {0};
UartManageObject g_uart9_manage_object = {0};
UartManageObject g_uart10_manage_object = {0};

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化UART
 *
 * @param huart UART编号
 * @param callback_function 处理回调函数
 * @param rx_buffer_length 接收缓冲区长度
 */
void UartInit(UART_HandleTypeDef *huart, UartCallback callback_function, uint16_t rx_buffer_length)
{
    if (huart->Instance == USART1)
    {
        g_uart1_manage_object.uart_handler_ = huart;
        g_uart1_manage_object.callback_function_ = callback_function;
        g_uart1_manage_object.rx_buffer_length_ = rx_buffer_length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart1_manage_object.rx_buffer_, g_uart1_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == USART2)
    {
        g_uart2_manage_object.uart_handler_ = huart;
        g_uart2_manage_object.callback_function_ = callback_function;
        g_uart2_manage_object.rx_buffer_length_ = rx_buffer_length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart2_manage_object.rx_buffer_, g_uart2_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == USART3)
    {
        g_uart3_manage_object.uart_handler_ = huart;
        g_uart3_manage_object.callback_function_ = callback_function;
        g_uart3_manage_object.rx_buffer_length_ = rx_buffer_length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart3_manage_object.rx_buffer_, g_uart3_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == UART4)
    {
        g_uart4_manage_object.uart_handler_ = huart;
        g_uart4_manage_object.callback_function_ = callback_function;
        g_uart4_manage_object.rx_buffer_length_ = rx_buffer_length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart4_manage_object.rx_buffer_, g_uart4_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == UART5)
    {
        g_uart5_manage_object.uart_handler_ = huart;
        g_uart5_manage_object.callback_function_ = callback_function;
        g_uart5_manage_object.rx_buffer_length_ = rx_buffer_length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart5_manage_object.rx_buffer_, g_uart5_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == USART6)
    {
        g_uart6_manage_object.uart_handler_ = huart;
        g_uart6_manage_object.callback_function_ = callback_function;
        g_uart6_manage_object.rx_buffer_length_ = rx_buffer_length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart6_manage_object.rx_buffer_, g_uart6_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == UART7)
    {
        g_uart7_manage_object.uart_handler_ = huart;
        g_uart7_manage_object.callback_function_ = callback_function;
        g_uart7_manage_object.rx_buffer_length_ = rx_buffer_length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart7_manage_object.rx_buffer_, g_uart7_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == UART8)
    {
        g_uart8_manage_object.uart_handler_ = huart;
        g_uart8_manage_object.callback_function_ = callback_function;
        g_uart8_manage_object.rx_buffer_length_ = rx_buffer_length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart8_manage_object.rx_buffer_, g_uart8_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == UART9)
    {
        g_uart9_manage_object.uart_handler_ = huart;
        g_uart9_manage_object.callback_function_ = callback_function;
        g_uart9_manage_object.rx_buffer_length_ = rx_buffer_length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart9_manage_object.rx_buffer_, g_uart9_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == USART10)
    {
        g_uart10_manage_object.uart_handler_ = huart;
        g_uart10_manage_object.callback_function_ = callback_function;
        g_uart10_manage_object.rx_buffer_length_ = rx_buffer_length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart10_manage_object.rx_buffer_, g_uart10_manage_object.rx_buffer_length_);
    }
}

/**
 * @brief 重新初始化UART结构体
 *
 * @param huart UART结构体
 */
void UartReinit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart1_manage_object.rx_buffer_, g_uart1_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == USART2)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart2_manage_object.rx_buffer_, g_uart2_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == USART3)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart3_manage_object.rx_buffer_, g_uart3_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == UART4)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart4_manage_object.rx_buffer_, g_uart4_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart5_manage_object.rx_buffer_, g_uart5_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == USART6)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart6_manage_object.rx_buffer_, g_uart6_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == UART7)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart7_manage_object.rx_buffer_, g_uart7_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == UART8)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart8_manage_object.rx_buffer_, g_uart8_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == UART9)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart9_manage_object.rx_buffer_, g_uart9_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == USART10)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart10_manage_object.rx_buffer_, g_uart10_manage_object.rx_buffer_length_);
    }
}

/**
 * @brief 发送数据帧
 *
 * @param huart UART编号
 * @param data 被发送的数据指针
 * @param length 长度
 * @return uint8_t 执行状态
 */
uint8_t UartSendData(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length)
{
    return (HAL_UART_Transmit_DMA(huart, data, length));
}

/**
 * @brief 发送字符数据帧
 *
 * @param huart UART编号
 * @param data 被发送的数据指针（字符数组）
 * @param length 长度
 * @return char 执行状态
 */
char UartSendCharData(UART_HandleTypeDef *huart, char *data, uint16_t length)
{
    if (huart == NULL || data == NULL || length == 0)
    {
        return HAL_ERROR;
    }

    return (HAL_UART_Transmit_DMA(huart, (uint8_t *)data, length));
}

/**
 * @brief UART的TIM定时器中断发送回调函数
 *
 */
void Tim1msUartPeriodElapsedCallback()
{
    // UART7串口绘图
    //UartSendData(&huart7, g_uart7_manage_object.tx_buffer_, 1 + 12 * sizeof(float));
}

} // namespace Driver

/**
 * @brief HAL库UART接收DMA空闲中断
 *
 * @param huart UART编号
 * @param Size 长度
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    //选择回调函数
    if (huart->Instance == USART1)
    {
        if (Driver::g_uart1_manage_object.callback_function_ != nullptr)
        {
            Driver::g_uart1_manage_object.callback_function_(Driver::g_uart1_manage_object.rx_buffer_, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, Driver::g_uart1_manage_object.rx_buffer_, Driver::g_uart1_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == USART2)
    {
        if (Driver::g_uart2_manage_object.callback_function_ != nullptr)
        {
            Driver::g_uart2_manage_object.callback_function_(Driver::g_uart2_manage_object.rx_buffer_, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, Driver::g_uart2_manage_object.rx_buffer_, Driver::g_uart2_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == USART3)
    {
        if (Driver::g_uart3_manage_object.callback_function_ != nullptr)
        {
            Driver::g_uart3_manage_object.callback_function_(Driver::g_uart3_manage_object.rx_buffer_, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, Driver::g_uart3_manage_object.rx_buffer_, Driver::g_uart3_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == UART4)
    {
        if (Driver::g_uart4_manage_object.callback_function_ != nullptr)
        {
            Driver::g_uart4_manage_object.callback_function_(Driver::g_uart4_manage_object.rx_buffer_, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, Driver::g_uart4_manage_object.rx_buffer_, Driver::g_uart4_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == UART5)
    {
        if (Driver::g_uart5_manage_object.callback_function_ != nullptr)
        {
            Driver::g_uart5_manage_object.callback_function_(Driver::g_uart5_manage_object.rx_buffer_, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, Driver::g_uart5_manage_object.rx_buffer_, Driver::g_uart5_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == USART6)
    {
        if (Driver::g_uart6_manage_object.callback_function_ != nullptr)
        {
            Driver::g_uart6_manage_object.callback_function_(Driver::g_uart6_manage_object.rx_buffer_, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, Driver::g_uart6_manage_object.rx_buffer_, Driver::g_uart6_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == UART7)
    {
        if (Driver::g_uart7_manage_object.callback_function_ != nullptr)
        {
            Driver::g_uart7_manage_object.callback_function_(Driver::g_uart7_manage_object.rx_buffer_, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, Driver::g_uart7_manage_object.rx_buffer_, Driver::g_uart7_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == UART8)
    {
        if (Driver::g_uart8_manage_object.callback_function_ != nullptr)
        {
            Driver::g_uart8_manage_object.callback_function_(Driver::g_uart8_manage_object.rx_buffer_, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, Driver::g_uart8_manage_object.rx_buffer_, Driver::g_uart8_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == UART9)
    {
        if (Driver::g_uart9_manage_object.callback_function_ != nullptr)
        {
            Driver::g_uart9_manage_object.callback_function_(Driver::g_uart9_manage_object.rx_buffer_, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, Driver::g_uart9_manage_object.rx_buffer_, Driver::g_uart9_manage_object.rx_buffer_length_);
    }
    else if (huart->Instance == USART10)
    {
        if (Driver::g_uart10_manage_object.callback_function_ != nullptr)
        {
            Driver::g_uart10_manage_object.callback_function_(Driver::g_uart10_manage_object.rx_buffer_, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, Driver::g_uart10_manage_object.rx_buffer_, Driver::g_uart10_manage_object.rx_buffer_length_);
    }
}

/************************ COPYRIGHT(C) ROBOPIONEER **************************/
