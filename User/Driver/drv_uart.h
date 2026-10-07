/**
 * @file drv_uart.h
 * @author Lucy (2478427315@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-10-04
 *
 * @copyright RoboPioneer (c) 2024
 *
 */

#ifndef DRV_UART_H
#define DRV_UART_H

/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx_hal.h"
#include "string.h"

/* CubeMX generated handles (global namespace) -------------------------------*/

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart9;
extern UART_HandleTypeDef huart10;

namespace Driver
{

/* Exported macros -----------------------------------------------------------*/

// 缓冲区字节长度
constexpr uint16_t kUartBufferSize = 256;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART通信接收回调函数数据类型
 *
 */
using UartCallback = void (*)(uint8_t *buffer, uint16_t length);

/**
 * @brief UART通信处理结构体
 */
struct UartManageObject
{
    UART_HandleTypeDef *uart_handler_;
    uint8_t tx_buffer_[kUartBufferSize];
    uint8_t rx_buffer_[kUartBufferSize];
    uint16_t rx_buffer_length_;
    UartCallback callback_function_;
};

/* Exported variables --------------------------------------------------------*/

extern UartManageObject g_uart1_manage_object;
extern UartManageObject g_uart2_manage_object;
extern UartManageObject g_uart3_manage_object;
extern UartManageObject g_uart4_manage_object;
extern UartManageObject g_uart5_manage_object;
extern UartManageObject g_uart6_manage_object;
extern UartManageObject g_uart7_manage_object;
extern UartManageObject g_uart8_manage_object;
extern UartManageObject g_uart9_manage_object;
extern UartManageObject g_uart10_manage_object;

/* Exported function declarations --------------------------------------------*/

void UartInit(UART_HandleTypeDef *huart, UartCallback callback_function, uint16_t rx_buffer_length);

void UartReinit(UART_HandleTypeDef *huart);

uint8_t UartSendData(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length);
char UartSendCharData(UART_HandleTypeDef *huart, char *data, uint16_t length);

void Tim1msUartPeriodElapsedCallback();

} // namespace Driver

#endif

/************************ COPYRIGHT(C) ROBOPIONEER **************************/
