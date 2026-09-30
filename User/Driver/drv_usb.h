/**
 * @file drv_usb.h
 * @author lyh
 * @brief  USB虚拟串口通信初始化与配置流程
 * @version 0.1
 * @date 2026-09-21
 *
 * @copyright
 *
 */

#ifndef DRV_USB_H
#define DRV_USB_H

/* Includes ------------------------------------------------------------------*/

#include <cstdint>

namespace Driver
{

/* Exported macros -----------------------------------------------------------*/

// 发送缓冲区的槽位
constexpr uint8_t kUsbTxBufferSlot = 10;

/* Exported types ------------------------------------------------------------*/


/**
 * @brief USB通信接收回调函数指针
 *
 */
using UsbRxCallback = void (*)(uint8_t *buffer, uint16_t length);

enum class USB_Status : uint8_t
{
    kOk = 0,
    kFailed,
    kTimeout,
};


/* Exported variables --------------------------------------------------------*/


/* Exported function declarations --------------------------------------------*/

/**
 * @brief 初始化USB
 *
 * @param rx_callback 接收回调函数
 */
void UsbInit(UsbRxCallback rx_callback);

/**
 * @brief 非阻塞式发送, 调用后立马返回
 * @param buf 发送缓冲区
 * @param len 缓冲区长度
 * @return USB_Status
 */
USB_Status UsbTransmit(uint8_t* buf, uint16_t len);

/**
 * @brief 阻塞式发送, 内部会忙等待发送完成标志, 再返回
 * @param buf 发送缓冲区
 * @param len 缓冲区长度
 * @param timeout 超时时间, 毫秒
 * @return USB_Status
 */
USB_Status UsbTransmitBlocked(uint8_t* buf, uint16_t len, uint32_t timeout);

/**
 * @brief 异步发送, 仅把数据加入发送环形缓冲区, 在定时回调执行发送
 * @param buf 发送缓冲区
 * @param len 缓冲区长度, 大于64会被截断
 * @return USB_Status
 */
USB_Status UsbTransmitAsync(uint8_t* buf, uint16_t len);

/**
 * @brief 清除发送缓冲区
 */
void UsbClearTransmitBuffer();

/**
 * @brief USB的TIM定时器中断发送回调函数, 调用周期即为发送周期
 * @note 实测每次发送间隔至少1ms, 若连续调用必丢包
 *
 */
void TimUsbSendPeriodElapsedCallback();

USB_Status UsbResetBlocked(uint32_t delay_ms);

} // namespace Driver

#endif

/************************ COPYRIGHT(C) NEUQ-RoboPioneers **************************/
