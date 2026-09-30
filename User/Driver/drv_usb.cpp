/**
 * @file drv_usb.cpp
 * @author lyh
 * @brief  USB虚拟串口通信初始化与配置流程
 * @version 0.1
 * @date 2026-09-21
 *
 * @copyright
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_usb.h"
#include "usbd_cdc_if.h"
#include "alg_circular_buffer.h"
#include <array>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

extern PCD_HandleTypeDef hpcd_USB_OTG_HS;

namespace Driver
{

UsbRxCallback g_rx_callback = nullptr;

volatile bool g_tx_process_flag = false;
volatile bool g_rx_process_flag = false;

static Algorithm::CircularBuffer<std::array<uint8_t, 64>, kUsbTxBufferSlot> g_tx_buffer = {};

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化USB
 *
 * @param rx_callback 接收回调函数
 */
void UsbInit(UsbRxCallback rx_callback)
{
	g_rx_callback = rx_callback;
	g_tx_process_flag = false;
}

/**
 * @brief USB异步发送, 把数据加入发送环形缓冲区
 * @param buf 发送缓冲区
 * @param len 缓冲区长度, 大于64会被截断
 * @return USB_Status
 */
USB_Status UsbTransmitAsync(uint8_t* buf, uint16_t len)
{
	if (len > 64) len = 64;

	std::array<uint8_t, 64> tx_buffer = {};
	memcpy(tx_buffer.data(), buf, len);
	if (g_tx_buffer.Push(tx_buffer))
	{
		return USB_Status::kOk;
	}
	return USB_Status::kFailed;
}

/**
 *
 * @param buf 发送缓冲区
 * @param len 缓冲区长度
 * @return USB_Status
 */
USB_Status UsbTransmit(uint8_t* buf, uint16_t len)
{
	auto res = CDC_Transmit_HS(buf, len);
	if (res == USBD_OK)
	{
		g_tx_process_flag = true;
	}
	return (res == USBD_OK) ? USB_Status::kOk : USB_Status::kFailed;
}

/**
 *
 * @param buf 发送缓冲区
 * @param len 缓冲区长度
 * @param timeout 超时时间, 毫秒
 * @return USB_Status
 */
USB_Status UsbTransmitBlocked(uint8_t* buf, uint16_t len, uint32_t timeout)
{
	// 先置标志位, 防止后面立马完成而失效
	g_tx_process_flag = true;
	auto res = CDC_Transmit_HS(buf, len);
	// 发送失败
	if (res != USBD_OK)
	{
		g_tx_process_flag = false;
		return USB_Status::kFailed;
	}

	// 忙等待发送结束
	const uint32_t start = HAL_GetTick();
	while (g_tx_process_flag)
	{
		if (HAL_GetTick() - start >= timeout)
		{
			g_tx_process_flag = false;
			return USB_Status::kTimeout;
		}
	}

	return USB_Status::kOk;
}

void UsbClearTransmitBuffer()
{
	g_tx_buffer.Clear();
}

USB_Status UsbResetBlocked(uint32_t delay_ms)
{
	UsbClearTransmitBuffer();
	g_tx_process_flag = false;

	if (HAL_PCD_Stop(&hpcd_USB_OTG_HS) != HAL_OK)
	{
		return USB_Status::kFailed;
	}

	HAL_Delay(delay_ms);

	if (HAL_PCD_Start(&hpcd_USB_OTG_HS) != HAL_OK)
	{
		return USB_Status::kFailed;
	}

	return USB_Status::kOk;
}

/**
 * @brief USB的TIM定时器中断发送回调函数, 调用周期即为发送周期
 * @note 实测每次发送间隔至少1ms, 若连续调用必丢包
 *
 */
void TimUsbSendPeriodElapsedCallback()
{
	std::array<uint8_t, 64> tx_buffer = {};

	// 先看发送缓冲区有没有数据
	if (g_tx_buffer.Pop(tx_buffer) == false) return;

	// 发送缓冲区存的数据
	UsbTransmit(tx_buffer.data(), 64);
}

} // namespace Driver

extern "C" void HAL_CDC_TxCpltCallback(void)
{
	Driver::g_tx_process_flag = false;
}

extern "C" void HAL_CDC_RxCpltCallback(uint8_t* pRxBuffer, uint16_t Length)
{
	Driver::g_rx_process_flag = true;
	if (Driver::g_rx_callback != nullptr)
	{
		Driver::g_rx_callback(pRxBuffer, Length);
	}
	Driver::g_rx_process_flag = false;
}

/************************ COPYRIGHT(C) NEUQ-RoboPioneers **************************/
