// tsk_config_and_callback.cpp
#include "tsk_config_and_callback.hpp"

#include <cstdint>
#include <memory>

#include "Communication_Interface.hpp"
#include "usb_cdc.hpp"

namespace Control_Frame
{

namespace
{
std::unique_ptr<USB_CDC> g_usb_device; // 全局 USB-CDC 实例

uint32_t Alive_Flag = 0;
uint32_t Pre_Alive_Flag = 0;

// 收：USB 接收线程把一整包递进来。先记一次计数，链路确认之前一律不解包
void MCU_RxCallback(uint8_t *data, uint16_t length)
{
    Alive_Flag += 1;

    if (!Alive)
    {
        return;
    }
    USB_Communication_Interface.Rx_RptlCallback(data, length);
}

// 发：Send() 打包好一整包后在这里交给设备
int64_t MCU_SendData(uint8_t *data, size_t length)
{
    if (g_usb_device->TransmitAdd(data, static_cast<uint16_t>(length)) == 0)
    {
        return -1;
    }
    return static_cast<int64_t>(length);
}
} // namespace

// 下位机链路存活标志位
bool Alive = false;

// Task_Init() 是否已完成
bool initialized = false;

/**
 * @brief 到点检测一次下位机是否存活，由控制循环按窗口周期调用
 *
 */
void Alive_PeriodElapsedCallback()
{
    if (Alive_Flag == Pre_Alive_Flag)
    {
        // 下位机断开连接
        Alive = false;
    }
    else
    {
        // 下位机保持连接
        Alive = true;
    }
    Pre_Alive_Flag = Alive_Flag;
}

/**
 * @brief 控制循环每拍调一次：存活才往外发
 *
 */
void Task_Loop()
{
    // 设备还没开起来：可能所有组件都还没走到 on_configure
    if (!initialized)
    {
        return;
    }

    Alive_PeriodElapsedCallback();

    // 没收到下位机回显之前一个字节都不发
    if (!Alive)
    {
        return;
    }

    USB_Communication_Interface.Send();
}

bool Task_Init()
{
    // 设备只开一次：chassis 和 leg 各自都会调进来，第二次起直接返回
    if (initialized)
    {
        return true;
    }

    g_usb_device = std::make_unique<USB_CDC>(USB_CDC_DEFAULT_DEVICE);
    if (!g_usb_device->IsOpen())
    {
        g_usb_device.reset();
        return false;
    }

    g_usb_device->RegisterRxCallback(MCU_RxCallback);
    USB_Communication_Interface.Init(MCU_SendData);

    initialized = true;
    return true;
}

} // namespace Control_Frame
