// tsk_config_and_callback.cpp
#include "tsk_config_and_callback.hpp"

#include <memory>

#include "Communication_Interface.hpp"
#include "usb_cdc.hpp"

namespace Control_Frame
{

namespace
{
std::unique_ptr<USB_CDC> g_usb_device;  // 全局 USB-CDC 实例
}  // namespace

bool Task_Init()
{
    g_usb_device = std::make_unique<USB_CDC>(USB_CDC_DEFAULT_DEVICE);
    if (!g_usb_device->IsOpen())
    {
        g_usb_device.reset();
        return false;
    }

    // 收：USB 接收线程把一整包递进来，直接转给中间件分发。
    // 无捕获的 lambda 能隐式转成 USB_CDC::RxHandle 那种裸函数指针。
    g_usb_device->RegisterRxCallback([](uint8_t * data, uint16_t length) {
        USB_Communication_Interface.Rx_RptlCallback(data, length);
    });

    // 发：Send() 打包好一整包后在这里交给设备。
    // 用 TransmitNow 而不是 TransmitAdd：控制环 1 kHz，异步队列按 10 ms 周期排空会积压。
    USB_Communication_Interface.Init([](uint8_t * data, size_t length) -> int64_t {
        if (g_usb_device->TransmitNow(data, static_cast<uint16_t>(length)) == 0)
        {
            return -1;
        }
        return static_cast<int64_t>(length);
    });

    return true;
}

}  // namespace Control_Frame
