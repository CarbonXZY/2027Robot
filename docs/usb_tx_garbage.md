# USB 上行乱码问题

- **日期**：2026-10-06
- **文件**：`User/Driver/drv_usb.cpp`
- **现象**：上位机收到的上行数据是乱码，遥控帧恒为 0

## 原因

发送函数把缓冲区放在了**栈上**：

```cpp
void TimUsbSendPeriodElapsedCallback()
{
    std::array<uint8_t, 64> tx_buffer = {};   // 栈局部变量
    if (g_tx_buffer.Pop(tx_buffer) == false) return;
    UsbTransmit(tx_buffer.data(), 64);        // 把指针交出去
}
```

往下走 `UsbTransmit → CDC_Transmit_HS`，而 `CDC_Transmit_HS` **不拷贝数据，只存指针**，
然后启动 USB DMA 异步发送并立即返回：

```c
USBD_CDC_SetTxBuffer(&hUsbDeviceHS, Buf, Len);  // 只存 Buf 指针
USBD_CDC_TransmitPacket(&hUsbDeviceHS);         // 异步发，立刻返回
```

问题就在这里：`TimUsbSendPeriodElapsedCallback` 是 TIM5 的 1ms 中断回调，
**函数一返回，栈帧就作废了**。而 USB DMA 还在照着那个指针读，
读到的已经是被别处复用的栈内存——里面是返回地址、RAM 指针等，
所以抓包看到的全是 `20 ff` / `24 05` 这种地址碎片，而不是真正的帧。

## 为什么改成 static

把缓冲区改成 `static`，它的生命周期就是**整个程序运行期**，
不再随函数返回而销毁，USB DMA 读多久都读得到正确的数据：

```cpp
static std::array<uint8_t, 64> tx_buffer = {};
```

一句话：**交给异步发送的缓冲区，生命周期必须长于这次发送本身**，
栈局部变量做不到，`static`（或全局、或调用方持有的对象）可以。
