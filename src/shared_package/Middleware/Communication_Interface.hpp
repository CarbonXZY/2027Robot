// Communication_Interface.hpp
#pragma once

#include <cstddef>
#include <cstdint>

namespace Control_Frame
{

// 中间层：算法 <-> 通信。
//
// 线格式（每帧）：[id][data...][crc16_lo][crc16_hi]
//   id 1 字节；data 长度由 registry 按 id 查出（收侧用 rx_size），线上不带长度字段。
//   crc16 覆盖 [id][data...]
//
// 发送：控制线程调 Send()，把所有已注册的 tx 帧打包后交给 send_function。
// 接收：Rx_RptlCallback() 交给底层 USB 注册，在接收线程里被回调，
//       由它就地把 buffer 里的帧解包、校验并分发到各 rx 结构体。
class Class_Communication_Interface
{
public:
    using Communication_Function = int64_t (*)(uint8_t *buf, size_t len);  // 发送字节，返回字节数

    void Init(Communication_Function send);

    // 注册一帧；tx/rx 可为空以支持只收/只发。失败返回 false。
    bool Register(uint8_t id, const void *tx, void *rx, uint8_t tx_size, uint8_t rx_size);

    void Send();

    // 接收回调：给底层 USB 用，收到一包就调进来。只解包 + 转发，不跨调用保留状态。
    // 帧长按 id 从 registry 查出，逐帧往前走；CRC 不过的那一帧直接丢掉。
    void Rx_RptlCallback(uint8_t *data, uint16_t length);

private:
    static constexpr size_t kMaxFrames  = 8;
    static constexpr size_t kBufferSize = 128;  // 发送打包缓冲

    static constexpr size_t kIdSize        = 1;
    static constexpr size_t kCrcSize       = 2;
    static constexpr size_t kFrameOverhead = kIdSize + kCrcSize;  // 每帧固定开销

    // 单次交给 send_function 的字节上限，按对端接收缓冲的大小设。
    // 一包装不下就切成多段发，切点只在帧边界上，所以每段开头都是 id。
    static constexpr size_t kMaxSendSize = 64;

    struct Entry
    {
        uint8_t      id;
        const void * tx;
        void *       rx;
        uint8_t      tx_size;
        uint8_t      rx_size;
    };

    Entry *Find(uint8_t id);

    Entry  entries[kMaxFrames]{};
    size_t count = 0;
    size_t used  = 0;

    uint8_t buffer[kBufferSize]{};  // 发送打包缓冲，仅控制线程访问

    Communication_Function send_function{};
};

// 全局实例：chassis 与传输层共用同一个 Communication_Interface
extern Class_Communication_Interface USB_Communication_Interface;

}  // namespace Control_Frame
