// Communication_Interface.hpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

namespace Control_Frame
{

// 中间层：算法 <-> 通信。线格式：[id][data][id][data]...
class Class_Communication_Interface
{
public:
    using Communication_Function = std::function<int64_t(uint8_t *buf, size_t len)>;  // 收发字节，返回字节数

    void Init(Communication_Function send, Communication_Function receive);

    // 注册一帧；tx/rx 可为空以支持只收/只发。失败返回 false。
    bool Register(uint8_t id, const void *tx, void *rx, uint8_t tx_size, uint8_t rx_size);

    void Send();
    void Receive();

private:
    static constexpr size_t kMaxFrames  = 8;
    static constexpr size_t kBufferSize = 128;

    struct Entry
    {
        uint8_t      id;
        const void * tx;
        void *       rx;
        uint8_t      tx_size;
        uint8_t      rx_size;
    };

    Entry *Find(uint8_t id);

    Entry         entries[kMaxFrames]{};
    size_t        count = 0;
    size_t        used = 0;
    uint8_t       buffer[kBufferSize]{};
    uint8_t       rx_buffer[kBufferSize]{};
    Communication_Function send_function{};
    Communication_Function receive_function{};
};

// 全局实例：chassis 与传输层共用同一个 Communication_Interface
extern Class_Communication_Interface USB_Communication_Interface;

}  // namespace Control_Frame
