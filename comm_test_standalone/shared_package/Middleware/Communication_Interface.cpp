// Communication_Interface.cpp
#include "Communication_Interface.hpp"

#include <cstring>

#include "alg_crc.h"

namespace Control_Frame
{

void Class_Communication_Interface::Init(Communication_Function send)
{
    // 不可重入：绑定只在 bringup 时做一次，count/used 归 Register 管，这里不动。
    send_function = send;
}

bool Class_Communication_Interface::Register(uint8_t id, const void *tx, void *rx, uint8_t tx_size, uint8_t rx_size)
{
    if (tx == nullptr && rx == nullptr)
    {
        return false;
    }
    if (tx_size == 0 && rx_size == 0)
    {
        return false;
    }
    if (Find(id) != nullptr)  // id 不允许重复注册，否则 Find() 是 first-match-wins，后面的永远被遮住
    {
        return false;
    }
    if (count >= kMaxFrames)
    {
        return false;
    }
    if (tx != nullptr && used + tx_size + kFrameOverhead > kBufferSize)
    {
        return false;
    }

    used += (tx == nullptr ? 0 : tx_size + kFrameOverhead);
    entries[count++] = Entry{id, tx, rx, tx_size, rx_size};
    return true;
}

Class_Communication_Interface::Entry *Class_Communication_Interface::Find(uint8_t id)
{
    for (size_t i = 0; i < count; ++i)
    {
        if (entries[i].id == id)
        {
            return &entries[i];
        }
    }
    return nullptr;
}

void Class_Communication_Interface::Send()
{
    if (!send_function || count == 0)
    {
        return;
    }

    uint8_t *p = buffer;
    for (size_t i = 0; i < count; ++i)
    {
        if (entries[i].tx == nullptr)
        {
            continue;
        }
        const size_t size  = entries[i].tx_size;
        const size_t frame = kIdSize + size + kCrcSize;

        // 这一帧装不进本段就先发本段。因为只在帧边界上切，每段开头必定是 id。
        if (p != buffer && static_cast<size_t>(p - buffer) + frame > kMaxSendSize)
        {
            send_function(buffer, static_cast<size_t>(p - buffer));
            p = buffer;
        }

        *p++ = entries[i].id;
        std::memcpy(p, entries[i].tx, size);
        const uint16_t crc = Algorithm::CRC_Lib::CRC16_CCITT::Calculate(p - kIdSize, kIdSize + size); // 覆盖 [id][data]
        p += size;
        *p++ = static_cast<uint8_t>(crc & 0xFF);
        *p++ = static_cast<uint8_t>(crc >> 8);
    }

    if (p != buffer)
    {
        send_function(buffer, static_cast<size_t>(p - buffer));
    }
}

void Class_Communication_Interface::Rx_RptlCallback(uint8_t *data, uint16_t length)
{
    // 一包就是完整的一批帧：帧长按 id 从 registry 查出，逐帧往前走，CRC 不过的那帧丢掉。
    Entry *e = nullptr;
    for (size_t i = 0; i + kFrameOverhead <= length; i += e->rx_size + kFrameOverhead)
    {
        e = Find(data[i]);
        if (e == nullptr || e->rx == nullptr)
        {
            break; // 未注册的 id，帧长无从得知
        }
        if (i + e->rx_size + kFrameOverhead > length)
        {
            break; // 不完整，丢掉
        }

        const uint16_t expect = static_cast<uint16_t>(data[i + kIdSize + e->rx_size] |
                                                      (static_cast<uint16_t>(data[i + kIdSize + e->rx_size + 1]) << 8));
        if (Algorithm::CRC_Lib::CRC16_CCITT::Calculate(data + i, kIdSize + e->rx_size) == expect)
        {
            std::memcpy(e->rx, data + i + kIdSize, e->rx_size);
        }
    }
}

Class_Communication_Interface USB_Communication_Interface;

} // namespace Control_Frame
