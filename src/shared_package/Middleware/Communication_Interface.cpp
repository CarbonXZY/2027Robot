// Communication_Interface.cpp
#include "Communication_Interface.hpp"

#include <cstring>
#include <utility>

namespace Control_Frame
{

    void Class_Communication_Interface::Init(Communication_Function send, Communication_Function receive)
    {
        send_function = std::move(send);
        receive_function = std::move(receive);
        // 不清空 count/used：绑定由 Register 管理，Init 只负责设置收发函数
    }

    bool Class_Communication_Interface::Register(uint8_t id, const void *tx, void *rx, uint8_t tx_size, uint8_t rx_size)
    {
        if ((tx == nullptr && rx == nullptr) || (tx_size == 0 && rx_size == 0))
        {
            return false;
        }
        if (count >= kMaxFrames)
        {
            return false;
        }
        if (tx != nullptr && used + 1 + tx_size > kBufferSize)
        {
            return false;
        }

        if (tx != nullptr)
        {
            used += 1 + tx_size;
        }
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

    void Class_Communication_Interface::Receive()
    {
        if (!receive_function)
        {
            return;
        }
        const int64_t n = receive_function(rx_buffer, kBufferSize);
        if (n <= 0)
        {
            return;
        }

        size_t i = 0;
        while (i < static_cast<size_t>(n))
        {
            const uint8_t id = rx_buffer[i++];
            Entry *e = Find(id);
            if (e == nullptr || e->rx_size > static_cast<size_t>(n) - i)
            {
                return;
            }
            if (e->rx != nullptr)
            {
                std::memcpy(e->rx, rx_buffer + i, e->rx_size);
            }
            i += e->rx_size;
        }
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
            *p++ = entries[i].id;
            std::memcpy(p, entries[i].tx, entries[i].tx_size);
            p += entries[i].tx_size;
        }
        send_function(buffer, static_cast<size_t>(p - buffer));
    }

Class_Communication_Interface USB_Communication_Interface;

} // namespace Control_Frame
