// sim_slave.hpp
#pragma once

#include <cstddef>
#include <cstdint>

// 模拟从机（电机侧），纯字节层：收主机一整包字节，校验、解析，再构造一整包回包。
// 返回回包字节数。这里故意不复用 Middleware，自己写一遍线格式，
// 这样主机侧打包/解包写错了才能被发现（两边共用一份实现的话，错了也是对称地错）。
size_t Sim_Slave_Handle(const uint8_t * in, size_t in_len, uint8_t * out, size_t out_cap);
