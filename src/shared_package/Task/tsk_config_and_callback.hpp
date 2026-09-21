// tsk_config_and_callback.hpp
#pragma once

namespace Control_Frame
{

// 打开 USB-CDC，并把 USB_Communication_Interface 的收发接到它上面。
// 打不开返回 false。
bool Task_Init();

}  // namespace Control_Frame
