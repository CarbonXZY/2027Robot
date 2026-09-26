// tsk_config_and_callback.hpp
#pragma once

namespace Control_Frame
{

// 打开 USB-CDC，并把 USB_Communication_Interface 的收发接到它上面。
// 打不开返回 false。
bool Task_Init();

// 下位机链路存活标志位
extern bool Alive;

// Task_Init() 是否已完成：设备只开一次，chassis/leg 多包共用
extern bool initialized;

/**
 * @brief 到点检测一次下位机是否存活，由控制循环按窗口周期调用
 *
 */
void Alive_PeriodElapsedCallback();

/**
 * @brief 控制循环每拍调一次：存活才往外发
 *
 */
void Task_Loop();

}  // namespace Control_Frame
