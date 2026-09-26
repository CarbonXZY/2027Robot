/**
 * @file    tsk_config_and_callback.hpp
 * @author  Carbon
 * @brief   任务配置与回调接口声明
 * @version 1.0
 * @date    2026-09-27
 *
 * @note    供 chassis / leg 等下游包 include，导出 Task_Init / Task_Loop / MCU_Alive
 */

#pragma once

namespace Task
{

/**
 * @brief 打开 USB-CDC，并把 USB_Communication_Interface 的收发接到它上面
 * @note  打不开就忙等重试，直到打开成功才返回 true
 */
bool Task_Init();

/**
 * @brief 下位机链路存活标志位
 */
extern bool MCU_Alive;

/**
 * @brief Task_Init() 是否已完成：设备只开一次，chassis/leg 多包共用
 */
extern bool initialized;

/**
 * @brief 到点检测一次下位机是否存活，由控制循环按窗口周期调用
 */
void Alive_PeriodElapsedCallback();

/**
 * @brief 控制循环每拍调一次：存活才往外发
 */
void Task_Loop();

}  // namespace Task
