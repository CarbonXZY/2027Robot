/**
 * @file    tsk_config_and_callback_lyh.cpp
 * @author  lyh
 * @date    2026-10-07
 * @brief   
 */
#ifdef DEBUG_LYH

/**
 * @file tsk_config_and_callback.cpp
 * @brief 任务调度与底层回调：驱动初始化、接线、周期分频
 *
 * 分工照 R1：Chariot 只提供几个回调，周期分频（mod）写在这里。
 * TIM5 已经配成 1ms（PSC 240-1，ARR 1000-1，240MHz），所以 mod 就是乘数：
 *   1ms    控制 + CAN 发送 + USB 上行
 *   100ms  电机自身掉线检测 + 遥控接收机掉线检测
 *   1000ms 上位机存活窗口
 * 判活窗口取 1000ms 而不是贴着上位机的 100ms：上位机是 sleep_for(100ms)，
 * 窗口跟它一般大就会相位打拍，偶发一个窗口整好一个包都不落，误判掉线。
 */

#include <cstddef>
#include <cstdint>

#include "Communication_Interface.hpp"
#include "drv_can.h"
#include "drv_tim.h"
#include "drv_uart.h"
#include "drv_usb.h"
#include "dvc_dwt.h"
#include "dvc_motor_instances.h"
#include "ita_chariot.h"
#include "tsk_config_and_callback.h"
#include "Gripper/mod_gripper.h"

namespace Task
{

// 全局初始化完成标志位
bool g_init_finished = false;

Chariot g_chariot;

Module::Gripper g_gripper(Device::g_motor.clamp);

namespace
{

uint8_t g_mod_alive_motor = 0;
uint16_t g_mod_alive_pc = 0;

// 底层 USB 收到一包就进来，转发给战车层（存活计数也在里面）
void UsbRxCallback(uint8_t *data, uint16_t length)
{
    g_chariot.McuRxCallback(data, length);
}

// 通信中间件的发送回调是裸函数指针，只能绑无捕获函数
int64_t UsbSendCallback(uint8_t *data, size_t length)
{
    return Driver::UsbTransmitAsync(data, static_cast<uint16_t>(length)) == Driver::USB_Status::kOk ? static_cast<int64_t>(length) : -1;
}

// 接收机（CRSF）串口收到一包就进来，转发给战车层
void CrsfUart7Callback(uint8_t *data, uint16_t length)
{
    g_chariot.CrsfRxCallback(data, length);
}

// CAN1 上 8 台 DJI 电机的反馈帧：0x201~0x204 腿关节，0x205~0x208 底盘轮
void Fdcan1Callback(Driver::FdcanRxBuffer *FDCAN_RxMessage)
{
    // 裸十六进制是真实 CAN 反馈 ID，与 MotorDjiId（1~8 的枚举）不是一回事
    switch (FDCAN_RxMessage->Header.Identifier)
    {
        case 0x201:
        {
            Device::g_motor.leg_front_left.FdcanRxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x202:
        {
            Device::g_motor.leg_front_right.FdcanRxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x203:
        {
            Device::g_motor.leg_rear_left.FdcanRxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x204:
        {
            Device::g_motor.leg_rear_right.FdcanRxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x205:
        {
            Device::g_motor.chassis_front_left.FdcanRxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x206:
        {
            Device::g_motor.chassis_front_right.FdcanRxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x207:
        {
            Device::g_motor.chassis_rear_left.FdcanRxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x208:
        {
            Device::g_motor.chassis_rear_right.FdcanRxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        default:
            break;
    }
}

void Fdcan2Callback(Driver::FdcanRxBuffer *FDCAN_RxMessage)
{
    switch (FDCAN_RxMessage->Header.Identifier)
    {
    case 0x201:
        Device::g_motor.clamp.FdcanRxCpltCallback(nullptr);
        break;

    default:
        break;
    }
}

/**
 * @brief TIM5 1ms 回调，周期分频都在这
 */
void Tim1msCallback()
{
    Device::DwtUpdate();

    // g_chariot.TimCalculatePeriodElapsedCallback();
    g_gripper.TimCalculate1msCallback();

    // 发送CAN帧
    Driver::Tim1msCanPeriodElapsedCallback();

    // 推一下 USB 发送环形缓冲，把上行帧真正发出去
    Driver::TimUsbSendPeriodElapsedCallback();

    if (++g_mod_alive_motor >= 100)
    {
        g_mod_alive_motor = 0;
        // g_chariot.Tim100msAlivePeriodElapsedCallback();
        g_gripper.TimAlive100msCallback();
    }

    if (++g_mod_alive_pc >= 1000)
    {
        g_mod_alive_pc = 0;
        // g_chariot.Tim1000msAlivePeriodElapsedCallback();
    }
}

} // namespace

} // namespace Task

void TaskInit()
{
    HAL_Delay(1000);

    // 驱动层初始化：使能 DWT 周期计数器（堵转消抖等计时靠它）
    Device::DwtInit();

    Driver::UsbInit(Task::UsbRxCallback);

    // 通信中间件绑上 USB 发送通道
    Middleware::g_usb_communication_interface.Init(Task::UsbSendCallback);

    // Task::g_chariot.Init();
    Task::g_gripper.Init();

    // 绑定 CRSF 串口回调
    UART_Init(&huart7, Task::CrsfUart7Callback, 64);

    // 电机对象先绑好再开 CAN 中断：反过来的话，FdcanInit 激活 RX 中断的瞬间
    // 电调正推反馈，回调里 manage_object 还是空指针
    Driver::FdcanInit(&hfdcan1, Task::Fdcan1Callback);
    Driver::FdcanInit(&hfdcan2, Task::Fdcan2Callback);

    // 定时器初始化：TIM5 挂 1ms 调度回调
    Driver::TimInit(&htim5, Task::Tim1msCallback);

    // 使能调度时钟
    HAL_TIM_Base_Start_IT(&htim5);

    // 标记初始化完成
    Task::g_init_finished = true;
}

/**
 * @brief 前台循环任务
 *
 */
void TaskLoop()
{
}

/************************ COPYRIGHT(C) NEUQ-RoboPioneers **************************/


#endif
