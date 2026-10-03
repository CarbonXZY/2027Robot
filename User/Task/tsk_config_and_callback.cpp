/**
 * @file tsk_config_and_callback.cpp
 * @brief 任务调度与底层回调：驱动初始化、接线、周期分频
 *
 * 分工照 R1：Chariot 只提供几个回调，周期分频（mod）写在这里。
 * TIM5 已经配成 1ms（PSC 240-1，ARR 1000-1，240MHz），所以 mod 就是乘数：
 *   1ms    控制 + CAN 发送 + USB 上行
 *   100ms  电机自身掉线检测
 *   1000ms 上位机存活窗口
 * 判活窗口取 1000ms 而不是贴着上位机的 100ms：上位机是 sleep_for(100ms)，
 * 窗口跟它一般大就会相位打拍，偶发一个窗口整好一个包都不落，误判掉线。
 */

#include "drv_can.h"
#include "drv_tim.h"
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

//Chariot g_chariot;
Device::MotorDjiC620 m3508(&hfdcan1, Device::MotorDjiId::kId0x201);

Module::Gripper g_gripper(Device::g_motor_clamp);

namespace
{

uint8_t g_mod_alive_motor = 0;
uint16_t g_mod_alive_pc = 0;

// 底层 USB 收到一包就进来，转发给战车层（存活计数也在里面）
void UsbRxCallback(uint8_t *data, uint16_t length)
{
    //g_chariot.McuRxCallback(data, length);
}

// 四台腿关节电机的反馈帧
void Fdcan1Callback(Driver::FdcanRxBuffer *FDCAN_RxMessage)
{
    // 裸十六进制是真实 CAN 反馈 ID，与 MotorDjiId（1~8 的枚举）不是一回事
    switch (FDCAN_RxMessage->Header.Identifier)
    {
        case 0x201:
        {
            //motor_test::CanRxCallBack();
            //Device::g_leg_front_left.FdcanRxCpltCallback(FDCAN_RxMessage->Data);
            Device::g_motor_clamp.FdcanRxCpltCallback(nullptr);
            break;
        }
        case 0x202:
        {
            //Device::g_leg_front_right.FdcanRxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x203:
        {
            //Device::g_leg_rear_left.FdcanRxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
        case 0x204:
        {
            //Device::g_leg_rear_right.FdcanRxCpltCallback(FDCAN_RxMessage->Data);
            break;
        }
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

    //g_chariot.TimCalculatePeriodElapsedCallback();

    // 推一下 USB 发送环形缓冲，把上行帧真正发出去
    Driver::TimUsbSendPeriodElapsedCallback();

    //motor_test::Timer1msCalculateCallback();
    g_gripper.TimCalculate1msCallback();

    // 发送CAN数据帧
    Driver::Tim1msCanPeriodElapsedCallback();

    if (++g_mod_alive_motor >= 100)
    {
        g_mod_alive_motor = 0;
        //g_chariot.Tim100msAlivePeriodElapsedCallback();
        //motor_test::Timer100msAliveCallback();

        g_gripper.TimAlive100msCallback();
    }

    if (++g_mod_alive_pc >= 1000)
    {
        g_mod_alive_pc = 0;
        //g_chariot.Tim1000msAlivePeriodElapsedCallback();
    }
}

} // namespace

} // namespace Task

void TaskInit()
{
    // 驱动层初始化：使能 DWT 周期计数器（堵转消抖等计时靠它）
    Device::DwtInit();

    Driver::UsbInit(Task::UsbRxCallback);

    //// 战车层：中间件接线 + 注册两帧（各电机的硬件初始化在 Device::g_leg_* 构造函数里）
    //Task::g_chariot.Init();

    //motor_test::Init();
    Task::g_gripper.Init();

    // 电机对象先绑好再开 CAN 中断：反过来的话，FdcanInit 激活 RX 中断的瞬间
    // 电调正推反馈，回调里 manage_object 还是空指针
    Driver::FdcanInit(&hfdcan1, Task::Fdcan1Callback);

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
