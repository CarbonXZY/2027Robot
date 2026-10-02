/**
 * @file alg_fsm.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 有限自动机
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_fsm.h"

/* Function prototypes -------------------------------------------------------*/

namespace Algorithm
{

/**
 * @brief 状态机初始化
 *
 * @param status_number 状态数量
 * @param now_status_serial 当前指定状态机初始编号
 */
void Fsm::Init(uint8_t status_number, uint8_t now_status_serial)
{
    status_number_ = status_number;

    now_status_serial_ = now_status_serial;

    // 所有状态全刷0
    for (int i = 0; i < status_number_; i++)
    {
        status_[i].status_stage = StatusStage::kDisable;
        status_[i].count_time = 0;
    }

    // 使能初始状态
    status_[now_status_serial_].status_stage = StatusStage::kEnable;
}

/**
 * @brief 定时器处理函数, 计算周期与模型有关
 * 这是一个模板, 使用时请根据不同处理情况在不同文件内重新定义
 *
 */
void Fsm::TimCalculatePeriodElapsedCallback()
{
    status_[now_status_serial_].count_time++;

    // 自己接着编写状态转移函数
}

} // namespace Algorithm

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
