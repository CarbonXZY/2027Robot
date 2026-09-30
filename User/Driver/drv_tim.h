/**
 * @file drv_tim.h
 * @author Lucy (2478427315@qq.com)
 * @brief 仿照SCUT-Robotlab改写的TIM定时器初始化与配置流程
 * @version 0.1
 * @date 2024-10-19 0.1 24-25赛季定稿
 *
 * @copyright RoboPionner
 *
 */

#ifndef DRV_TIM_H
#define DRV_TIM_H

/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx_hal.h"

/* CubeMX generated handle (global namespace) -------------------------------*/

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim12;

namespace Task
{
// Task 层初始化完成后置位，TIM 调度回调以它为门
extern bool g_init_finished;
} // namespace Task

namespace Driver
{

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief TIM定时器回调函数数据类型
 *
 */
using TimCallback = void (*)();

/**
 * @brief TIM定时器处理结构体
 *
 */
struct TimManageObject
{
    TIM_HandleTypeDef *tim_handler_;
    TimCallback callback_function_;
};

/* Exported variables --------------------------------------------------------*/

extern TimManageObject g_tim1_manage_object;
extern TimManageObject g_tim2_manage_object;
extern TimManageObject g_tim3_manage_object;
extern TimManageObject g_tim4_manage_object;
extern TimManageObject g_tim5_manage_object;
extern TimManageObject g_tim6_manage_object;
extern TimManageObject g_tim7_manage_object;
extern TimManageObject g_tim8_manage_object;
extern TimManageObject g_tim12_manage_object;
extern TimManageObject g_tim13_manage_object;
extern TimManageObject g_tim14_manage_object;
extern TimManageObject g_tim15_manage_object;
extern TimManageObject g_tim16_manage_object;
extern TimManageObject g_tim17_manage_object;
extern TimManageObject g_tim23_manage_object;
extern TimManageObject g_tim24_manage_object;

/* Exported function declarations --------------------------------------------*/

void TimInit(TIM_HandleTypeDef *htim, TimCallback callback_function);

} // namespace Driver

#endif

/************************ COPYRIGHT(C) ROBOPIONNER **************************/
