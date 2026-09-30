/**
 * @file drv_tim.cpp
 * @author Lucy (2478427315@qq.com)
 * @brief 仿照SCUT-Robotlab改写的TIM定时器初始化与配置流程
 * @version 0.1
 * @date 2024-10-19 0.1 24-25赛季定稿
 *
 * @copyright RoboPionner
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_tim.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

namespace Driver
{

TimManageObject g_tim1_manage_object;
TimManageObject g_tim2_manage_object;
TimManageObject g_tim3_manage_object;
TimManageObject g_tim4_manage_object;
TimManageObject g_tim5_manage_object;
TimManageObject g_tim6_manage_object;
TimManageObject g_tim7_manage_object;
TimManageObject g_tim8_manage_object;
TimManageObject g_tim12_manage_object;
TimManageObject g_tim13_manage_object;
TimManageObject g_tim14_manage_object;
TimManageObject g_tim15_manage_object;
TimManageObject g_tim16_manage_object;
TimManageObject g_tim17_manage_object;
TimManageObject g_tim23_manage_object;
TimManageObject g_tim24_manage_object;


/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化TIM定时器
 *
 * @param htim 定时器编号
 * @param callback_function 处理回调函数
 */
void TimInit(TIM_HandleTypeDef *htim, TimCallback callback_function)
{
    if (htim->Instance == TIM1)
    {
        g_tim1_manage_object.tim_handler_ = htim;
        g_tim1_manage_object.callback_function_ = callback_function;
    }
    else if (htim->Instance == TIM2)
    {
        g_tim2_manage_object.tim_handler_ = htim;
        g_tim2_manage_object.callback_function_ = callback_function;
    }
    else if (htim->Instance == TIM3)
    {
        g_tim3_manage_object.tim_handler_ = htim;
        g_tim3_manage_object.callback_function_ = callback_function;
    }
    else if (htim->Instance == TIM4)
    {
        g_tim4_manage_object.tim_handler_ = htim;
        g_tim4_manage_object.callback_function_ = callback_function;
    }
    else if (htim->Instance == TIM5)
    {
        g_tim5_manage_object.tim_handler_ = htim;
        g_tim5_manage_object.callback_function_ = callback_function;
    }
    else if (htim->Instance == TIM6)
    {
        g_tim6_manage_object.tim_handler_ = htim;
        g_tim6_manage_object.callback_function_ = callback_function;
    }
    else if (htim->Instance == TIM7)
    {
        g_tim7_manage_object.tim_handler_ = htim;
        g_tim7_manage_object.callback_function_ = callback_function;
    }
    else if (htim->Instance == TIM8)
    {
        g_tim8_manage_object.tim_handler_ = htim;
        g_tim8_manage_object.callback_function_ = callback_function;
    }
    else if (htim->Instance == TIM12)
    {
        g_tim12_manage_object.tim_handler_ = htim;
        g_tim12_manage_object.callback_function_ = callback_function;
    }
    else if (htim->Instance == TIM13)
    {
        g_tim13_manage_object.tim_handler_ = htim;
        g_tim13_manage_object.callback_function_ = callback_function;
    }
    else if (htim->Instance == TIM14)
    {
        g_tim14_manage_object.tim_handler_ = htim;
        g_tim14_manage_object.callback_function_ = callback_function;
    }
    else if (htim->Instance == TIM15)
    {
        g_tim15_manage_object.tim_handler_ = htim;
        g_tim15_manage_object.callback_function_ = callback_function;
    }
    else if (htim->Instance == TIM16)
    {
        g_tim16_manage_object.tim_handler_ = htim;
        g_tim16_manage_object.callback_function_ = callback_function;
    }
    else if (htim->Instance == TIM17)
    {
        g_tim17_manage_object.tim_handler_ = htim;
        g_tim17_manage_object.callback_function_ = callback_function;
    }
		else if (htim->Instance == TIM23)
    {
        g_tim23_manage_object.tim_handler_ = htim;
        g_tim23_manage_object.callback_function_ = callback_function;
    }
		else if (htim->Instance == TIM24)
    {
        g_tim24_manage_object.tim_handler_ = htim;
        g_tim24_manage_object.callback_function_ = callback_function;
    }
}

} // namespace Driver

/**
 * @brief HAL库TIM定时器中断
 *
 * @param htim TIM编号
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 判断程序初始化完成
    if (Task::g_init_finished == false)
    {
        return;
    }

    // 选择回调函数
    if (htim->Instance == TIM1)
    {
        if(Driver::g_tim1_manage_object.callback_function_ != nullptr)
        {
            Driver::g_tim1_manage_object.callback_function_();
        }
    }
    else if (htim->Instance == TIM2)
    {
        if(Driver::g_tim2_manage_object.callback_function_ != nullptr)
        {
            Driver::g_tim2_manage_object.callback_function_();
        }
    }
    else if (htim->Instance == TIM3)
    {
        if(Driver::g_tim3_manage_object.callback_function_ != nullptr)
        {
            Driver::g_tim3_manage_object.callback_function_();
        }
    }
    else if (htim->Instance == TIM4)
    {
        if(Driver::g_tim4_manage_object.callback_function_ != nullptr)
        {
            Driver::g_tim4_manage_object.callback_function_();
        }
    }
    else if (htim->Instance == TIM5)
    {
        if(Driver::g_tim5_manage_object.callback_function_ != nullptr)
        {
            Driver::g_tim5_manage_object.callback_function_();
        }
    }
    else if (htim->Instance == TIM6)
    {
        if(Driver::g_tim6_manage_object.callback_function_ != nullptr)
        {
            Driver::g_tim6_manage_object.callback_function_();
        }
    }
    else if (htim->Instance == TIM7)
    {
        if(Driver::g_tim7_manage_object.callback_function_ != nullptr)
        {
            Driver::g_tim7_manage_object.callback_function_();
        }
    }
    else if (htim->Instance == TIM8)
    {
        if(Driver::g_tim8_manage_object.callback_function_ != nullptr)
        {
            Driver::g_tim8_manage_object.callback_function_();
        }
    }
    else if (htim->Instance == TIM12)
    {
        if(Driver::g_tim12_manage_object.callback_function_ != nullptr)
        {
            Driver::g_tim12_manage_object.callback_function_();
        }
    }
    else if (htim->Instance == TIM13)
    {
        if(Driver::g_tim13_manage_object.callback_function_ != nullptr)
        {
            Driver::g_tim13_manage_object.callback_function_();
        }
    }
    else if (htim->Instance == TIM14)
    {
        if(Driver::g_tim14_manage_object.callback_function_ != nullptr)
        {
            Driver::g_tim14_manage_object.callback_function_();
        }
    }
		else if (htim->Instance == TIM15)
    {
        if(Driver::g_tim15_manage_object.callback_function_ != nullptr)
        {
            Driver::g_tim15_manage_object.callback_function_();
        }
    }
    else if (htim->Instance == TIM16)
    {
        if(Driver::g_tim16_manage_object.callback_function_ != nullptr)
        {
            Driver::g_tim16_manage_object.callback_function_();
        }
    }
    else if (htim->Instance == TIM17)
    {
        if(Driver::g_tim17_manage_object.callback_function_ != nullptr)
        {
            Driver::g_tim17_manage_object.callback_function_();
        }
    }
		else if (htim->Instance == TIM23)
    {
        if(Driver::g_tim23_manage_object.callback_function_ != nullptr)
        {
            Driver::g_tim23_manage_object.callback_function_();
        }
    }
    else if (htim->Instance == TIM24)
    {
        if(Driver::g_tim24_manage_object.callback_function_ != nullptr)
        {
            Driver::g_tim24_manage_object.callback_function_();
        }
    }
}

/************************ COPYRIGHT(C) ROBOPIONNER **************************/
