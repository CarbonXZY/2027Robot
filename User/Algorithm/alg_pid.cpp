/**
 * @file alg_pid.cpp
 * @author yssickjgd 1345578933@qq.com
 * @brief PID算法
 * @version 0.1
 * @date 2022-05-08
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_pid.h"

namespace Algorithm
{

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief PID初始化
 *
 * @param k_p P值
 * @param k_i I值
 * @param k_d D值
 * @param k_f 前馈
 * @param i_out_max 积分限幅
 * @param out_max 输出限幅
 * @param d_t 时间片长度
 */
void Pid::Init(float k_p, float k_i, float k_d, float k_f, float i_out_max, float out_max, float d_t, float dead_zone, float i_variable_speed_a, float i_variable_speed_b, float i_separate_threshold, PidDFirst d_first)
{
    k_p_ = k_p;
    k_i_ = k_i;
    k_d_ = k_d;
    k_f_ = k_f;
    i_out_max_ = i_out_max;
    out_max_ = out_max;
    d_t_ = d_t;
    dead_zone_ = dead_zone;
    i_variable_speed_a_ = i_variable_speed_a;
    i_variable_speed_b_ = i_variable_speed_b;
    i_separate_threshold_ = i_separate_threshold;
    d_first_ = d_first;
}

/**
 * @brief 获取误差值
 *
 * @return float 误差值
 */
float Pid::GetIntegralError()
{
    return (integral_error_);
}

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
float Pid::GetOut()
{
    return (out_);
}

/**
 * @brief 设定PID的P
 *
 * @param k_p PID的P
 */
void Pid::SetKP(float k_p)
{
    k_p_ = k_p;
}

/**
 * @brief 设定PID的I
 *
 * @param k_i PID的I
 */
void Pid::SetKI(float k_i)
{
    k_i_ = k_i;
}

/**
 * @brief 设定PID的D
 *
 * @param k_d PID的D
 */
void Pid::SetKD(float k_d)
{
    k_d_ = k_d;
}

/**
 * @brief 设定前馈
 *
 * @param k_f 前馈
 */
void Pid::SetKF(float k_f)
{
    k_f_ = k_f;
}

/**
 * @brief 设定积分限幅, 0为不限制
 *
 * @param i_out_max 积分限幅, 0为不限制
 */
void Pid::SetIOutMax(float i_out_max)
{
    i_out_max_ = i_out_max;
}

/**
 * @brief 设定输出限幅, 0为不限制
 *
 * @param out_max 输出限幅, 0为不限制
 */
void Pid::SetOutMax(float out_max)
{
    out_max_ = out_max;
}

/**
 * @brief 设定定速内段阈值, 0为不限制
 *
 * @param i_variable_speed_a 定速内段阈值, 0为不限制
 */
void Pid::SetIVariableSpeedA(float i_variable_speed_a)
{
    i_variable_speed_a_ = i_variable_speed_a;
}

/**
 * @brief 设定变速区间, 0为不限制
 *
 * @param i_variable_speed_b 变速区间, 0为不限制
 */
void Pid::SetIVariableSpeedB(float i_variable_speed_b)
{
    i_variable_speed_b_ = i_variable_speed_b;
}

/**
 * @brief 设定积分分离阈值，需为正数, 0为不限制
 *
 * @param i_separate_threshold 积分分离阈值，需为正数, 0为不限制
 */
void Pid::SetISeparateThreshold(float i_separate_threshold)
{
    i_separate_threshold_ = i_separate_threshold;
}

/**
 * @brief 设定目标值
 *
 * @param target 目标值
 */
void Pid::SetTarget(float target)
{
    target_ = target;
}

/**
 * @brief 设定当前值
 *
 * @param now 当前值
 */
void Pid::SetNow(float now)
{
    now_ = now;
}

/**
 * @brief 设定积分, 一般用于积分清零
 *
 * @param integral_error 积分值
 */
void Pid::SetIntegralError(float integral_error)
{
    integral_error_ = integral_error;
}

/**
 * @brief PID调整值
 *
 * @return float 输出值
 */
void Pid::TimCalculatePeriodElapsedCallback()
{
    // P输出
    float p_out = 0.0f;
    // I输出
    float i_out = 0.0f;
    // D输出
    float d_out = 0.0f;
    // F输出
    float f_out = 0.0f;
    //误差
    float error;
    //绝对值误差
    float abs_error;
    //线性变速积分
    float speed_ratio;

    error = target_ - now_;
    abs_error = Driver::MathAbs(error);

    //判断死区
    if (abs_error < dead_zone_)
    {
        target_ = now_;
        error = 0.0f;
        abs_error = 0.0f;
    }

    //计算p项

    p_out = k_p_ * error;

    //计算i项

    if (i_variable_speed_a_ == 0.0f && i_variable_speed_b_ == 0.0f)
    {
        //非变速积分
        speed_ratio = 1.0f;
    }
    else
    {
        //变速积分
        if (abs_error <= i_variable_speed_b_)
        {
            speed_ratio = 1.0f;
        }
        else if (i_variable_speed_b_ < abs_error && abs_error < i_variable_speed_a_ + i_variable_speed_b_)
        {
            speed_ratio = (i_variable_speed_a_ + i_variable_speed_b_ - abs_error) / i_variable_speed_a_;
        }
        else if (abs_error >= i_variable_speed_b_)
        {
            speed_ratio = 0.0f;
        }
    }

    //积分限幅
    if (i_out_max_ != 0.0f)
    {
        Driver::MathConstrain(&integral_error_, -i_out_max_ / k_i_, i_out_max_ / k_i_);
    }

    if (i_separate_threshold_ == 0.0f)
    {
        //没有积分分离
        integral_error_ += speed_ratio * d_t_ * error;
        i_out = k_i_ * integral_error_;
    }
    else
    {
        //积分分离使能
        if (abs_error < i_separate_threshold_)
        {
            integral_error_ += speed_ratio * d_t_ * error;
            i_out = k_i_ * integral_error_;
        }
        else
        {
            integral_error_ = 0.0f;
            i_out = 0.0f;
        }
    }

    //计算d项

    if (d_first_ == PidDFirst::kDisable)
    {
        //没有微分先行
        d_out = k_d_ * (error - pre_error_) / d_t_;
    }
    else
    {
        //微分先行使能
        d_out = k_d_ * (out_ - pre_out_) / d_t_;
    }

    //计算前馈

    f_out = (target_ - pre_target_) * k_f_;

    //计算总共的输出

    out_ = p_out + i_out + d_out + f_out;
    //输出限幅
    if (out_max_ != 0.0f)
    {
        Driver::MathConstrain(&out_, -out_max_, out_max_);
    }

    //善后工作
    pre_now_ = now_;
    pre_target_ = target_;
    pre_out_ = out_;
    pre_error_ = error;
}

} // namespace Algorithm

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
