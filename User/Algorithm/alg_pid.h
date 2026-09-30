/**
 * @file alg_pid.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief PID算法
 * @version 0.1
 * @date 2022-05-03
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef ALG_PID_H
#define ALG_PID_H

/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

namespace Algorithm
{

/**
 * @brief 微分先行
 *
 */
enum class PidDFirst
{
    kDisable = 0,
    kEnable,
};

struct PidParameters
{
    float k_p = 0.0f;
    float k_i = 0.0f;
    float k_d = 0.0f;
    float k_f = 0.0f;

    float i_out_max = 0.0f;
    float out_max = 30.0f;
    float d_t = 0.001f;
    float dead_zone = 0.0f;

    float i_variable_speed_a = 0.0f;
    float i_variable_speed_b = 0.0f;
    float i_separate_threshold = 0.0f;

    PidDFirst d_first = PidDFirst::kDisable;
};

/**
 * @brief Reusable, PID算法
 *
 */
class Pid
{
public:
    void Init(float k_p, float k_i, float k_d, float k_f = 0.0f, float i_out_max = 0.0f, float out_max = 0.0f, float d_t = 0.001f, float dead_zone = 0.0f, float i_variable_speed_a = 0.0f, float i_variable_speed_b = 0.0f, float i_separate_threshold = 0.0f, PidDFirst d_first = PidDFirst::kDisable);

    float GetIntegralError();
    float GetOut();

    void SetKP(float k_p);
    void SetKI(float k_i);
    void SetKD(float k_d);
    void SetKF(float k_f);
    void SetIOutMax(float i_out_max);
    void SetOutMax(float out_max);
    void SetIVariableSpeedA(float i_variable_speed_a);
    void SetIVariableSpeedB(float i_variable_speed_b);
    void SetISeparateThreshold(float i_separate_threshold);
    void SetTarget(float target);
    void SetNow(float now);
    void SetIntegralError(float integral_error);

    void TimCalculatePeriodElapsedCallback();

protected:
    // 初始化相关常量

    // PID计时器周期, s
    float d_t_;
    // 死区, Error在其绝对值内不输出
    float dead_zone_;
    // 微分先行
    PidDFirst d_first_;

    // 常量

    // 内部变量

    // 之前的当前值
    float pre_now_ = 0.0f;
    // 之前的目标值
    float pre_target_ = 0.0f;
    // 之前的输出值
    float pre_out_ = 0.0f;
    // 前向误差
    float pre_error_ = 0.0f;

    // 读变量

    // 输出值
    float out_ = 0.0f;

    // 写变量

    // PID的P
    float k_p_ = 0.0f;
    // PID的I
    float k_i_ = 0.0f;
    // PID的D
    float k_d_ = 0.0f;
    // 前馈
    float k_f_ = 0.0f;

    // 积分限幅, 0为不限制
    float i_out_max_ = 0;
    // 输出限幅, 0为不限制
    float out_max_ = 0;

    // 变速积分定速内段阈值, 0为不限制
    float i_variable_speed_a_ = 0.0f;
    // 变速积分变速区间, 0为不限制
    float i_variable_speed_b_ = 0.0f;
    // 积分分离阈值，需为正数, 0为不限制
    float i_separate_threshold_ = 0.0f;

    // 目标值
    float target_ = 0.0f;
    // 当前值
    float now_ = 0.0f;

    // 读写变量

    // 积分值
    float integral_error_ = 0.0f;

    // 内部函数
};

} // namespace Algorithm

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
