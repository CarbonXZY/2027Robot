/**
 * @file    alg_interpolator.cpp
 * @author  lyh
 * @date    2026-10-14
 * @brief   两点间插值, 支持线性/多项式/S型曲线
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_interpolator.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

namespace Algorithm
{

/**
 * @brief 曲线映射, 将归一化进度按指定曲线映射为输出进度
 *        此函数为纯函数, 便于单独复用或测试
 *
 * @param progress 归一化进度, 定义域[0, 1], 超界时自动限幅
 * @param type 曲线类型
 * @return float 映射后的进度, 值域[0, 1], 两端严格取到0与1
 */
float Interpolator::Ease(float progress, Type type)
{
    // 限幅, 同时挡住曲线在端点处的舍入误差
    if (progress <= 0.0f)
    {
        return (0.0f);
    }
    if (progress >= 1.0f)
    {
        return (1.0f);
    }

    switch (type)
    {
        case Type::kPolynomial:
        {
            // 三次平滑, p^2 * (3 - 2p), 两端一阶导为0, 起步与收尾无速度突变
            return (progress * progress * (3.0f - 2.0f * progress));
        }
        case Type::kSigmoid:
        {
            // 逻辑斯蒂曲线, 在两端各取一点后归一化, 保证首尾严格为0与1
            const float half = 0.5f;
            const float start = 1.0f / (1.0f + std::exp(sigmoid_steepness_ * half));
            const float end = 1.0f / (1.0f + std::exp(-sigmoid_steepness_ * half));
            const float now = 1.0f / (1.0f + std::exp(-sigmoid_steepness_ * (progress - half)));

            return ((now - start) / (end - start));
        }
        case Type::kLinear:
        default:
        {
            return (progress);
        }
    }
}

/**
 * @brief 由当前步数重新计算插值结果
 *        使用绝对位置计算而非累加, 避免逐步累加的浮点漂移
 */
void Interpolator::Calculate()
{
    now_ = start_ + (end_ - start_) * Ease(static_cast<float>(count_) / static_cast<float>(step_), type_);
    if ((count_ >= step_))
    {
        is_finished_ = true;
    }
}

/**
 * @brief 插值初始化
 *
 * @param start 起始点
 * @param end 终止点
 * @param step 总步数, 不得为0, 最小为1
 * @param type 曲线类型
 */
void Interpolator::Reset(float start, float end, uint32_t step, Type type)
{
    start_ = start;
    end_ = end;
    // step不得为0, 最小为1
    step_ = (step == 0) ? 1 : step;
    type_ = type;
    count_ = 0;
    is_finished_ = false;
    Calculate();
}

/**
 * @brief 设定起始点, 会据此重新计算当前插值结果
 */
void Interpolator::SetStart(float start)
{
    start_ = start;
    Calculate();
}

/**
 * @brief 设定终止点, 会据此重新计算当前插值结果
 */
void Interpolator::SetEnd(float end)
{
    end_ = end;
    Calculate();
}

/**
 * @brief 直接跳转到第count步, 会据此重新计算当前插值结果, 超界时自动限幅
 */
void Interpolator::SetCount(uint32_t count)
{
    count_ = (count > step_) ? step_ : count;
    Calculate();
}

/**
 * @brief 设定曲线类型, 会据此重新计算当前插值结果
 */
void Interpolator::SetType(Type type)
{
    type_ = type;
    Calculate();
}

/**
 * @brief 设定曲线类型, 会据此重新计算当前插值结果
 */
void Interpolator::SetSigmoidSteepness(float steepness)
{
    sigmoid_steepness_ = steepness;
    Calculate();
}

/**
 * @brief 每次调用前进一步, 并返回当前插值结果, 已结束时保持终止点
 *
 * @return float 当前插值结果
 */
float Interpolator::Update()
{
    prev_ = now_;
    if (count_ < step_)
    {
        count_++;
        Calculate();
    }

    return (now_);
}

} // namespace Algorithm

/* Exported function definitions ---------------------------------------------*/

/************************ COPYRIGHT(C) NEUQ-RoboPioneers **************************/
