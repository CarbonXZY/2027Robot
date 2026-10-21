/**
 * @file    alg_interpolator.h
 * @author  lyh
 * @date    2026-10-14
 * @brief   两点间插值, 支持线性/多项式/S型曲线
 */

#ifndef ALG_INTERPOLATOR_H
#define ALG_INTERPOLATOR_H

/* Includes ------------------------------------------------------------------*/

#include <cmath>
#include <cstdint>

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

namespace Algorithm
{

/**
 * @brief 两点间插值
 *        以起始点与终止点为两端, 在step步内过渡
 *        now_ = start_ + (end_ - start_) * Ease(count_ / step_)
 *        曲线的区别只体现在Ease对进度的映射上, 两端始终严格取到起始点与终止点
 *        无构造函数, 实例化后须调用Reset进行初始化
 */
class Interpolator
{
public:
    //

    enum class Type
    {
        kLinear = 0,
        kPolynomial,
        kSigmoid,
    };

    void Reset(float start, float end, uint32_t step, Type type = Type::kLinear);

    // 读变量

    // 获取当前插值结果
    float GetNow() const { return (now_); }
    // 获取上一次插值结果
    float GetPrev() const { return (prev_); }
    // 获取起始点
    float GetStart() const { return (start_); }
    // 获取终止点
    float GetEnd() const { return (end_); }
    // 获取总步数, 恒大于等于1
    uint32_t GetStep() const { return (step_); }
    // 获取当前步数
    uint32_t GetCount() const { return (count_); }
    // 获取插值进度, 取值范围[0, 1]
    float GetProgress() const { return (static_cast<float>(count_) / static_cast<float>(step_)); }
    // 获取插值是否结束
    bool IsFinished() const { return (count_ >= step_); }
    // 获取曲线类型
    Type GetType() const { return (type_); }

    // 写变量

    // 设定起始点, 会据此重新计算当前插值结果
    void SetStart(float start);
    // 设定终止点, 会据此重新计算当前插值结果
    void SetEnd(float end);
    // 直接跳转到第count步, 会据此重新计算当前插值结果, 超界时自动限幅
    void SetCount(uint32_t count);
    // 设定曲线类型, 会据此重新计算当前插值结果
    void SetType(Type type);
    // 设定S型曲线陡峭程度
    void SetSigmoidSteepness(float steepness);

    // 每次调用前进一步, 并返回当前插值结果, 已结束时保持终止点
    float Update();

protected:
    // 常量

    // S型曲线的陡峭程度, 越大越接近阶跃, 越小越接近线性, 建议取值[4, 20]
    float sigmoid_steepness_ = 8.0f;

    // 内部变量

    // 曲线类型
    Type type_ = Type::kLinear;
    // 起始点
    float start_ = 0.0f;
    // 终止点
    float end_ = 0.0f;
    // 总步数, 恒大于等于1
    uint32_t step_ = 1;
    // 当前步数
    uint32_t count_ = 0;
    // 当前插值结果
    float now_ = 0.0f;
    // 上一次插值结果
    float prev_ = 0.0f;

    // 内部函数

    // 曲线映射, 将归一化进度按曲线类型映射为输出进度, 超界时自动限幅
    float Ease(float progress, Type type);

    // 由当前步数重新计算插值结果
    void Calculate();
};

} // namespace Algorithm

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) NEUQ-RoboPioneers **************************/
