/**
* @file alg_cross_couple.h
* @author lyh
* @date 2026-10-19
* @brief 多电机交叉耦合同步控制器
*
*/
#ifndef ALG_CROSS_COUPLE_H
#define ALG_CROSS_COUPLE_H

#include <cstdint>
#include <array>

namespace Algorithm
{

/**
 * @brief 多电机交叉耦合同步控制器
 *
 * 平台无关的纯算法库  
 * 计算参与电机位置的平均值，再按各电机与平均值的偏差施加比例修正
 * 使各电机收敛到同一位置，用于同轴或并联驱动的多电机机构
 *
 * @tparam MaxMotorNum 最大电机数量(存储容量), 实际参与数量由 SetMotorNum 决定
 */
template <uint32_t MaxMotorNum = 4>
class CrossCouple
{
public:
    //

    void Init(uint32_t motor_num)
    {
        SetMotorNum(motor_num);
    }

    /**
     * @brief 设置实际参与的电机数量
     * @param num 实际数量, 会被钳制到 [1, MaxMotorNum]
     */
    void SetMotorNum(uint32_t num)
    {
        motor_num_ = num < 1 ? 1 : (num > MaxMotorNum ? MaxMotorNum : num);
        // 未参与计算的槽位输出清零, 避免数量调小后残留旧的补偿量
        for (uint32_t i = motor_num_; i < MaxMotorNum; i++)
        {
            output_[i] = 0.0f;
        }
    }

    /**
     * @brief 获取实际参与的电机数量
     */
    uint32_t GetMotorNum() const { return motor_num_; }

    /**
     * @brief 设置耦合刚度
     * @param kp 位置偏差到修正量的比例系数
     */
    void SetKp(float kp) { kp_ = kp; }

    /**
     * @brief 设置各电机的当前实际位置
     * @param now 各电机位置，需处于同一输出坐标系
     */
    void SetNow(const std::array<float, MaxMotorNum>& now) { now_ = now; }

    /**
     * @brief 设置各电机的当前实际位置
     * @param now 当前值
     * @param id 编号
     */
    void SetNow(float now, uint32_t id) { now_[id] = now; }

    /**
     * @brief 获取各电机位置的平均值
     */
    float GetNowAverage() const { return now_avg_; }

    /**
     * @brief 获取耦合刚度
     */
    float GetKp() const { return kp_; }

    /**
     * @brief 获取当前输入的各电机位置
     */
    const std::array<float, MaxMotorNum>& GetNow() const { return now_; }

    /**
     * @brief 获取各电机的同步修正量，应叠加到电机指令上
     */
    const std::array<float, MaxMotorNum>& GetOutput() const { return output_; }

    /**
     * @brief 计算当前值的平均值
     */
    void CalculateNowAverage();

    /**
     * @brief 根据当前位置计算平均位置与各电机的同步修正量
     */
    void Update();

protected:
    //

    uint32_t motor_num_ = MaxMotorNum;            // 实际参与的电机数量
    std::array<float, MaxMotorNum> now_ = {};     // 各电机当前实际位置
    float kp_ = 0.0f;                             // 耦合刚度
    float now_avg_ = 0.0f;                        // 参与电机的平均位置
    std::array<float, MaxMotorNum> output_ = {};  // 各电机的同步修正量
};

/**
 * @brief 计算当前值的平均值
 */
template <uint32_t MaxMotorNum>
void CrossCouple<MaxMotorNum>::CalculateNowAverage()
{
    float pos_sum = 0;
    for (uint32_t i = 0; i < motor_num_; i++)
    {
        pos_sum += now_[i];
    }
    now_avg_ = pos_sum / motor_num_;
}

/**
 * @brief 计算平均位置与各电机的同步修正量
 */
template <uint32_t MaxMotorNum>
void CrossCouple<MaxMotorNum>::Update()
{
    // 计算平均位置
    CalculateNowAverage();

    // 计算各项值, 未参与的槽位输出保持为 0
    for (uint32_t i = 0; i < MaxMotorNum; i++)
    {
        if (i < motor_num_)
        {
            output_[i] = kp_ * (now_avg_ - now_[i]);
        }
        else
        {
            output_[i] = 0.0f;
        }
    }
}

}

#endif
