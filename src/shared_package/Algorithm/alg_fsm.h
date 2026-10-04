/**
 * @file alg_fsm.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 有限自动机
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

#ifndef ALG_FSM_H
#define ALG_FSM_H

/* Includes ------------------------------------------------------------------*/

#include <cstdint>

namespace Algorithm
{

/* Exported constants --------------------------------------------------------*/

// 状态数量上限
constexpr uint8_t kStatusMax = 30;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 状态所处的阶段
 */
enum class StatusStage
{
    kDisable = 0,
    kEnable,
};

/**
 * @brief 状态结构体
 */
struct Status
{
    StatusStage status_stage;
    uint32_t count_time;
};

/**
 * @brief Reusable, 有限自动机核心, 一般有时间需求的则采用有限自动机
 * 使用时请继承->声明友元后使用
 */
class Fsm
{
public:
    Status status_[kStatusMax];

    void Init(uint8_t status_number, uint8_t now_status_serial = 0);

    inline uint8_t GetNowStatusSerial() const;

    inline void SetStatus(uint8_t next_status_serial);

    void TimCalculatePeriodElapsedCallback();

protected:
    // 状态数量
    uint8_t status_number_;

    // FSM当前状态
    uint8_t now_status_serial_ = 0;
};

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取FSM当前状态
 *
 * @return uint8_t FSM当前状态
 */
inline uint8_t Fsm::GetNowStatusSerial() const
{
    return now_status_serial_;
}

/**
 * @brief 设置状态改变
 *
 * @param next_status_serial 下一个状态
 */
inline void Fsm::SetStatus(uint8_t next_status_serial)
{
    // 失能当前状态, 计数器清零
    status_[now_status_serial_].status_stage = StatusStage::kDisable;
    status_[now_status_serial_].count_time = 0;

    // 转到下一个状态
    status_[next_status_serial].status_stage = StatusStage::kEnable;
    now_status_serial_ = next_status_serial;
}

} // namespace Algorithm

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
