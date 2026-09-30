#include "dvc_dwt.h"

namespace Device
{

DwtTime g_system_time;

static uint64_t g_accumulated_cycles = 0;
static uint32_t g_last_cycle_count = 0;

void DwtInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    g_last_cycle_count = DWT->CYCCNT;
    g_accumulated_cycles = 0;
}
uint32_t g_cpu_freq = 0;
void DwtUpdate(void)
{
    g_cpu_freq = HAL_RCC_GetSysClockFreq();
    if (g_cpu_freq == 0) return;
    uint32_t current_cnt = DWT->CYCCNT;

    // 利用无符号减法自动处理 32 位溢出
    uint32_t delta = current_cnt - g_last_cycle_count;
    g_accumulated_cycles += delta;
    g_last_cycle_count = current_cnt;

    // 正确的时间单位换算
    g_system_time.s  = (uint32_t)(g_accumulated_cycles / g_cpu_freq);
    g_system_time.ms = (uint32_t)(g_accumulated_cycles / (g_cpu_freq / 1000));
    g_system_time.us = (uint64_t)(g_accumulated_cycles / (g_cpu_freq / 1000000));
}

uint32_t DwtGetCurrentTimeS(void)
{
    DwtUpdate();
    return g_system_time.s;
}
uint32_t DwtGetCurrentTimeMs(void)
{
    DwtUpdate();
    return g_system_time.ms;
}
uint32_t DwtGetCurrentTimeUs(void)
{
    DwtUpdate();
    return g_system_time.us;
}

float DwtGetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(HAL_RCC_GetSysClockFreq()));
    *cnt_last = cnt_now;

    DwtUpdate();

    return dt;
}

double DwtGetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(HAL_RCC_GetSysClockFreq()));
    *cnt_last = cnt_now;

    DwtUpdate();

    return dt;
}


/**
 * @brief  非阻塞微秒级超时检查
 * @param  timer  : 定时器结构体指针，需提前设置 expire_time
 * @param  status : 模式选择
 * - kLoop: 循环模式。到期后自动补偿时间基准，消除累积误差。
 * - kOnce: 单次模式。到期后不再更新时间基准。
 * @return bool   : true  - 已达到或超过设定的超时时间
 * false - 仍在等待中
 */
bool IsTimerExpiredUs(SoftTimer *timer, ExpireStatus status)
{
    // 获取当前微秒级时间戳
    uint32_t current_time = DwtGetCurrentTimeUs();

    // 计算时间差（无符号减法自动处理 32 位翻转）
    uint32_t delta = current_time - timer->start_time;

    if (delta >= timer->expire_time)
    {
        // 更新起始时间点
        // 注意：这里使用 += 而不是 = current_time 可以补偿函数调用的微小抖动
        if (status == ExpireStatus::kLoop)
        {
            timer->start_time += timer->expire_time;
        }
        return true;
    }
    return false;
}

} // namespace Device
