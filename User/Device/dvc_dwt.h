#ifndef DVC_SYSTEM_TIME_H
#define DVC_SYSTEM_TIME_H

#include "drv_config.h"

namespace Device
{

struct DwtTime
{
    uint32_t s;
    uint32_t ms;
    uint32_t us;
};

struct SoftTimer
{
    uint32_t start_time;   // 上次触发的时间点
    uint32_t expire_time;  // 设定的间隔时间
};

enum class ExpireStatus
{
    //只使用一次
    kOnce = 0,
    //循环使用
    kLoop = 1
};

//非阻塞式延时，用于判断是否超时
bool IsTimerExpiredUs(SoftTimer *timer, ExpireStatus status = ExpireStatus::kLoop);
void DwtInit();
uint32_t DwtGetCurrentTimeS();
uint32_t DwtGetCurrentTimeMs();
uint32_t DwtGetCurrentTimeUs();
void DwtUpdate();
float DwtGetDeltaT(uint32_t *cnt_last);
double DwtGetDeltaT64(uint32_t *cnt_last);

} // namespace Device

#endif
