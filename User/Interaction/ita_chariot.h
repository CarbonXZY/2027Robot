/**
 * @file ita_chariot.h
 * @brief 整车组合：把各机构绑到一起，对上层只暴露几个回调
 *
 * 周期分频（mod）写在 tsk 里，这里只负责「一拍该干什么」。
 */

#ifndef ITA_CHARIOT_H
#define ITA_CHARIOT_H

#include <cstdint>

#include "mod_leg.h"

class Chariot
{
public:
    Chariot();

    Module::Leg<4> leg_;

    // 上位机链路存活，由 1000ms 窗口刷新
    bool pc_is_alive_ = false;

    // 全车校准完成标志，1ms 周期刷新；未完成前不向上位机回传
    bool calibration_finished_ = false;

    void Init();

    // 底层 USB 接收回调：记一次存活并转发给通信中间件
    void McuRxCallback(uint8_t *data, uint16_t length);

    // 1ms：腿控制 + CAN 发送 + USB 上行
    void TimCalculatePeriodElapsedCallback();

    // 100ms：电机自身掉线检测
    void Tim100msAlivePeriodElapsedCallback();

    // 1000ms：上位机存活窗口
    void Tim1000msAlivePeriodElapsedCallback();

private:
    uint32_t alive_flag_ = 0;
    uint32_t pre_alive_flag_ = 0;
};

#endif
