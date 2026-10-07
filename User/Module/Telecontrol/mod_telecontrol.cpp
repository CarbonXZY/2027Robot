/**
 * @file mod_telecontrol.cpp
 * @brief 遥控接收（CRSF）解析 + 上行回传，与上位机 R2_PC 的 telecontrol_bridge 配对
 *
 * 职责：
 *   收：UART7 上的 CRSF 原始字节交给 Class_CRSF 解包成摇杆/开关/链路状态
 *   发：每 1ms 把解包结果刷进 tx_buffer_，由通信中间件打包回传上位机
 *
 * 遥控帧（id 见 Middleware/mid_config.h 的 CommFrameId::kTelecontrol）在本类 Init() 里自注册。
 * 只上行，所以 Register() 的 rx 传 nullptr，
 * 注意 Register() 的 tx/rx 是站在本板视角，与结构体名的「本板视角」是一致的。
 */

#include "mod_telecontrol.h"

#include <cstddef>
#include <cstdint>

#include "Communication_Interface.hpp"
#include "mid_config.h"

namespace Module
{

void Telecontrol::Init(const Parameters &parameters)
{
    crsf_.Init(parameters.crsf_uart);

    Middleware::g_usb_communication_interface.Register(
        static_cast<uint8_t>(Middleware::CommFrameId::kTelecontrol),
        &tx_buffer_, nullptr,
        static_cast<uint8_t>(sizeof(tx_buffer_)), 0);
}

void Telecontrol::UartRxCallback(uint8_t *data, uint16_t length)
{
    crsf_.CRSF_UART_RxCpltCallback(data, length);
}

void Telecontrol::TimCalculatePeriodElapsedCallback()
{
    tx_buffer_.right_x = crsf_.Get_Right_X();
    tx_buffer_.right_y = crsf_.Get_Right_Y();
    tx_buffer_.left_x  = crsf_.Get_Left_X();
    tx_buffer_.left_y  = crsf_.Get_Left_Y();
    tx_buffer_.s1      = crsf_.Get_S1();

    // 开关枚举值与上位机 RcState.msg 的 LOW/MIDDLE/HIGH 一致，直接转字节
    tx_buffer_.sa = static_cast<uint8_t>(crsf_.Get_SA());
    tx_buffer_.sb = static_cast<uint8_t>(crsf_.Get_SB());
    tx_buffer_.sc = static_cast<uint8_t>(crsf_.Get_SC());
    tx_buffer_.sd = static_cast<uint8_t>(crsf_.Get_SD());
    tx_buffer_.se = static_cast<uint8_t>(crsf_.Get_SE());

    tx_buffer_.rssi         = crsf_.Get_RSSI();
    tx_buffer_.link_quality = crsf_.Get_LinkQuality();
    tx_buffer_.snr          = crsf_.Get_SNR();
    tx_buffer_.failsafe     = crsf_.Get_Failsafe() ? 1U : 0U;
}

void Telecontrol::Tim100msAlivePeriodElapsedCallback()
{
    crsf_.TIM1msMod50_Alive_PeriodElapsedCallback();
}

} // namespace Module
