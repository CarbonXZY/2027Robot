/**
 * @file mod_telecontrol.h
 * @brief 遥控接收（CRSF）解析 + 上行回传，与上位机 R2_PC 的 telecontrol_bridge 配对
 *
 * 一帧，id=5，与上位机 URDF 的 frame_id 默认值一致（robot.urdf: <param name="frame_id">5</param>）：
 *   上行（本板 → 上位机）[id=5][14 字段]  29B
 *   下行无（上位机 TelecontrolSystem 是 type="sensor"，只收不发）
 *
 * 结构体名按「本板视角」：Tx = 本板发出去的，对应上位机 telecontrol_bridge.hpp 的
 * TelecontrolRx。接收机接 UART7（420000 8N1），原始字节交给 Class_CRSF 解包。
 */

#ifndef MOD_TELECONTROL_H
#define MOD_TELECONTROL_H

#include <cstdint>

#include "dvc_crsf.h"

namespace Module
{

#pragma pack(push, 1)
/**
 * @brief 遥控上行结构体
 *
 * 线上方向：本板 → 上位机（上位机 telecontrol_bridge.hpp 的 TelecontrolRx，29B）
 * 所有 float 排在前面，避免 pack(1) 下出现非对齐 float 访问
 */
struct TelecontrolTx
{
    float   right_x;
    float   right_y;
    float   left_x;
    float   left_y;
    float   s1;

    uint8_t sa;
    uint8_t sb;
    uint8_t sc;
    uint8_t sd;
    uint8_t se;

    uint8_t rssi;
    uint8_t link_quality;
    int8_t  snr;
    uint8_t failsafe;
};
#pragma pack(pop)

static_assert(sizeof(TelecontrolTx) == 29, "wire layout must match R2_PC telecontrol_bridge.hpp");

class Telecontrol
{
public:
    /**
     * @brief 遥控参数（照 R1 集中配置的做法）
     */
    struct Parameters
    {
        // 接收机所在串口，CRSF 固定 420000 8N1
        UART_HandleTypeDef *crsf_uart = &huart7;
    };

    /**
     * @brief 绑定接收机串口，并把 id=5 的上行帧注册进通信中间件
     */
    void Init(const Parameters &parameters);

    /**
     * @brief 1ms：把最近一帧遥控数据刷进上行结构体
     */
    void TimCalculatePeriodElapsedCallback();

    /**
     * @brief 100ms：接收机判活，掉线则置 failsafe
     */
    void Tim100msAlivePeriodElapsedCallback();

    /**
     * @brief 接收机串口原始字节，透传给 CRSF 解析
     */
    void UartRxCallback(uint8_t *data, uint16_t length);

    // 上行：遥控状态，通信中间件直接读走
    TelecontrolTx tx_buffer_{};

private:
    Class_CRSF crsf_;
};

} // namespace Module

#endif
