/**
 * @file dvc_crsf.h
 * @brief CRSF 协议解析模块 - RadioMaster Pocket
 * @note  
 */

#ifndef DVC_CRSF_H
#define DVC_CRSF_H

#include "drv_uart.h"
#include <cstdint>

#ifdef __cplusplus

/* ==================== 通道定义（按遥控器实际顺序 1~10） ==================== */
#define CRSF_CH_RIGHT_X     (1U)    // 通道1：右摇杆 X
#define CRSF_CH_RIGHT_Y     (2U)    // 通道2：右摇杆 Y
#define CRSF_CH_LEFT_Y      (3U)    // 通道3：左摇杆 Y
#define CRSF_CH_LEFT_X      (4U)    // 通道4：左摇杆 X
#define CRSF_CH_SA          (5U)    // 通道5：开关 A
#define CRSF_CH_SB          (6U)    // 通道6：开关 B（三档）
#define CRSF_CH_SC          (7U)    // 通道7：开关 C（三档）
#define CRSF_CH_SD          (8U)    // 通道8：开关 D
#define CRSF_CH_SE          (9U)    // 通道9：开关 E
#define CRSF_CH_S1          (10U)   // 通道10：滑块 S1

/* ==================== CRSF 协议常量 ==================== */
#define CRSF_FRAME_HEADER           (0xC8U)
#define CRSF_MAX_CHANNELS           (16U)      // 目前只用到前 10 个
#define CRSF_FRAME_MIN_LENGTH       (4U)
#define CRSF_FRAME_MAX_LENGTH       (64U)

/* ==================== 帧类型 ==================== */
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED   (0x16U)
#define CRSF_FRAMETYPE_LINK_STATISTICS      (0x14U)
#define CRSF_FRAMETYPE_HEARTBEAT            (0x0BU)

/* ==================== 通道值范围 ==================== */
#define CRSF_CHANNEL_MIN        (172U)
#define CRSF_CHANNEL_MID        (992U)
#define CRSF_CHANNEL_MAX        (1811U)

/* ==================== 开关阈值（动态计算） ==================== */
#define CRSF_SWITCH_LOW_THRESHOLD   ((CRSF_CHANNEL_MIN + CRSF_CHANNEL_MID) / 2)   // ~582
#define CRSF_SWITCH_MID_THRESHOLD   ((CRSF_CHANNEL_MID + CRSF_CHANNEL_MAX) / 2)   // ~1401

/* ==================== 状态枚举 ==================== */
enum Enum_CRSF_Status
{
    CRSF_Status_DISABLE = 0,
    CRSF_Status_ENABLE,
};

enum Enum_CRSF_Update_Status
{
    CRSF_Status_DisUpdate = 0,
    CRSF_Status_Update,
};

enum Enum_CRSF_Switch_Pos
{
    CRSF_SWITCH_LOW = 0,
    CRSF_SWITCH_MIDDLE = 1,
    CRSF_SWITCH_HIGH = 2,
};

/* ==================== 解析结果结构体 ==================== */
struct Struct_CRSF_Rx_Data
{
    // 摇杆（-1 ~ +1）
    float Right_X;
    float Right_Y;
    float Left_X;
    float Left_Y;

    // 开关
    Enum_CRSF_Switch_Pos SA;
    Enum_CRSF_Switch_Pos SB;   // 三档
    Enum_CRSF_Switch_Pos SC;   // 三档
    Enum_CRSF_Switch_Pos SD;
    Enum_CRSF_Switch_Pos SE;

    // 滑块（-100 ~ +100）
    float S1;

    // 原始通道值
    uint16_t Channel[CRSF_MAX_CHANNELS];

    // 链路状态
    uint8_t RSSI;
    uint8_t LinkQuality;
    int8_t  SNR;
    bool    Failsafe;
};

/* ==================== UART 接收帧结构 ==================== */
#pragma pack(push, 1)
struct Struct_CRSF_UART_Rx_Data
{
    uint8_t Header;
    uint8_t Length;
    uint8_t Type;
    uint8_t Payload[60];
    uint8_t CRC8;
};
#pragma pack(pop)

/* ==================== CRSF 协议解析类 ==================== */
class Class_CRSF
{
public:
    void Init(UART_HandleTypeDef *huart);
    void CRSF_UART_RxCpltCallback(uint8_t *Rx_Buffer, uint16_t Length);
    void TIM1msMod50_Alive_PeriodElapsedCallback();

    // 数据获取接口
    inline Enum_CRSF_Status Get_Status() { return CRSF_Status; }
    inline Enum_CRSF_Update_Status Get_Update_Status() { return CRSF_Update_Status; }

    inline float Get_Right_X() { return Rx_Data.Right_X; }
    inline float Get_Right_Y() { return Rx_Data.Right_Y; }
    inline float Get_Left_X()  { return Rx_Data.Left_X; }
    inline float Get_Left_Y()  { return Rx_Data.Left_Y; }

    inline Enum_CRSF_Switch_Pos Get_SA() { return Rx_Data.SA; }
    inline Enum_CRSF_Switch_Pos Get_SB() { return Rx_Data.SB; }
    inline Enum_CRSF_Switch_Pos Get_SC() { return Rx_Data.SC; }
    inline Enum_CRSF_Switch_Pos Get_SD() { return Rx_Data.SD; }
    inline Enum_CRSF_Switch_Pos Get_SE() { return Rx_Data.SE; }

    inline float Get_S1() { return Rx_Data.S1; }

    inline uint8_t Get_RSSI() { return Rx_Data.RSSI; }
    inline uint8_t Get_LinkQuality() { return Rx_Data.LinkQuality; }
    inline int8_t Get_SNR() { return Rx_Data.SNR; }
    inline bool Get_Failsafe() { return Rx_Data.Failsafe; }

protected:
    Struct_UART_Manage_Object *UART_Manage_Object = nullptr;

    Struct_CRSF_UART_Rx_Data Now_UART_Rx_Data;
    Struct_CRSF_UART_Rx_Data Pre_UART_Rx_Data;
    Struct_CRSF_Rx_Data Rx_Data;
    Struct_CRSF_Rx_Data Pre_Rx_Data;

    uint32_t CRSF_Flag = 0U;
    uint32_t Pre_CRSF_Flag = 0U;
    uint16_t Offline_Cnt = 0U;
    uint16_t Error_Cnt = 0U;

    Enum_CRSF_Status CRSF_Status = CRSF_Status_DISABLE;
    Enum_CRSF_Update_Status CRSF_Update_Status = CRSF_Status_DisUpdate;

    void Reset_Control_Data();
    void Invalidate_Control_Data();
    bool Decode_CRSF_Frame(uint8_t *Rx_Buffer, uint16_t Length);
    void Process_CRSF_Data();
    uint16_t Extract_Channel(uint8_t *payload, uint8_t channel_index);
    uint8_t Calculate_CRC8(uint8_t *data, uint8_t length);
    bool Validate_CRC8(uint8_t *frame);

    float Normalize_Stick(uint16_t value);
    float Normalize_Slider(uint16_t value);
    Enum_CRSF_Switch_Pos Decode_Switch(uint16_t value);
    void Judge_Update();
};

#endif
#endif