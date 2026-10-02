/**
 * @file dvc_crsf.cpp
 * @brief CRSF 协议解析实现 - RadioMaster Pocket
 */

#include "dvc_crsf.h"
#include "tsk_config_and_callback.h"
#include "drv_math.h"
#include <string.h>

/* ==================== CRC8 查找表 ==================== */
static const uint8_t CRSF_CRC8_TABLE[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

/* ==================== 公有方法 ==================== */

void Class_CRSF::Init(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
        UART_Manage_Object = &UART1_Manage_Object;
    else if (huart->Instance == USART2)
        UART_Manage_Object = &UART2_Manage_Object;
    else if (huart->Instance == USART3)
        UART_Manage_Object = &UART3_Manage_Object;
    else if (huart->Instance == UART4)
        UART_Manage_Object = &UART4_Manage_Object;
    else if (huart->Instance == UART5)
        UART_Manage_Object = &UART5_Manage_Object;
    else if (huart->Instance == USART6)
        UART_Manage_Object = &UART6_Manage_Object;
    else if (huart->Instance == UART7)
        UART_Manage_Object = &UART7_Manage_Object;
    else if (huart->Instance == UART8)
        UART_Manage_Object = &UART8_Manage_Object;

    Reset_Control_Data();
}

void Class_CRSF::Reset_Control_Data()
{
    memset(&Now_UART_Rx_Data, 0, sizeof(Now_UART_Rx_Data));
    memset(&Pre_UART_Rx_Data, 0, sizeof(Pre_UART_Rx_Data));
    memset(&Rx_Data, 0, sizeof(Rx_Data));
    memset(&Pre_Rx_Data, 0, sizeof(Pre_Rx_Data));

    // 默认开关低档
    Rx_Data.SA = CRSF_SWITCH_LOW;
    Rx_Data.SB = CRSF_SWITCH_LOW;
    Rx_Data.SC = CRSF_SWITCH_LOW;
    Rx_Data.SD = CRSF_SWITCH_LOW;
    Rx_Data.SE = CRSF_SWITCH_LOW;
    Rx_Data.Failsafe = false;
}

void Class_CRSF::Invalidate_Control_Data()
{
    memset(&Rx_Data, 0, sizeof(Rx_Data));
    CRSF_Update_Status = CRSF_Status_DisUpdate;
    Rx_Data.Failsafe = true;
}

/* ==================== 协议解析核心函数 ==================== */

uint8_t Class_CRSF::Calculate_CRC8(uint8_t *data, uint8_t length)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++)
        crc = CRSF_CRC8_TABLE[crc ^ data[i]];
    return crc;
}

bool Class_CRSF::Validate_CRC8(uint8_t *frame)
{
    uint8_t length = frame[1];
    if (length < 2 || length > 62) return false;
    uint8_t crc_calc = Calculate_CRC8(&frame[2], length - 1);
    uint8_t crc_recv = frame[1 + length];
    return (crc_calc == crc_recv);
}

uint16_t Class_CRSF::Extract_Channel(uint8_t *payload, uint8_t channel_index)
{
    uint32_t bit_index = (uint32_t)channel_index * 11U;
    uint32_t byte_index = bit_index / 8U;
    uint32_t shift = bit_index % 8U;
    uint32_t packed = (uint32_t)payload[byte_index] |
                     ((uint32_t)payload[byte_index + 1U] << 8U) |
                     ((uint32_t)payload[byte_index + 2U] << 16U);
    return (uint16_t)((packed >> shift) & 0x07FFU);
}

/* ==================== 数据归一化与解码 ==================== */

float Class_CRSF::Normalize_Stick(uint16_t value)
{
    if (value < CRSF_CHANNEL_MIN || value > CRSF_CHANNEL_MAX)
        return 0.0f;
    float normalized = ((float)value - CRSF_CHANNEL_MID) / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) * 2.0f;
    Driver::MathConstrain(&normalized, -1.0f, 1.0f);
    return normalized;   // 直接返回 -1 ~ +1
}

float Class_CRSF::Normalize_Slider(uint16_t value)
{
    if (value < CRSF_CHANNEL_MIN || value > CRSF_CHANNEL_MAX)
        return 0.0f;
    float normalized = ((float)value - CRSF_CHANNEL_MIN) / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) * 2.0f - 1.0f;
    Driver::MathConstrain(&normalized, -1.0f, 1.0f);
    return normalized * 100.0f;   // 保持 -100 ~ +100
}

Enum_CRSF_Switch_Pos Class_CRSF::Decode_Switch(uint16_t value)
{
    if (value < CRSF_SWITCH_LOW_THRESHOLD)
        return CRSF_SWITCH_LOW;
    else if (value < CRSF_SWITCH_MID_THRESHOLD)
        return CRSF_SWITCH_MIDDLE;
    else
        return CRSF_SWITCH_HIGH;
}

void Class_CRSF::Judge_Update()
{
    // 仅检测摇杆和 SA~SD 的变化（SE 未加入，可根据需要添加）
    if (Pre_Rx_Data.Right_X == Rx_Data.Right_X &&
        Pre_Rx_Data.Right_Y == Rx_Data.Right_Y &&
        Pre_Rx_Data.Left_X  == Rx_Data.Left_X  &&
        Pre_Rx_Data.Left_Y  == Rx_Data.Left_Y  &&
        Pre_Rx_Data.SA == Rx_Data.SA &&
        Pre_Rx_Data.SB == Rx_Data.SB &&
        Pre_Rx_Data.SC == Rx_Data.SC &&
        Pre_Rx_Data.SD == Rx_Data.SD)
    {
        CRSF_Update_Status = CRSF_Status_DisUpdate;
    }
    else
    {
        CRSF_Update_Status = CRSF_Status_Update;
    }
}

/* ==================== 帧处理 ==================== */

bool Class_CRSF::Decode_CRSF_Frame(uint8_t *Rx_Buffer, uint16_t Length)
{
    int latest_valid_offset = -1;
    for (uint16_t offset = 0; offset + 2 <= Length; offset++)
    {
        uint8_t *frame = &Rx_Buffer[offset];
        if (frame[0] != CRSF_FRAME_HEADER) continue;
        uint8_t frame_length = frame[1];
        if (frame_length < 2 || frame_length > 62) continue;
        if (offset + 1 + frame_length > Length) continue;
        if (!Validate_CRC8(frame)) continue;
        latest_valid_offset = (int)offset;
    }
    if (latest_valid_offset < 0) return false;
    memcpy(&Now_UART_Rx_Data, &Rx_Buffer[latest_valid_offset], sizeof(Now_UART_Rx_Data));
    return true;
}

/**
 * @brief 处理 RC 数据帧：提取通道 1~10
 *        摇杆输出 -1~+1，开关三档，滑块 S1 输出 -100~+100
 */
void Class_CRSF::Process_CRSF_Data()
{
    if (Now_UART_Rx_Data.Type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
    {
        // --- 摇杆：通道 1~4 ---
        uint16_t ch_rx = Extract_Channel(Now_UART_Rx_Data.Payload, CRSF_CH_RIGHT_X - 1);
        uint16_t ch_ry = Extract_Channel(Now_UART_Rx_Data.Payload, CRSF_CH_RIGHT_Y - 1);
        uint16_t ch_ly = Extract_Channel(Now_UART_Rx_Data.Payload, CRSF_CH_LEFT_Y  - 1);
        uint16_t ch_lx = Extract_Channel(Now_UART_Rx_Data.Payload, CRSF_CH_LEFT_X  - 1);
        Rx_Data.Right_X = Normalize_Stick(ch_rx);
        Rx_Data.Right_Y = Normalize_Stick(ch_ry);
        Rx_Data.Left_Y  = Normalize_Stick(ch_ly);
        Rx_Data.Left_X  = Normalize_Stick(ch_lx);

        // --- 开关：通道 5~9 ---
        uint16_t ch_sa = Extract_Channel(Now_UART_Rx_Data.Payload, CRSF_CH_SA - 1);
        uint16_t ch_sb = Extract_Channel(Now_UART_Rx_Data.Payload, CRSF_CH_SB - 1);
        uint16_t ch_sc = Extract_Channel(Now_UART_Rx_Data.Payload, CRSF_CH_SC - 1);
        uint16_t ch_sd = Extract_Channel(Now_UART_Rx_Data.Payload, CRSF_CH_SD - 1);
        uint16_t ch_se = Extract_Channel(Now_UART_Rx_Data.Payload, CRSF_CH_SE - 1);
        Rx_Data.SA = Decode_Switch(ch_sa);
        Rx_Data.SB = Decode_Switch(ch_sb);   // 三档，中间值识别为 MIDDLE
        Rx_Data.SC = Decode_Switch(ch_sc);   // 三档，中间值识别为 MIDDLE
        Rx_Data.SD = Decode_Switch(ch_sd);
        Rx_Data.SE = Decode_Switch(ch_se);

        // --- 滑块：通道 10 ---
        uint16_t ch_s1 = Extract_Channel(Now_UART_Rx_Data.Payload, CRSF_CH_S1 - 1);
        Rx_Data.S1 = Normalize_Slider(ch_s1);   // -100 ~ +100

        Rx_Data.Failsafe = false;
    }
    else if (Now_UART_Rx_Data.Type == CRSF_FRAMETYPE_LINK_STATISTICS)
    {
        if (Now_UART_Rx_Data.Length >= 11)
        {
            Rx_Data.RSSI = Now_UART_Rx_Data.Payload[0];
            Rx_Data.LinkQuality = Now_UART_Rx_Data.Payload[2];
            Rx_Data.SNR = (int8_t)Now_UART_Rx_Data.Payload[3];
        }
    }
    // 心跳包忽略
}

/* ==================== 回调函数 ==================== */

void Class_CRSF::CRSF_UART_RxCpltCallback(uint8_t *Rx_Buffer, uint16_t Length)
{
    if (!Decode_CRSF_Frame(Rx_Buffer, Length))
    {
        Error_Cnt++;
        return;
    }
    Process_CRSF_Data();
    Judge_Update();
    CRSF_Flag++;
    Pre_UART_Rx_Data = Now_UART_Rx_Data;
    Pre_Rx_Data = Rx_Data;
}

void Class_CRSF::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    if (CRSF_Flag == Pre_CRSF_Flag)
    {
        CRSF_Status = CRSF_Status_DISABLE;
        Offline_Cnt++;
        Invalidate_Control_Data();
    }
    else
    {
        CRSF_Status = CRSF_Status_ENABLE;
    }
    Pre_CRSF_Flag = CRSF_Flag;
}