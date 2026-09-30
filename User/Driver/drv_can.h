/**
 * @file drv_can.h
 * @author Lucy
 * @brief H723 FDCAN配置为经典CAN数据包
 * @version 0.1
 * @date 2024-10-02
 *
 * @copyright RoboPioneer (c) 2024
 *
 */

#ifndef DRV_CAN_H
#define DRV_CAN_H

/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx_hal.h"

/* CubeMX generated handles (global namespace) -------------------------------*/

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

namespace Driver
{

/* Exported macros -----------------------------------------------------------*/

// 滤波器编号
constexpr uint32_t FdcanFilter(uint32_t x)
{
    return x << 3;
}

// 接收队列
constexpr uint32_t kFdcanFifo0 = (0 << 2);
constexpr uint32_t kFdcanFifo1 = (1 << 2);

//标准帧或扩展帧
constexpr uint32_t kFdcanStdId = (0 << 1);
constexpr uint32_t kFdcanExtId = (1 << 1);

// 数据帧或遥控帧
constexpr uint32_t kFdcanDataType = (0 << 0);
constexpr uint32_t kFdcanRemoteType = (1 << 0);

/* Exported types ------------------------------------------------------------*/
/**
 * @brief ID类型
 *
 */
enum class FdcanIdType
{
    kStandard = 0,
    kExtended,
};

/**
 * @brief CAN接收的信息结构体
 *
 */
struct FdcanRxBuffer
{
    FDCAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
};

/**
 * @brief CAN通信接收回调函数数据类型
 *
 */
using FdcanCallback = void (*)(FdcanRxBuffer *);

/**
 * @brief CAN通信处理结构体
 *
 */
struct FdcanManageObject
{
    FDCAN_HandleTypeDef *fdcan_handler_;
    FdcanRxBuffer rx_buffer_;
    FdcanCallback callback_function_;
};

/* Exported variables ---------------------------------------------------------*/

extern FdcanManageObject g_fdcan1_manage_object;
extern FdcanManageObject g_fdcan2_manage_object;
extern FdcanManageObject g_fdcan3_manage_object;

extern uint8_t g_fdcan1_0x1ff_tx_data[];
extern uint8_t g_fdcan1_0x200_tx_data[];
extern uint8_t g_fdcan1_0x2ff_tx_data[];
extern uint8_t g_fdcan1_0x1fe_tx_data[];
extern uint8_t g_fdcan1_0x2fe_tx_data[];
extern uint8_t g_fdcan1_0x3fe_tx_data[];
extern uint8_t g_fdcan1_0x4fe_tx_data[];

extern uint8_t g_fdcan1_0x500_tx_data[];

extern uint8_t g_fdcan2_0x1ff_tx_data[];
extern uint8_t g_fdcan2_0x200_tx_data[];
extern uint8_t g_fdcan2_0x2ff_tx_data[];
extern uint8_t g_fdcan2_0x1fe_tx_data[];
extern uint8_t g_fdcan2_0x2fe_tx_data[];
extern uint8_t g_fdcan2_0x3fe_tx_data[];
extern uint8_t g_fdcan2_0x4fe_tx_data[];

extern uint8_t g_fdcan3_0x1ff_tx_data[];
extern uint8_t g_fdcan3_0x200_tx_data[];
extern uint8_t g_fdcan3_0x2ff_tx_data[];
extern uint8_t g_fdcan3_0x1fe_tx_data[];
extern uint8_t g_fdcan3_0x2fe_tx_data[];
extern uint8_t g_fdcan3_0x3fe_tx_data[];
extern uint8_t g_fdcan3_0x4fe_tx_data[];

/* Exported function declarations ---------------------------------------------*/

void FdcanInit(FDCAN_HandleTypeDef *hfdcan, FdcanCallback callback_function);

void FdcanFilterMaskConfig(FDCAN_HandleTypeDef *hfdcan, uint8_t object_para, uint32_t id, uint32_t mask_id);

uint8_t FdcanSendData(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, FdcanIdType fdcan_id_type = FdcanIdType::kStandard, uint8_t data_length = 8);

void Tim1msCanPeriodElapsedCallback();

} // namespace Driver

#endif

/************************ COPYRIGHT(C) ROBOPIONEER **************************/
