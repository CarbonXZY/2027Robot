/**
 * @file drv_can.cpp
 * @author Lucy
 * @brief H723 FDCAN配置为经典CAN数据包
 * @version 0.1
 * @date 2024-10-02
 *
 * @copyright RoboPioneer (c) 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_can.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

namespace Driver
{

//定义FdcanManage结构体
FdcanManageObject g_fdcan1_manage_object = {};
FdcanManageObject g_fdcan2_manage_object = {};
FdcanManageObject g_fdcan3_manage_object = {};

// CAN通信发送缓冲区
uint8_t g_fdcan1_0x1ff_tx_data[8];
uint8_t g_fdcan1_0x200_tx_data[8];
uint8_t g_fdcan1_0x2ff_tx_data[8];
uint8_t g_fdcan1_0x1fe_tx_data[8];
uint8_t g_fdcan1_0x2fe_tx_data[8];
uint8_t g_fdcan1_0x3fe_tx_data[8];
uint8_t g_fdcan1_0x4fe_tx_data[8];

// 0x500 DJI从板发送数据缓冲区
uint8_t g_fdcan1_0x500_tx_data[8];

uint8_t g_fdcan2_0x1ff_tx_data[8];
uint8_t g_fdcan2_0x200_tx_data[8];
uint8_t g_fdcan2_0x2ff_tx_data[8];
uint8_t g_fdcan2_0x1fe_tx_data[8];
uint8_t g_fdcan2_0x2fe_tx_data[8];
uint8_t g_fdcan2_0x3fe_tx_data[8];
uint8_t g_fdcan2_0x4fe_tx_data[8];

uint8_t g_fdcan3_0x1ff_tx_data[8];
uint8_t g_fdcan3_0x200_tx_data[8];
uint8_t g_fdcan3_0x2ff_tx_data[8];
uint8_t g_fdcan3_0x1fe_tx_data[8];
uint8_t g_fdcan3_0x2fe_tx_data[8];
uint8_t g_fdcan3_0x3fe_tx_data[8];
uint8_t g_fdcan3_0x4fe_tx_data[8];

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
* @brief 初始化CAN总线,如需添加过滤器，参考备注
 *
 * @param hfdcan CAN编号
 * @param callback_function 处理回调函数
 */
void FdcanInit(FDCAN_HandleTypeDef *hfdcan, FdcanCallback callback_function)
{
    if (hfdcan->Instance == FDCAN1)
    {
        g_fdcan1_manage_object.fdcan_handler_ = hfdcan;
        g_fdcan1_manage_object.callback_function_ = callback_function;
        FdcanFilterMaskConfig(hfdcan, FdcanFilter(0) | kFdcanFifo0 | kFdcanStdId | kFdcanDataType, 0, 0);
        FdcanFilterMaskConfig(hfdcan, FdcanFilter(1) | kFdcanFifo1 | kFdcanExtId | kFdcanDataType, 0, 0);
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        g_fdcan2_manage_object.fdcan_handler_ = hfdcan;
        g_fdcan2_manage_object.callback_function_ = callback_function;
        FdcanFilterMaskConfig(hfdcan, FdcanFilter(0) | kFdcanFifo0 | kFdcanStdId | kFdcanDataType, 0, 0);
        FdcanFilterMaskConfig(hfdcan, FdcanFilter(1) | kFdcanFifo1 | kFdcanExtId | kFdcanDataType, 0, 0);
        //FdcanFilterMaskConfig(hfdcan, FdcanFilter(15) | kFdcanFifo1 | kFdcanStdId | kFdcanDataType, 0, 0);
    }
		else if (hfdcan->Instance == FDCAN3)
    {
        g_fdcan3_manage_object.fdcan_handler_ = hfdcan;
        g_fdcan3_manage_object.callback_function_ = callback_function;
        FdcanFilterMaskConfig(hfdcan, FdcanFilter(0) | kFdcanFifo0 | kFdcanStdId | kFdcanDataType, 0, 0);
        FdcanFilterMaskConfig(hfdcan, FdcanFilter(1) | kFdcanFifo1 | kFdcanExtId | kFdcanDataType, 0, 0);
    }

		HAL_FDCAN_Start(hfdcan);
}

/**
 * @brief 配置CAN的过滤器
 *
 * @param hfdcan CAN编号
 * @param object_para 编号 | FIFOx | ID类型 | 帧类型
 * @param id ID
 * @param mask_id 屏蔽位(0x3ff, 0x1fffffff)
 */
void FdcanFilterMaskConfig(FDCAN_HandleTypeDef *hfdcan, uint8_t object_para, uint32_t id, uint32_t mask_id)
{
    FDCAN_FilterTypeDef fdcan_filter_init_structure;

    //检测传参是否正确
    assert_param(hfdcan != NULL);

    //标准帧或拓展帧判断
    if((object_para & 0x02) == 0)
    {
        fdcan_filter_init_structure.IdType = FDCAN_STANDARD_ID;
    }
		else if((object_para & 0x02) != 0)
		{
				fdcan_filter_init_structure.IdType = FDCAN_EXTENDED_ID;
		}
    //设置滤波器编号
    fdcan_filter_init_structure.FilterIndex = object_para >> 3;//object_para >> 3
    //设置过滤器MASK模式
    fdcan_filter_init_structure.FilterType = FDCAN_FILTER_MASK;
    //设置FIFO
    if((object_para & 0x04) == 0)
    {
        fdcan_filter_init_structure.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    }
    else if((object_para & 0x04) != 0)
    {
        fdcan_filter_init_structure.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    }
    //设置过滤ID及掩码
    fdcan_filter_init_structure.FilterID1 = id;
    fdcan_filter_init_structure.FilterID2 = mask_id;

    //使用结构体进行初始化
    HAL_FDCAN_ConfigFilter(hfdcan, &fdcan_filter_init_structure);
    //启用全局过滤
    if((object_para & 0x01) == 0)
    {
        HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    }
    else if((object_para & 0x01) != 0)
    {
        HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, ENABLE, ENABLE);
    }
    //打开FIFO区的新消息通知
    if((object_para & 0x04) == 0)
    {
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }
    else if((object_para & 0x04) != 0)
    {
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    }
}

/**
 * @brief 发送数据帧
 *
 * @param hfdcan CAN编号
 * @param id ID
 * @param data 被发送的数据指针
 * @param fdcan_id_type 拓展ID标准ID选择变量，默认为标准ID
 * @return uint8_t 执行状态
 */
uint8_t FdcanSendData(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, FdcanIdType fdcan_id_type, uint8_t data_length)
{
    FDCAN_TxHeaderTypeDef tx_header;

    //检测传参是否正确
    assert_param(hfdcan != NULL);

    tx_header.Identifier = id;                          //ID号
		//标准拓展ID判断，默认为标准ID
		if(fdcan_id_type == FdcanIdType::kStandard)
		{
				tx_header.IdType = FDCAN_STANDARD_ID;			      //标准ID
		}
    else if(fdcan_id_type == FdcanIdType::kExtended)
		{
				tx_header.IdType = FDCAN_EXTENDED_ID;
		}
    tx_header.TxFrameType = FDCAN_DATA_FRAME;		        //数据帧
    switch(data_length)
    {
        case(1):
        {
          tx_header.DataLength = FDCAN_DLC_BYTES_1;						//数据长度

          break;
        }
        case(2):
        {
          tx_header.DataLength = FDCAN_DLC_BYTES_2;						//数据长度

          break;
        }
        case(3):
        {
          tx_header.DataLength = FDCAN_DLC_BYTES_3;						//数据长度

          break;
        }
        case(4):
        {
          tx_header.DataLength = FDCAN_DLC_BYTES_4;						//数据长度

          break;
        }
        case(5):
        {
          tx_header.DataLength = FDCAN_DLC_BYTES_5;						//数据长度

          break;
        }
        case(6):
        {
          tx_header.DataLength = FDCAN_DLC_BYTES_6;						//数据长度

          break;
        }
        case(7):
        {
          tx_header.DataLength = FDCAN_DLC_BYTES_7;						//数据长度

          break;
        }
        case(8):
        {
          tx_header.DataLength = FDCAN_DLC_BYTES_8;						//数据长度

          break;
        }
    }


    //以下是FDCAN相较于经典CAN配置有拓展的地方

    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;     //CAN发送错误提示（？）
		tx_header.BitRateSwitch = FDCAN_BRS_OFF;              //波特率切换关闭
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;               //经典CAN模式
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;    //不储存发送事件（？）
    tx_header.MessageMarker = 0;	                        //消息标记0（？）

    return (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, data));
}

/**
 * @brief CAN的TIM定时器中断发送回调函数
 *
 */
void Tim1msCanPeriodElapsedCallback()
{
		//如需使用拓展ID，切记第四个参数设为FdcanIdType::kExtended
		//FdcanSendData(&hfdcan1, 0x1ff, g_fdcan1_0x1ff_tx_data, FdcanIdType::kExtended);

    // CAN1电机
    // 0x200 管 ID 0x201~0x204（腿 4 关节），0x1ff 管 ID 0x205~0x208（底盘 4 轮），
    // 每台电机占两字节。C620 要求控制帧 ~1kHz 不能断，两帧都必须每毫秒都发，
    // 哪怕内容是 0。
    FdcanSendData(&hfdcan1, 0x200, g_fdcan1_0x200_tx_data);
    FdcanSendData(&hfdcan1, 0x1ff, g_fdcan1_0x1ff_tx_data);
    // FdcanSendData(&hfdcan1, 0x2ff, g_fdcan1_0x2ff_tx_data);

    // CAN2电机
    // FdcanSendData(&hfdcan2, 0x1ff, g_fdcan2_0x1ff_tx_data);
    // FdcanSendData(&hfdcan2, 0x200, g_fdcan2_0x200_tx_data);
    // FdcanSendData(&hfdcan2, 0x2ff, g_fdcan2_0x2ff_tx_data);

		// CAN3电机
    // FdcanSendData(&hfdcan3, 0x1ff, g_fdcan3_0x1ff_tx_data);
    // FdcanSendData(&hfdcan3, 0x200, g_fdcan3_0x200_tx_data);
    // FdcanSendData(&hfdcan3, 0x2ff, g_fdcan3_0x2ff_tx_data);
}

} // namespace Driver

/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hfdcan CAN编号
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    //选择回调函数
    if (hfdcan->Instance == FDCAN1)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &Driver::g_fdcan1_manage_object.rx_buffer_.Header, Driver::g_fdcan1_manage_object.rx_buffer_.Data);
				if(Driver::g_fdcan1_manage_object.callback_function_ != nullptr)
				{
						Driver::g_fdcan1_manage_object.callback_function_(&Driver::g_fdcan1_manage_object.rx_buffer_);
				}
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &Driver::g_fdcan2_manage_object.rx_buffer_.Header, Driver::g_fdcan2_manage_object.rx_buffer_.Data);
        if(Driver::g_fdcan2_manage_object.callback_function_ != nullptr)
				{
						Driver::g_fdcan2_manage_object.callback_function_(&Driver::g_fdcan2_manage_object.rx_buffer_);
				}
    }
		else if (hfdcan->Instance == FDCAN3)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &Driver::g_fdcan3_manage_object.rx_buffer_.Header, Driver::g_fdcan3_manage_object.rx_buffer_.Data);
        if(Driver::g_fdcan3_manage_object.callback_function_ != nullptr)
				{
						Driver::g_fdcan3_manage_object.callback_function_(&Driver::g_fdcan3_manage_object.rx_buffer_);
				}
    }
}

/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hfdcan CAN编号
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    //选择回调函数
    if (hfdcan->Instance == FDCAN1)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &Driver::g_fdcan1_manage_object.rx_buffer_.Header, Driver::g_fdcan1_manage_object.rx_buffer_.Data);
				if(Driver::g_fdcan1_manage_object.callback_function_ != nullptr)
				{
						Driver::g_fdcan1_manage_object.callback_function_(&Driver::g_fdcan1_manage_object.rx_buffer_);
				}

    }
    else if (hfdcan->Instance == FDCAN2)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &Driver::g_fdcan2_manage_object.rx_buffer_.Header, Driver::g_fdcan2_manage_object.rx_buffer_.Data);
        if(Driver::g_fdcan2_manage_object.callback_function_ != nullptr)
				{
						Driver::g_fdcan2_manage_object.callback_function_(&Driver::g_fdcan2_manage_object.rx_buffer_);
				}
    }
		else if (hfdcan->Instance == FDCAN3)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &Driver::g_fdcan3_manage_object.rx_buffer_.Header, Driver::g_fdcan3_manage_object.rx_buffer_.Data);
        if(Driver::g_fdcan3_manage_object.callback_function_ != nullptr)
				{
						Driver::g_fdcan3_manage_object.callback_function_(&Driver::g_fdcan3_manage_object.rx_buffer_);
				}
    }
}




/************************ COPYRIGHT(C) ROBOPIONEER **************************/
