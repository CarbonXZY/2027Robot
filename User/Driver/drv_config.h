#ifndef __DRV_CONFIG_H__
#define __DRV_CONFIG_H__
/*-----------------------------------------------------------------------------*/
// @brief 该文件包含了驱动层的配置选项
// @author Clover
// @date 2024-06-01
// @version 0.1
// @brief 配置选项
/*-----------------------------------------------------------------------------*/
#define STM32H7xx

#ifdef STM32H7xx
#include "stm32h7xx_hal.h"
#endif /* STM32H7xx */


#ifdef STM32H5xx
#include "stm32h5xx_hal.h"
#endif /* STM32H5xx */


#endif /* __DRV_CONFIG_H__ */