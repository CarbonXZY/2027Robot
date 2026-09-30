
#ifndef TSK_CONFIG_AND_CALLBACK_H
#define TSK_CONFIG_AND_CALLBACK_H


/* Includes ------------------------------------------------------------------*/

#ifdef STM32H723xx
#include "stm32h7xx_hal.h"
#endif
#ifdef STM32F407xx
#include "stm32f4xx_hal.h"
#endif
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/


#ifdef __cplusplus
extern "C" {
#endif

void TaskInit();
void TaskLoop();

#ifdef __cplusplus
}
#endif


#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
