/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_threadx.h
 * @author  MCD Application Team
 * @brief   ThreadX applicative header file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_THREADX_H__
#define __APP_THREADX_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_api.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
TX_QUEUE MsgQueueOne;
TX_QUEUE MsgQueueTwo;
TX_THREAD MsgReceiverThread;

/* Rx/TX flag */
#define RX_NEW_RECEIVED_DATA 0x01
#define TX_NEW_TRANSMITTED_DATA 0x02

TX_EVENT_FLAGS_GROUP EventFlag;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define APP_QUEUE_SIZE 64
#define APP_STACK_SIZE 512
#define RECEIVER_THREAD_PRIO 30
#define RECEIVER_THREAD_PREEMPTION_THRESHOLD RECEIVER_THREAD_PRIO
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
UINT App_ThreadX_Init(VOID *memory_ptr);
void MX_ThreadX_Init(void);
/* USER CODE BEGIN EFP */
static VOID App_Delay(ULONG Delay);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /* __APP_THREADX_H__ */
