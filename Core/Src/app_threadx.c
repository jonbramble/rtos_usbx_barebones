/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_threadx.c
 * @author  MCD Application Team
 * @brief   ThreadX applicative file
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

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void MsgReceiverThread_Entry(ULONG thread_input);
/* USER CODE END PFP */

/**
 * @brief  Application ThreadX Initialization.
 * @param memory_ptr: memory pointer
 * @retval int
 */
UINT App_ThreadX_Init(VOID *memory_ptr) {
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL *)memory_ptr;

  /* USER CODE BEGIN App_ThreadX_Init */
  (void)byte_pool;

  CHAR *pointer;

  /* Allocate the MsgQueueOne.  */
  if (tx_byte_allocate(byte_pool, (VOID **)&pointer,
                       APP_QUEUE_SIZE * sizeof(ULONG),
                       TX_NO_WAIT) != TX_SUCCESS) {
    ret = TX_POOL_ERROR;
  }

  // Create the queue
  if (tx_queue_create(&MsgQueueOne, "Message Queue One", TX_1_ULONG, pointer,
                      APP_QUEUE_SIZE * sizeof(ULONG)) != TX_SUCCESS) {
    ret = TX_QUEUE_ERROR;
  }

  /* Allocate the MsgQueueTwo.  */
  if (tx_byte_allocate(byte_pool, (VOID **)&pointer,
                       APP_QUEUE_SIZE * sizeof(ULONG),
                       TX_NO_WAIT) != TX_SUCCESS) {
    ret = TX_POOL_ERROR;
  }

  // Create the queue
  if (tx_queue_create(&MsgQueueTwo, "Message Queue TWO", TX_1_ULONG, pointer,
                      APP_QUEUE_SIZE * sizeof(ULONG)) != TX_SUCCESS) {
    ret = TX_QUEUE_ERROR;
  }

  /* Allocate the stack for MsgReceiverThread.  */
  if (tx_byte_allocate(byte_pool, (VOID **)&pointer, APP_STACK_SIZE,
                       TX_NO_WAIT) != TX_SUCCESS) {
    ret = TX_POOL_ERROR;
  }

  /* Create MsgReceiverThread.  */
  if (tx_thread_create(&MsgReceiverThread, "Message Queue Receiver Thread",
                       MsgReceiverThread_Entry, 0, pointer, APP_STACK_SIZE,
                       RECEIVER_THREAD_PRIO,
                       RECEIVER_THREAD_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE,
                       TX_AUTO_START) != TX_SUCCESS) {
    ret = TX_THREAD_ERROR;
  }

  /* USER CODE END App_ThreadX_Init */

  return ret;
}

/**
 * @brief  MX_ThreadX_Init
 * @param  None
 * @retval None
 */
void MX_ThreadX_Init(void) {
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */
void App_Delay(ULONG Delay) {
  ULONG initial_time = tx_time_get();
  while ((tx_time_get() - initial_time) < Delay)
    ;
}

void MsgReceiverThread_Entry(ULONG thread_input) {
  ULONG RMsg = 0;
  UINT status = 0;
  CHAR *name;
  ULONG enqueued;
  ULONG available_storage;
  TX_THREAD *first_suspended;
  ULONG suspended_count;
  TX_QUEUE *next_queue;

  CHAR TMsg;

  (void)thread_input;
  /* Infinite loop */
  while (1) {
    status =
        tx_queue_info_get(&MsgQueueOne, &name, &enqueued, &available_storage,
                          &first_suspended, &suspended_count, &next_queue);

    if (status != TX_SUCCESS) {
      Error_Handler();
    }

    if (enqueued > 0) {
      // read all of queue, look for line termination
      for (ULONG m = 0; m < enqueued; m++) {
        status = tx_queue_receive(&MsgQueueOne, &RMsg, TX_NO_WAIT);

        if (RMsg != '\n') {
          TMsg = (CHAR)RMsg;
          tx_queue_send(&MsgQueueTwo, &TMsg,
                        TX_NO_WAIT);  // TX_WAIT_FOREVER or not?

        } else {
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
          App_Delay(3);
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
          App_Delay(3);
        }
      }
    }
  }
}

/* USER CODE END 1 */
