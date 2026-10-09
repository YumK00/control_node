/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "stm32f103xb.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "stdio.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "adc_dma.h"               // ADC_DMA_GetValue()
#include "serial.h"                // Serial_SendData()
#include "interrupt.h"             // g_sample_enable 采样开关
#include "stm32f1xx_hal_gpio.h"    // HAL_GPIO_TogglePin()
#include <stdint.h>
#include "can.h"                   // CAN_Receive()
#include "protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
  uint16_t Infrared;   // 红外传感器 ch0
  uint16_t Light;      // 光敏传感器 ch1
}s_serialData;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */


osThreadId_t canRxHandle;
const osThreadAttr_t canRxTask_attributes = {
  .name = "canRxTask",
  .stack_size = 256 * 4,  
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t canHeartHandle;
const osThreadAttr_t canHeartTask_attributes = {
  .name = "canHeartTask",
  .stack_size = 256 * 4,  
  .priority = (osPriority_t) osPriorityLow7,
};


/* USER CODE END Variables */
/* Definitions for defaultTask */
/* 默认任务：LED 闪烁 */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void canRxTask(void *argument);
void canHeartTask(void *argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */




void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
   /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* CAN 接收队列：深度 8(能缓存8帧还没被显示)，每个元素是一整帧。 */
  canRxQueue = osMessageQueueNew(8, sizeof(CanFrame), NULL);
  if (canRxQueue == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  canRxHandle = osThreadNew(canRxTask, NULL, &canRxTask_attributes);
  canHeartHandle = osThreadNew(canHeartTask, NULL, &canHeartTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/* USER CODE END Header_StartDefaultTask */

void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);   // 翻转LED
    osDelay(2000);        // 睡2000ms，主动让出CPU
  }
  /* USER CODE END StartDefaultTask */
}


/* USER CODE BEGIN Application */


void canHeartTask(void *argument){
  uint8_t hb_cnt = 0;                    // 心跳计数，每发一次 +1(0~255 自动回绕)
  uint8_t hb[LEN_HEARTBEAT];             // 2 字节：魔数 + 计数

  for(;;){
    PROTO_PackHeartbeat(hb, HB_MAGIC_B, hb_cnt++);
    CAN_Send(CAN_ID_B_HEARTBEAT, hb, LEN_HEARTBEAT);   // 0x701，2 字节
    vTaskDelay(pdMS_TO_TICKS(1000));     // 1 秒一次
  }
}

/**
 * @brief  CAN 接收显示任务
 * @note   思路：死等队列。没帧就睡，来了帧立刻被唤醒，不空转也不漏帧。
 */
void canRxTask(void *argument){
  CanFrame frame;
  A_Sensor_t sensor;
  uint8_t last_seq  = 0;   // 上一帧序列号(丢帧基准)
  uint8_t seq_valid = 0;   // 0=还没收到第一帧，1=已能算丢帧

  OLED_ShowString(1, 1, "ID:");
  OLED_ShowString(2, 1, "LEN:");
  OLED_ShowString(3, 1, "D:");

  for(;;){
    if (osMessageQueueGet(canRxQueue, &frame, NULL, osWaitForever) != osOK)
    {
      continue;
    }

    OLED_ShowHexNum(1, 5, frame.id, 3);    // 11位ID → 最多 3 位十六进制(0x7FF)
    OLED_ShowNum(2, 6, frame.len, 1);      // DLC 0~8，1 位就够

    for (uint8_t i = 0; i < 8; i++)
    {
      uint8_t Line   = (i < 4) ? 3 : 4;
      uint8_t Column = 3 + (i % 4) * 2;

      if (i < frame.len)
      {
        OLED_ShowNum(Line, Column, frame.data[i], 2);
      }
      else
      {
        OLED_ShowString(Line, Column, "--");
      }
    }

    /* ---- 0x100 传感器帧：解包 + 算丢帧 ---- */
    if (frame.id == CAN_ID_A_SENSOR && frame.len == LEN_A_SENSOR)
    {
      PROTO_UnpackASensor(frame.data, &sensor);

      OLED_ShowString(2, 9, "LOST:");
      if (seq_valid)
      {
        /* 丢帧数 = 新seq - 旧seq - 1(正常相邻就是 0 丢帧) */
        OLED_ShowNum(2, 14, PROTO_SeqLost(sensor.seq, last_seq), 2);
      }
      else
      {
        OLED_ShowString(2, 14, "--");   // 第一帧，还没基准，无法算
      }
      last_seq  = sensor.seq;
      seq_valid = 1;
    }
  }
}

/* USER CODE END Application */
