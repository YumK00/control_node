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
#include "motor.h"                 // Motor_GetSpeed()
#include <stdbool.h>               // bool / true / false
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
/* 心跳超时门限：对端心跳周期 1s，3 倍时间收不到就判掉线 */
#define HB_TIMEOUT_MS   3000u

/* 电机命令超时：转着的时候 3 秒没有新命令就停车(网关掉线/CAN 线断了) */
#define MOTOR_CMD_TIMEOUT_MS   3000u

/* motorTask 的轮询周期。10ms 足够跟上下发的命令(命令本身是百毫秒级的)，
 * 又不会白占 CPU。以后要上闭环 PID，就把它当控制周期用。 */
#define MOTOR_TASK_PERIOD_MS     10u
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

osThreadId_t canTxHandle;
const osThreadAttr_t canTxTask_attributes = {
  .name = "canTxTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t motorHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 128 * 4,   /* 只调 Motor_SetSpeed()，不碰 I2C/printf，小栈够用 */
  .priority = (osPriority_t) osPriorityAboveNormal,   /* 必须高于 canRxTask */
};

/* ==================== 电机命令邮箱 ====================
 * canRxTask 写、motorTask 读，一写一读，没有第三方碰。
 *
 * 【为什么用共享变量而不是队列】
 * 目标速度是"最新值有效"，不是事件流：网关连发三条 SETSPEED，
 * 只关心最后一条。队列会把三条都排着，电机依次 50→60→70 抖三下。
 *
 * 【写入顺序不能反】先写 target 再写 cnt —— 这样读到新 cnt 的一方
 * 必然也看到了新的 target。反过来写的话，读到新 cnt 却可能还是旧 target。
 */
static volatile int16_t g_motor_target  = 0;   /* 目标速度 -100~+100 */
static volatile uint8_t g_motor_cmd_cnt = 0;   /* 命令序号，每来一条 +1(自然回绕) */
static volatile bool g_estop_istrue = false;   // false: 正常运行，true: 紧急停止

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
void canTxTask(void *argument);
void motorTask(void *argument);
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
  if(canRxHandle == NULL){Error_Handler();}
  canHeartHandle = osThreadNew(canHeartTask, NULL, &canHeartTask_attributes);
  if(canHeartHandle == NULL){Error_Handler();}
  canTxHandle = osThreadNew(canTxTask, NULL, &canTxTask_attributes);
  if(canTxHandle == NULL){Error_Handler();}
  motorHandle = osThreadNew(motorTask, NULL, &motorTask_attributes);
  if(motorHandle == NULL){Error_Handler();}
  /* 栈和 TCB 都是从 configTOTAL_HEAP_SIZE(8KB) 里分的，堆不够时
   * osThreadNew 返回 NULL，任务会【静默地永远不运行】，连报错都没有。
   * 所以创建完必须查一次。 */
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
 * @brief  B 状态上报任务：1Hz 把本地执行器状态发给网关(0x102)
 * @note   这个任务【只取快照、不采样】。以后电机控制逻辑做出来了，
 *         让它去更新 g_b_status，本任务一行都不用改 —— 和 canRxTask
 *         用队列隔离 ISR 是同一个思路：谁拥有数据谁负责维护。
 *
 *         现阶段还没有电机反馈(没编码器、没电流采样)，
 *         所以 motor 只能是"停着"(猜的)，pwm 则是直接从寄存器读回来的真值。
 */
void canTxTask(void *argument){
  uint8_t data[LEN_B_STATUS];

  for(;;){
    /* 向电机层要一次"当前速度"：符号是方向，绝对值是占空比，
     * 一次调用同时喂饱 B_STATUS 的 B0(状态) 和 B1(占空比)。 */
    int16_t spd = Motor_GetSpeed();

    B_Status_t b = {
      .motor = (spd > 0) ? MST_FWD : ((spd < 0) ? MST_REV : MST_IDLE),
      .pwm   = (uint8_t)((spd > 0) ? spd : -spd),   /* 取绝对值，B1 是 0~100 */
      .mode  = MODE_MANUAL,       /* 占位：模式切换(0x332)还没做 */
      .flags = 0,                 /* 占位：ST_COMM_OK 要等网关心跳判活做完再补 */
    };
    PROTO_PackBStatus(data, b.motor, b.pwm, b.mode, b.flags);
    CAN_Send(CAN_ID_B_STATUS, data, LEN_B_STATUS);
    vTaskDelay(pdMS_TO_TICKS(1000));     // 1 秒一次
  }
}


/**
 * @brief  电机任务：执行命令 + 断线保护
 * @note   为什么电机要独立一个任务，而不是写在 canRxTask 里：
 *
 *         ① 超时停车需要"定时醒来"。canRxTask 是纯事件驱动的
 *            (osMessageQueueGet 阻塞等帧)，没帧它根本不跑，
 *            天生回答不了"距离上一条命令过了多久"。
 *
 *         ② canRxTask 刷一次屏要在软件 I2C 上跑约 23ms。电机的事排
 *            在它后面，一条急停要等一屏刷完才执行，而且这 23ms 还会
 *            随显示内容变长变短 —— 等于把电机的响应时间交给 OLED 决定。
 *
 *         本任务优先级高于 canRxTask，所以能打断刷屏。canRxTask 忙等
 *         时虽然不主动让出 CPU，但 SysTick 每 1ms 会打断它，调度器
 *         就在那一刻切过来。
 *
 *         本任务是 Motor_SetSpeed() 的【唯一调用者】。两个任务同时改
 *         PWM 和方向脚会打乱换向时序(见 motor.c 里那段电流尖峰说明)。
 */
void motorTask(void *argument){
  for(;;){
    if(g_estop_istrue == false){
      Motor_SetSpeed(20);
    }
    else{
      Motor_SetSpeed(0);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  // uint8_t  last_cnt  = 0;      /* 上一次见过的命令序号 */
  // uint32_t last_tick = 0;      /* 上一次收到命令的时刻 */
  // int16_t  applied   = 0;      /* 已经下发给电机的速度 */

  // for(;;){
  //   /* ---- 1. 有新命令就执行 ---- */
  //   if (g_motor_cmd_cnt != last_cnt){      /* uint8 自然回绕，用 != 比较天生正确 */
  //     last_cnt  = g_motor_cmd_cnt;
  //     /* 每条命令都要刷新计时，不能只在"值变了"的时候刷：
  //      * 网关每 100ms 重发同一个速度时 target == applied，
  //      * 只比值的话计时永远不更新，超时保护会误判成"没命令"而停车。 */
  //     last_tick = osKernelGetTickCount();

  //     if (g_motor_target != applied){
  //       Motor_SetSpeed(g_motor_target);
  //       applied = g_motor_target;
  //     }
  //   }

  //   /* ---- 2. 断线保护：正在转，但超过 3 秒没有新命令 ---- */
  //   if (applied != 0 && (osKernelGetTickCount() - last_tick) > MOTOR_CMD_TIMEOUT_MS){
  //     Motor_SetSpeed(0);
  //     applied = 0;
  //   }

  //   osDelay(MOTOR_TASK_PERIOD_MS);
  // }
}

/**
 * @brief  CAN 接收显示任务：传感器数据显示 + A 心跳判活
 * @note   ---------------------
 *         |id len lost        |
 *         |ir ldr seq         |
           |1/0 1/0 12345      |
           |a ON/OFF  gw ON/OFF|
           ---------------------
 */
void canRxTask(void *argument){
  CanFrame frame;
  A_Sensor_t sensor;
  uint8_t last_seq  = 0;   // 上一帧序列号(丢帧基准)
  uint8_t seq_valid = 0;   // 0=还没收到第一帧，1=已能算丢帧

  uint32_t last_a_tick = 0;    // 上次收到 A 心跳的时刻
  uint8_t  a_online    = 0;    // A 是否在线

  uint32_t last_gw_tick = 0;    // 上次收到 gw 心跳的时刻
  uint8_t  gw_online    = 0;    // 网关是否在线
  for(;;){
    /* 最多等 500ms：有帧就处理，没帧也要醒来查超时 */
    if (osMessageQueueGet(canRxQueue, &frame, NULL, pdMS_TO_TICKS(500)) == osOK)
    {
      OLED_ShowHexNum(1, 1, frame.id, 3);    // 第1行：11位ID → 最多 3 位十六进制(0x7FF)
      OLED_ShowNum(1, 5, frame.len, 1);      //       DLC 0~8，1 位就够
      /*紧急制动0x000*/
      if(frame.id ==CAN_ID_ESTOP && frame.len == LEN_ESTOP){
        if(frame.data[0] == ESTOP_ACTION_CLEAR){
          g_estop_istrue = false;
          OLED_ShowString(3, 11,"      ");
        }
        else{
          g_estop_istrue = true;
          OLED_ShowString(3, 11,"ESTOP");
        }
        
      }
      /* ---- 0x101 传感器帧：解包成三个值 + 算丢帧 ---- */
      if (frame.id == CAN_ID_A_SENSOR && frame.len == LEN_A_SENSOR)
      {
        PROTO_UnpackASensor(frame.data, &sensor);

        OLED_ShowNum(2, 1,  sensor.ir,  4);    // 第 1-4 列
        OLED_ShowNum(2, 6,  sensor.ldr, 4);    // 第 6-9 列，第 5 列是间隔
        OLED_ShowNum(2, 11, sensor.seq, 3);    // 第 11-13 列，第 10 列是间隔
        OLED_ShowString(2, 14, "   ");         // 抹掉第 14-16 列可能残留的字符

        if (seq_valid)
        {
          /* 丢帧数 = 新seq - 旧seq - 1(正常相邻就是 0 丢帧) */
          OLED_ShowNum(1, 7, PROTO_SeqLost(sensor.seq, last_seq), 2);
        }
        else
        {
          OLED_ShowString(1, 7, "--");   // 第一帧，还没基准，无法算
        }
        last_seq  = sensor.seq;
        seq_valid = 1;
      }

      /* ---- 0x701 A 心跳帧：ID + DLC + 魔数 三者都对才算有效 ---- */
      if (frame.id == CAN_ID_A_HEARTBEAT && frame.len == LEN_HEARTBEAT
          && frame.data[0] == HB_MAGIC_A)
      {
        last_a_tick = osKernelGetTickCount();
        if (!a_online)
        {
          a_online = 1;
          OLED_ShowString(4, 1, "a ON ");
        }
      }
      /* ---- 0x703 网关心跳 ---- */
      else if (frame.id == CAN_ID_GW_HEARTBEAT && frame.len == LEN_HEARTBEAT
               && frame.data[0] == HB_MAGIC_GW)
      {
        last_gw_tick = osKernelGetTickCount();
        if (!gw_online)
        {
          gw_online = 1;
          OLED_ShowString(4, 7, "gw ON ");
        }
      }

      /* ---- 0x302 网关下发的电机命令 ----
       * 这里【只解包 + 写命令邮箱】，不碰电机。
       * 真正调 Motor_SetSpeed() 的是 motorTask，保证只有一个执行者。 */
      else if (frame.id == CAN_ID_CTRL_MOTOR_B && frame.len == LEN_CTRL_MOTOR)
      {
        Ctrl_Motor_t cmd;
        PROTO_UnpackCtrlMotor(frame.data, &cmd);

        /* 幂等：CAN 有自动重发，同一个 cid 只执行一次，
         * 否则一条"反转"命令会被执行两遍。 */
        static uint8_t last_cid = 0;
        if (cmd.cid != last_cid)
        {
          last_cid = cmd.cid;

          /* cur 带符号(方向)，mag 只取大小。两个都要留着：
           * 换向命令要用 mag 保持速度，SETSPEED 要用 cur 的符号保持方向。 */
          int16_t cur = Motor_GetSpeed();
          int16_t mag = (cur < 0) ? -cur : cur;

          int16_t want;
          switch (cmd.cmd)
          {
          case MOTOR_STOP: want = 0;                            break;
          case MOTOR_FWD:  want = (mag > 0) ?  mag :  100;      break;
          case MOTOR_REV:  want = (mag > 0) ? -mag : -100;      break;
          /* SETSPEED 只给大小，方向沿用当前的；现在停着没方向就默认正转 */
          default:         want = (cur < 0) ? -(int16_t)cmd.arg
                                            :  (int16_t)cmd.arg; break;
          }

          g_motor_target = want;     /* 先写值 */
          g_motor_cmd_cnt++;         /* 再写序号，顺序不能反 */
        }
      }

      /* ---- 0x103 网关状态：WiFi / MQTT / 本地缓存条数 ---- */
      else if (frame.id == CAN_ID_GW_STATUS && frame.len == LEN_GW_STATUS)
      {
        OLED_ShowNum(3, 1, frame.data[0], 1);                 // WiFi：0=断 1=连
        OLED_ShowNum(3, 3, frame.data[1], 1);                 // MQTT：0=断 1=连
        OLED_ShowNum(3, 5, PROTO_GetU16(frame.data + 2), 5);  // 缓存条数(大端)
      }
    }

    /* ---- 超时判活(必须在"收到帧"的 if 外，每轮都查) ----
     * 之前写在 if 里：A 掉线后总线安静下来，任务再也进不了 if，
     * OFF 永远不显示。"收到心跳"和"心跳超时"是互斥的两件事。 */
    if (a_online && (osKernelGetTickCount() - last_a_tick) > HB_TIMEOUT_MS)
    {
      a_online = 0;
      OLED_ShowString(4, 1, "a OFF");   // 阶段2 补发 ALM_COMM_LOST 报警帧
    }
    if (gw_online && (osKernelGetTickCount() - last_gw_tick) > HB_TIMEOUT_MS)
    {
      gw_online = 0;
      OLED_ShowString(4, 7, "gw OFF");
    }
  }
}

/* USER CODE END Application */
