# 执行控制节点（control_node）

> STM32F103 + FreeRTOS 的 CAN 执行节点，节点号 `NODE_B = 2`，负责把网关下发的电机命令翻译成 H 桥动作，并把自身状态与急停标志回传总线。
> 在整条链路里它是"手"：采集节点（A）是"眼"，ESP32-S3 网关是"网关与上云口"，B 挂在同一条 250 kbps CAN 总线上，收 `0x302` 干活、发 `0x102` 汇报、收 `0x000` 立刻停。

## 一、项目简介

本仓库是"工业边缘智能网关"项目的执行侧固件。整套系统由三个嵌入式仓库组成（采集节点 A、执行节点 B、ESP32-S3 网关），本仓库是其中的执行节点；CubeMX 工程结构，只保留 `Core/Inc` 与 `Core/Src`。工程内真正跑起来的数据流只有三条：

1. **命令下行**：网关发 `0x302 CTRL_MOTOR_B` → CAN 接收中断入队 → `canRxTask` 解包、做 `cid` 幂等、写"命令邮箱" → `motorTask` 调用 `Motor_SetSpeed()` → TIM3_CH1 占空比 + PA4/PA5 方向脚。
2. **状态上行**：`canTxTask` 1 Hz 打包 `0x102 B_STATUS`（电机状态、当前占空比、模式、flags 位图），`canHeartTask` 1 Hz 发 `0x702` 心跳。
3. **安全与本地显示**：`0x000` 急停广播置/清 `g_estop_istrue`；`canRxTask` 同时监听 A 的 `0x101` 传感器帧、`0x701/0x703` 心跳与 `0x103` 网关状态，把 ID、DLC、丢帧数、传感器值、在线状态刷到 0.96 寸 OLED 上。

边界要说清楚：舵机式的"目标速度跟踪"没有做，`Motor_SetSpeed()` 收的是**占空比百分比**，不是转速；`MOTOR_SETSPEED` 的 `B1` 被直接当作 `0~100` 的占空比使用。

## 二、系统架构

```text
 ┌──────────────┐   MQTT(JSON)  ┌─────────────────────┐
 │  MQTTX / 云端│◄─────────────►│  gateway_node       │
 │   手机端     │               │  ESP32-S3 NODE_GW=3 │
 └──────────────┘               └──────┬───────┬──────┘
               0x3xx 命令 / 0x000 急停 │       │ 0x102/0x702、0x101
                                       ▼       ▼
        ═════════ CAN 2.0A 250 kbps（总线两端各 120Ω）═════════
      ┌──────────────────┐           ┌──────────────────┐
      │ collecton_node   │           │ control_node  ★  │
      │ NODE_A=1  采集   │           │ NODE_B=2  执行   │
      └──────────────────┘           └──────────────────┘
```

B 节点没有自己的传感器输入决策链，但**监听了 A 的 `0x101`**：A 的传感器帧是广播，B 收到后只用于 OLED 显示与丢帧统计，不参与任何控制判断——这是"执行节点也可本地观察"的设计，不是闭环。

### 三个工程分工

| 仓库 | 硬件平台 | 节点号 | 核心职责 | 主要帧（据 `protocol.h` 帧总表） |
| --- | --- | --- | --- | --- |
| `collecton_node` | STM32F103 | `NODE_A = 1` | 红外/光敏等模拟量采集、事件报警 | 发 `0x101`/`0x701`（`0x011` 报警协议已定义、尚未实现）；收 `0x000`/`0x103`/`0x702`/`0x703`（`0x301`/`0x311` 未解析） |
| `control_node` ★ | STM32F103 | `NODE_B = 2` | 电机执行、状态回传、急停响应 | 发 `0x102`/`0x702`；收 `0x000`/`0x302`/`0x703`/`0x101`/`0x103`/`0x701` |
| `gateway_node` | ESP32-S3 | `NODE_GW = 3` | CAN ⇄ WiFi/MQTT 双向桥接、判活与统计 | 发 `0x103`/`0x703` 与全部 `0x3xx` 控制帧；收各节点上报（断网本地缓存为后续阶段，代码未实现） |

三个嵌入式工程**共用同一份 `protocol.h`**（纯头文件，只有宏与 `static inline`，只依赖 `<stdint.h>`），任一端的结构体/字节序改动都必须三端同步。

## 三、目录结构

```text
control_node/
└── Core/
    ├── Inc/
    │   ├── FreeRTOSConfig.h        # 1 kHz tick、56 级优先级、heap_4 8 KB
    │   ├── stm32f1xx_hal_conf.h    # HAL 模块裁剪、HSE_VALUE=8 MHz、TIM4 时基
    │   ├── can.h                   # CanFrame 结构、接收队列句柄、CAN_Send
    │   ├── protocol.h              # CAN 应用层协议 v3.0（三端共用，无 .c）
    │   ├── motor.h / pwm.h / adc_dma.h / serial.h  # 电机、PWM、ADC+DMA、串口接口
    │   └── iic.h / oled.h / OLED_Font.h  # 软件 I2C、SSD1306 显示、8×16 字模
    └── Src/
        ├── main.c                  # 时钟树、外设初始化顺序、TIM4 回调 HAL_IncTick
        ├── freertos.c              # 5 个任务 + 接收队列 + 命令邮箱 + 判活（核心）
        ├── stm32f1xx_hal_msp.c     # UART/ADC/CAN 的引脚、时钟、NVIC
        ├── stm32f1xx_hal_timebase_tim.c  # HAL 时基改用 TIM4（把 TIM3 让给 PWM）
        ├── stm32f1xx_it.c          # Cortex-M3 异常 + TIM4_IRQHandler
        ├── can.c                   # 250 kbps 位定时、全通滤波、FIFO0 中断收帧
        ├── motor.c                 # H 桥方向组合、换向时序、±100 限幅
        ├── pwm.c                   # TIM3_CH1 PWM1 模式、百分比 → CCR
        ├── adc_dma.c               # ADC1 扫描 + DMA1_Channel1 循环模式
        ├── serial.c                # USART2 阻塞收发 + 帧打包/解析
        ├── iic.c                   # GPIO 模拟 I2C + DWT 微秒延时
        ├── oled.c                  # SSD1306 初始化、8×16 字符、1 KB 绘图缓冲
        └── gpio.c / interrupt.c    # 端口时钟与 LED、PB12 按键 EXTI + 20 ms 消抖
```

仓库里**没有** `Drivers/`、`.ioc`、`.uvprojx`、`Makefile`、`CMakeLists.txt`——只有应用层源码，见第七节。

## 四、核心模块说明

| 文件 | 职责 | 关键实现点 |
| --- | --- | --- |
| `freertos.c` | 任务调度与业务主干 | 5 个任务；`canRxQueue`（深度 8，元素 `CanFrame`）；命令邮箱两个 volatile 变量；`B_GwOnline()` 统一判活；`B_GetFlags()` 现算 flags |
| `can.c` | CAN1 底层收发 | 正常模式、250 kbps、全通掩码滤波器、FIFO0 中断收帧入队、发送前查空闲邮箱、`AutoBusOff`/`AutoRetransmission` 使能 |
| `motor.c` | 电机执行层 | 方向 = PA4/PA5 组合（00 停 / 10 正转 / 01 反转）；换向前先降占空比并等一个 PWM 更新事件；`±100` 限幅 |
| `pwm.c` | 调速通道 | TIM3_CH1、PWM 模式 1、`PSC=71`/`ARR=999`、CCR 预装载；`%` → CCR 换算，100% 写 `PWM_PERIOD` 而非 `ARR` |
| `adc_dma.c` | 模拟量采集 | ADC1 规则组 CH0/CH1、扫描 + 连续 + DMA 循环，`adc_dma_buf[2]` 由 CPU 直接读；**本节点未消费该数据** |
| `serial.c` | 调试串口 | USART2 115200 8N1、阻塞收发、`0xAA 0x55 + len + data + sum` 帧格式；**当前无任务调用** |
| `iic.c` / `oled.c` | 本地显示 | 软件 I2C（PB8=SCL、PB9=SDA，开漏，地址 `0x78`）、SSD1306 128×64、8×16 字模、4 行 ×16 列；另备 1 KB 缓冲与 `OLED_Update()`（运行期未启用） |
| `gpio.c` / `interrupt.c` | 端口与按键 | `MX_GPIO_Init()` 只开时钟、`LED_Init()` 自行补开 GPIOB 后配 PB15/PA8/PC13；PB12 下降沿 EXTI、优先级 5、20 ms 消抖翻转 `g_sample_enable`（该变量在本仓库无人读取） |
| `stm32f1xx_hal_timebase_tim.c` | HAL 时基 | TIM4 产生 1 ms 中断 → `HAL_IncTick()`，避免与 TIM3 PWM 抢定时器 |

### FreeRTOS 任务与同步对象

`configTICK_RATE_HZ = 1000`（1 tick = 1 ms）；`configTOTAL_HEAP_SIZE = 8192`（heap_4）；CMSIS-RTOS v2 的 `stack_size` 单位是**字节**。

| 任务 | CMSIS 优先级 | 栈 | 周期/触发 | 职责 |
| --- | --- | --- | --- | --- |
| `motorTask` | `osPriorityAboveNormal`（32） | 128×4 = 512 B | 10 ms 轮询 | 唯一调用 `Motor_SetSpeed()`；执行命令邮箱；急停时归零并丢弃积压命令；网关掉线 3 s 停车 |
| `canTxTask` | `osPriorityAboveNormal`（32） | 256×4 = 1024 B | 1000 ms | 打包并发 `0x102 B_STATUS` |
| `canRxTask` | `osPriorityNormal`（24） | 256×4 = 1024 B | 阻塞取队列，超时 500 ms | 解析 6 类收帧、刷 OLED、A/网关心跳判活 |
| `canHeartTask` | `osPriorityLow7`（15） | 256×4 = 1024 B | 1000 ms | 发 `0x702` 心跳（魔数 `0x5A` + 计数） |
| `defaultTask` | `osPriorityLow`（8） | 128×4 = 512 B | 2000 ms | 翻转 PC13 LED，指示系统在跑 |

队列/信号量清单：只有 `canRxQueue = osMessageQueueNew(8, sizeof(CanFrame), NULL)`（`CanFrame` = ID 4 B + DLC 1 B + 数据 8 B，对齐后 16 B）。`RTOS_MUTEX`/`RTOS_SEMAPHORES`/`RTOS_TIMERS` 段全空——**没有互斥量、没有信号量、没有软定时器**（`configUSE_TIMERS = 1` 只让内核创建了定时器守护任务）。

### "收 CAN" 与 "电机控制" 的解耦

```text
CAN1 RX0 中断             canRxTask（Normal）       motorTask（AboveNormal）
回调里整帧入队  ──►       解析 0x302 / 0x000  ──►   读邮箱 → Motor_SetSpeed()
（osMessageQueuePut）     写 g_motor_target         （唯一执行者）
                          再 ++g_motor_cmd_cnt
```

三点理由（`freertos.c` 注释里有明确依据）：

1. **超时停车需要"定时醒来"**。`canRxTask` 是纯事件驱动，没帧就不跑，回答不了"距上次命令过了多久"。
2. **`canRxTask` 刷一屏要占用软件 I2C 约 23 ms**（每字符两次 8 字节 I2C 事务）。若电机逻辑挂在它后面，一条急停要等整屏刷完，响应时间被 OLED 内容决定。
3. **单写者/单读者**：`g_motor_target` 先写、`g_motor_cmd_cnt` 后写，`motorTask` 以 `cnt` 变化为"有新命令"的判据，读到新 `cnt` 必然看到新 `target`；`motorTask` 优先级更高，可打断刷屏（SysTick 每 1 ms 抢占）。

## 五、CAN 应用层协议（v3.0）

公共约定（三端一致，改任一项都要同步）：

- **帧类型与 ID 结构**：CAN 2.0A 标准帧，11 位 ID，250 kbps（采样点 87.5%）；`ID = 功能组(高 7 位，16 的倍数) | 节点号(低 4 位)`，宏 `CAN_ID(group, node)`；本节点 `NODE_B = 2`。
- **字节序**：多字节**大端**，必须走 `PROTO_PutU16/GetU16` 逐字节移位，不能强转指针（小端 CPU + 对齐风险）。
- **DLC 固定**：每帧长度等于 `LEN_xxx`；接收侧每类帧都先校验 `frame.len == LEN_xxx`，不等则整帧忽略。
- **`cid` 幂等**：`0x302/0x312/0x322/0x332` 末字节是命令序号，接收端记住上一次 `cid`，相同即丢弃（CAN 硬件自动重发会带来重复帧）。
- **心跳判活**：`0x701` 魔数 `0xA5`（A）、`0x702` 魔数 `0x5A`（B）、`0x703` 魔数 `0xC3`（网关）；ID + 魔数都对才算有效，超时 3000 ms 判掉线。
- **急停最高优先级**：`0x000` 是数值最小的标准帧 ID，仲裁天然最高；停/解除由载荷 `B0` 区分，因为低 4 位已被节点号占用。

### 本节点接收的帧

| ID | DLC | 方向 | 名称 | 本仓库处理 |
| --- | --- | --- | --- | --- |
| `0x000` | 1 | 广播 → 全体 | ESTOP | 是：`B0=2` 解除，其余值一律置急停 |
| `0x101` | 5 | A → 全体 | A_SENSOR | 是：解包红外/光敏/seq，OLED 显示 + `PROTO_SeqLost()` 丢帧 |
| `0x103` | 4 | 网关 → 全体 | GW_STATUS | 是：OLED 显示 WiFi/MQTT/本地缓存条数 |
| `0x302` | 3 | 网关 → B | CTRL_MOTOR_B | 是：`cid` 幂等 + 命令翻译 + 写邮箱 |
| `0x701` | 2 | A → 全体 | A_HEARTBEAT | 是：刷 A 在线状态 |
| `0x703` | 2 | 网关 → 全体 | GW_HEARTBEAT | 是：刷 `g_gw_hb_tick`，供断线保护与 flags bit0 |

`0x312 CTRL_TARGET_B`、`0x322 CTRL_PID_B`、`0x332 CTRL_MODE_B` **在协议里定义、在本仓库未处理**（`canRxTask` 中没有对应分支，`B_STATUS` 的 `B2` 恒为 `MODE_MANUAL`）。

### 本节点发送的帧

| ID | DLC | 方向 | 名称 | 周期 | 载荷 |
| --- | --- | --- | --- | --- | --- |
| `0x102` | 4 | B → 全体 | B_STATUS | 1000 ms | `B0` 电机状态 `MST_xxx`、`B1` 当前占空比 0~100、`B2` 模式、`B3` flags |
| `0x702` | 2 | B → 全体 | B_HEARTBEAT | 1000 ms | `B0 = 0x5A`、`B1` 计数自增 |

`0x102` 的 `B0/B1` 来自 `Motor_GetSpeed()` 的有符号返回值：符号定方向，绝对值当占空比。`B3` flags 中 bit0 `ST_COMM_OK` 由 `B_GwOnline()` 现算（3 s 内有网关心跳）、bit3 `ST_ESTOP` 取 `g_estop_istrue`；**bit1 `ST_THRESH`、bit2 `ST_ALARM` 恒为 0**，代码注释写明"还没有生产者，硬填就是造假位"。`0x012 ALARM_B` 在协议里定义了，但本仓库**没有任何一处调用 `PROTO_PackAlarm()`**，因此报警帧目前不会被发出。

## 六、硬件与引脚

MCU 依据：三个源文件都 `#include "stm32f103xb.h"`，`stm32f1xx_hal_conf.h` 中 `HSE_VALUE = 8000000`，工程使用 CAN1 与 `USB_LP_CAN1_RX0_IRQn` → **STM32F103xB 系列（中等容量，128 KB Flash / 20 KB SRAM，Cortex-M3）**。结合 PC13 板载 LED 与 8 MHz 晶振，实物多为 STM32F103C8T6 核心板；仓库未附 `.ioc`，具体封装需按实物核对。时钟树（`main.c: SystemClock_Config()`）：HSE 8 MHz → PLL ×9 = **SYSCLK 72 MHz** → AHB **÷2** ⇒ **HCLK = 36 MHz**，APB1/APB2 均 ÷1 ⇒ **PCLK1 = PCLK2 = 36 MHz**，Flash 延迟 2 WS。

### 引脚表

| 引脚 | 复用/模式 | 功能 | 依据 |
| --- | --- | --- | --- |
| PA0 | 模拟输入 | ADC1_IN0（规则组 Rank1，本节点未消费） | `HAL_ADC_MspInit` |
| PA1 | 模拟输入 | ADC1_IN1（Rank2，未消费） | `HAL_ADC_MspInit` |
| PA2 | 复用推挽 | USART2_TX，115200 8N1 | `HAL_UART_MspInit` |
| PA3 | 上拉输入 | USART2_RX | `HAL_UART_MspInit` |
| PA4 | 推挽输出 | 电机方向 IN1（`MOTOR_PIN1`） | `motor.c` |
| PA5 | 推挽输出 | 电机方向 IN2（`MOTOR_PIN2`） | `motor.c` |
| PA6 | 复用推挽 | TIM3_CH1，电机 PWM | `pwm.c` |
| PA8 | 推挽输出 | LED，仅 `Error_Handler()` 里闪烁 | `LED_Init` |
| PA11 | 输入 | CAN1_RX（接收发器 RXD） | `HAL_CAN_MspInit` |
| PA12 | 复用推挽 | CAN1_TX（接收发器 TXD） | `HAL_CAN_MspInit` |
| PB8 | 开漏输出 | 软件 I2C SCL（OLED） | `iic.c` |
| PB9 | 开漏输出 | 软件 I2C SDA（OLED，从机地址 `0x78`） | `iic.c` |
| PB12 | 上拉 + 下降沿 EXTI | 按键，翻转 `g_sample_enable` | `Key_Init` |
| PB15 | 推挽输出 | LED，`LED_Init` 置高后未再操作 | `LED_Init` |
| PC13 | 推挽输出 | 板载 LED，`defaultTask` 每 2000 ms 翻转 | `LED_Init` + `StartDefaultTask` |
| PA13/PA14 | SWD | 下载调试（`__HAL_AFIO_REMAP_SWJ_NOJTAG` 关 JTAG、留 SWD） | `HAL_MspInit` |

硬件形态：**直流有刷电机 + 双 H 桥驱动板**（`motor.c` 注释按 IN1/IN2 真值表写，注明接"驱动板的 IN1/AIN1、IN2/AIN2"，L298N/TB6612 一类模块通用），**没有编码器、没有电流采样**，因此不存在测速与闭环。

### 外设资源占用

| 资源 | 配置 | 用途 |
| --- | --- | --- |
| TIM3 CH1 | `PSC=71`、`ARR=999`、PWM 模式 1、CCR 预装载、有效电平高 | 电机调速；计数时钟 36 MHz/72 = 500 kHz ⇒ **载频约 500 Hz**（`pwm.c` 注释按 72 MHz 推算写作 1 kHz） |
| TIM4 | 1 ms 更新中断（`HAL_InitTick`） | HAL 时基 `HAL_IncTick()`，故 TIM3 才给 PWM |
| CAN1 | 正常模式、`Prescaler=9`、`BS1=13tq`、`BS2=2tq`、`SJW=2tq` ⇒ 16 tq × 4 MHz = 250 kbps、采样点 87.5%、`AutoBusOff`/`AutoRetransmission` 使能、RX FIFO0 中断 | 协议收发 |
| 滤波器 | Bank0、IDMASK、32 位、ID 与掩码全 0 ⇒ 全通 | 三端所有帧都进 FIFO0，由任务按 ID 分派 |
| USART2 | 115200、8N1、全双工、阻塞收发 | 调试口（当前无调用） |
| ADC1 + DMA1_Channel1 | 扫描 + 连续 + 循环、2 通道、`ADC_SAMPLETIME_55CYCLES_5`、ADCCLK = 36 MHz/6 = 6 MHz | 预留模拟量（未消费） |
| 软件 I2C | PB8/PB9 开漏 + DWT CYCCNT 微秒延时 | SSD1306 128×64 |
| FreeRTOS | heap_4 8 KB、1 kHz tick、5 个应用任务 | 调度 |


## 七、技术栈与技术亮点

技术栈：C（C99，HAL 风格）、STM32F1 HAL、FreeRTOS 10.3.1 + CMSIS-RTOS v2 封装、heap_4、CAN 2.0A、SSD1306 软件 I2C、DWT 周期计数器延时、CubeMX 目录约定。亮点（均能在代码里找到对应实现）：

1. **中断只做入队**：`HAL_CAN_RxFifo0MsgPendingCallback()` 拷一帧进 `CanFrame` 后 `osMessageQueuePut(..., 0U)`，超时给 0，队列满就丢一帧而不在中断里阻塞。
2. **收/执行分离 + 命令邮箱**：`canRxTask` 只解析与写邮箱，`motorTask` 是 `Motor_SetSpeed()` 的唯一调用者，避免两个任务同时改 PWM 与方向脚打乱换向时序。
3. **换向时序防电流尖峰**：`SetPWMDuty(0)` → `PWM_WaitNextPeriod()` 等更新事件让 0% 真正落到引脚 → 再翻 IN1/IN2 → 再给占空比；同方向调速不会插入多余停顿。
4. **`fail-safe` 的急停解析**：`0x000` 只把 `B0 == ESTOP_ACTION_CLEAR(2)` 当解除，其余任何值（含未知值）都置急停，符合 `protocol.h` 对安全帧的硬性要求。
5. **判活判据选"网关心跳"而非"命令超时"**：命令是状态不是事件，网关活着就一直保持最后速度；用命令超时会让网关被迫每秒重发同一命令、并把 `cid` 逼成保活计数器。
6. **状态位现算不缓存**：`B_GetFlags()` 每次打包时用一次减法 + 比较算 bit0，避免"网关口掉电后那 3 秒里没有任何代码去清位"的脏状态。
7. **`volatile` 命令邮箱的写序 + `osThreadNew` 返回值检查**：先写 `g_motor_target` 再 `++g_motor_cmd_cnt`，16 位对齐读写下无锁也读不到半新半旧的一对值；栈与 TCB 都从 8 KB heap 分配，堆不够时任务会静默不运行，因此创建后立即判 `NULL`。
8. **协议层纯头文件**：`protocol.h` 只有宏与 `static inline`，包含它的每个 `.c` 各拿一份函数副本，不产生链接冲突，也不依赖任何 HAL 类型，可直接给 ESP32 复用。

## 九、典型问题与排查

**1. 网关下发了 `0x302`，OLED 上能看到 ID，但电机不转**

- 根因：`motorTask` 的断线保护要求"正在转时 3 s 内有 `0x703` 网关心跳"；且上电时 `g_gw_hb_tick = 0`，未确认网关在线前 `B_GwOnline()` 返回 `false`，速度会被立刻清零。若网关只发命令不发心跳，电机永远起不来。
- 定位：看 OLED 第 4 行 `gw ON/gw OFF`，再看 `0x102` 的 `B3` bit0（`ST_COMM_OK`）；也可断点 `B_GwOnline()` 观察 `g_gw_hb_tick` 是否被 `0x703` 分支刷新。
- 处理：确认网关按 1 Hz 发 `0x703` 且 `B0 = 0xC3`；只做电机联调时需同步模拟网关心跳，不能只发命令。

**2. 网关发的第一条电机命令"没反应"，之后都正常**

- 根因：`canRxTask` 中幂等判据是 `static uint8_t last_cid = 0;`，初值就是 0——网关若从 `cid = 0` 开始编号，第一帧被判为重复而丢弃。
- 定位：在 `if (cmd.cid != last_cid)` 处断点，看首帧 `cmd.cid` 是否为 0。
- 处理：网关 `cid` 从 1 起编号，或把本地初值改成 `0xFF`（任选其一，两端约定即可）。

**3. 急停响应"慢半拍"，或某条急停帧完全没生效**

- 根因：`0x000` 分支在 `canRxTask` 里处理，而该任务每次刷屏要在**软件 I2C** 上跑约 23 ms，急停只能等当前这一轮显示做完；此外前置条件是 `frame.len == LEN_ESTOP`，DLC 不是 1 的急停帧会被整帧忽略——`protocol.h` 的兜底要求只覆盖了 `B0` 取值，没兜 DLC。
- 定位：示波器同时抓 `0x000` 帧与 PA4/PA5/PA6；或统计 `canRxTask` 中 `osMessageQueueGet` 返回后到写 `g_estop_istrue` 的时间差。
- 处理：如需更硬的实时性，把 `0x000` 的判定提到 `HAL_CAN_RxFifo0MsgPendingCallback()` 里直接置 `volatile` 标志（只做一次 ID/DLC 比较，不碰 OLED），任务侧只负责显示。

**4. 实测 PWM 载频不是注释里的 1 kHz，I2C 也偏慢**

- 根因：`main.c` 的时钟树在 PLL ×9（72 MHz）之后又做了 **AHB ÷2**，实际 HCLK = 36 MHz。`pwm.c` 按 72 MHz 推算 `PSC=71` 得 1 kHz，实际是 `36 MHz/72/1000 ≈ 500 Hz`；同理 `adc_dma.c` 注释的 ADCCLK 12 MHz 实为 6 MHz，`iic.c` 的 `CPU_FREQ_MHZ 72` 使 `Delay_us(1)` 实际约 2 µs，软件 I2C 速率约为设计值一半。这些注释与实配不符，属于**已知差异**，不是编译器问题。
- 定位：示波器测 PA6 周期（应约 2 ms）；或读 `RCC->CFGR` 的 HPRE 位确认分频。
- 处理：按 500 Hz 选驱动板参数即可；若要 1 kHz，把 `PWM_PRESCALER` 改成 35（36 MHz/36/1000 = 1 kHz），同时把 `iic.c` 的 `CPU_FREQ_MHZ` 改成 36 让 I2C 时基正确。
