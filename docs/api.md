# 接口速查（API Reference）

本文是**查阅用**的速查表：每个模块向应用层暴露了哪些函数、什么前置条件、能不能在中断里调、值域是多少。

**各模块的实现原理与踩坑记录不在本文**，见对应详解：

| 模块 | 原理详解 |
| --- | --- |
| OLED | [oled.md](oled.md) |
| 按键 | [key.md](key.md) |
| 电位器 | [pot.md](pot.md) |
| 串口 | [serial.md](serial.md) |
| 电机 | [motor.md](motor.md) |
| 编码器 | [encoder.md](encoder.md) |
| 角度传感器 | [angle.md](angle.md) |
| 系统节拍 | 见 [bsp/tick/bsp_tick.c](../bsp/tick/bsp_tick.c) 文件头（1 ms 的推导过程） |

**面向读者**：本工程的使用者。假设你会 C、知道中断和 volatile 的基本概念，不需要读过实现。

---

## 目录

- [一、全局约定](#一全局约定)
  - [1.1 分层与依赖方向](#11-分层与依赖方向)
  - [1.2 错误码 error_t](#12-错误码-error_t)
  - [1.3 句柄与引脚宏](#13-句柄与引脚宏)
  - [1.4 初始化顺序](#14-初始化顺序硬依赖不能乱)
  - [1.5 中断优先级](#15-中断优先级)
- [二、模块接口](#二模块接口)
  - [2.1 系统节拍 bsp_tick](#21-系统节拍--bsptickbsp_tickh)
  - [2.2 按键 bsp_key](#22-按键--bspkeybsp_keyh)
  - [2.3 电位器 bsp_pot](#23-电位器--bsppotbsp_poth)
  - [2.4 串口 bsp_serial](#24-串口--bspserialbsp_serialh)
  - [2.5 OLED](#25-oled)
  - [2.6 电机 bsp_motor](#26-电机--bspmotorbsp_motorh)
  - [2.7 编码器 bsp_encoder](#27-编码器--bspencoderbsp_encoderh)
  - [2.8 角度传感器 bsp_angle](#28-角度传感器--bspanglebsp_angleh)
  - [2.9 PID 算法 algorithm/pid](#29-pid-算法--algorithmpidpidh)
  - [2.10 控制器 app/control](#210-控制器--appcontrolcontrolh)
  - [2.11 串口控制台 app/console](#211-串口控制台--appconsoleconsoleh)
- [三、调用上下文总表](#三调用上下文总表)
- [四、主循环最小骨架](#四主循环最小骨架)
- [五、陷阱清单](#五陷阱清单)

---

## 一、全局约定

### 1.1 分层与依赖方向

```
应用层    Core/main.c + app/{control,console}/    把算法和 bsp 组装成具体功能
           │  只能往下调，不允许被下层反向调用
算法层    algorithm/pid/       纯算法，只依赖 <stdbool.h>，**不含任何硬件头文件**
           │
BSP 层    bsp/{tick,key,pot,serial,oled,motor,encoder,angle}/   「这块板上什么接在哪个脚」
           │  可以碰 HAL、可以碰 CubeMX 生成的引脚宏
设备层    driver/device/ssd1306/                  只认 i2c_if_t，零 HAL 符号
总线层    driver/bus/i2c/i2c_soft.*               贴着 HAL，允许 HAL_GPIO_*
公共层    driver/common/error.h                   全工程错误码
```

`algorithm/` 是**独立的一层**：它不让任何人依赖硬件，也不依赖 `bsp_*`——
这样它能在 PC 上用 gcc 单独编译、跑单元测试（阶跃响应、积分饱和这类错误在 PC 上看比烧板子快）。

应用层可以 include `bsp_*.h`、`algorithm/` 和 `app/` 内的头文件。
唯一需要跨层拿的东西是 OLED 字库表：要额外 include [ssd1306_fonts.h](../driver/device/ssd1306/ssd1306_fonts.h)（`ssd1306.h` 已经由 `bsp_oled.h` 带进来）。

### 1.2 错误码 error_t

定义在 [driver/common/error.h](../driver/common/error.h)。**所有 `_init` 系列都返回它，调用者必须检查**。

| 值 | 含义 | 哪些接口会返回 |
| --- | --- | --- |
| `ERR_OK` | 成功 | 全部 |
| `ERR_TIMEOUT` | 等待超时 | `bsp_pot_sample`、`bsp_serial_flush` |
| `ERR_BUSY` | 设备/总线忙 | 目前无接口主动返回 |
| `ERR_INVALID_PARAM` | 空指针 / 越界 | 各 `*_init`、`ssd1306_set_cursor` |
| `ERR_NOT_INITIALIZED` | 没初始化就调用 | `bsp_pot_sample`、`bsp_serial_flush`、`ssd1306_set_*` |
| `ERR_NOT_READY` | 硬件没就绪 | 各 `*_init`、`bsp_pot_sample` |
| `ERR_OVERFLOW` | 溢出 | `bsp_tick_register`（槽位满）、`bsp_key_init`（转发） |
| `ERR_UNDERFLOW` | 下溢 | 目前无接口使用 |
| `ERR_COMMUNICATION` | 从机没应答 | `bsp_oled_init`、`ssd1306_*` |
| `ERR_NOT_SUPPORTED` | 不支持的操作 | 目前无接口使用 |

初始化失败的报错方式是编成 LED 闪烁次数（见 [main.c](../Core/Src/main.c) 的 `app_fatal_blink()`），以后加模块往后顺延编号：

| 闪烁次数 | 模块 |
| --- | --- |
| 1 | OLED（屏没应答） |
| 2 | 系统节拍（HAL 启动失败） |
| 3 | 按键（节拍槽位满） |
| 4 | 电位器（ADC 自校准失败） |
| 5 | 串口（接收中断没武装上） |
| 6 | 电机（PWM 没启动起来） |
| 7 | 编码器（HAL 启动失败） |
| 8 | 角度传感器（ADC1 自校准失败） |
| 9 | 节拍任务注册失败（槽位满） |
| 10 | 控制器（`control_init` 失败） |
| 11 | 串口控制台（`console_init` 失败） |

### 1.3 句柄与引脚宏

**句柄**：CubeMX 只在 `main.c` 里**定义** `hadc2` / `htim1` / `huart1`，**不给 `main.h` 生成 extern**。所以每个 BSP 在自己的 `.c` 里补一份：

| 符号 | 类型 | 声明处 |
| --- | --- | --- |
| `htim1` | `TIM_HandleTypeDef` | [bsp/tick/bsp_tick.c](../bsp/tick/bsp_tick.c) |
| `htim2` | `TIM_HandleTypeDef` | [bsp/motor/bsp_motor.c](../bsp/motor/bsp_motor.c) |
| `htim3` | `TIM_HandleTypeDef` | [bsp/encoder/bsp_encoder.c](../bsp/encoder/bsp_encoder.c) |
| `hadc1` | `ADC_HandleTypeDef` | [bsp/angle/bsp_angle.c](../bsp/angle/bsp_angle.c) |
| `hadc2` | `ADC_HandleTypeDef` | [bsp/pot/bsp_pot.c](../bsp/pot/bsp_pot.c) |
| `huart1` | `UART_HandleTypeDef` | [bsp/serial/bsp_serial.c](../bsp/serial/bsp_serial.c) |

⚠️ 应用层**不要重复定义**这六个符号，也不要改动它们。

**引脚宏**（由 CubeMX 生成在 [main.h](../Core/Inc/main.h)）：

| 宏 | 引脚 | 归谁用 |
| --- | --- | --- |
| `LED_Pin` / `LED_GPIO_Port` | PC13 | 应用层（心跳 / 错误闪烁） |
| `KEY1..KEY4_Pin` / `_GPIO_Port` | PB10 / PB11 / PA11 / PA12 | 仅 bsp_key.c 内部 |
| `OLED_SCL_Pin` / `OLED_SDA_Pin` | PB8 / PB9 | 仅 bsp_oled.c 内部 |
| `MOTOR_AIN1_Pin` / `_GPIO_Port` | PB12 | 仅 bsp_motor.c 内部 |
| `MOTOR_AIN2_Pin` / `_GPIO_Port` | PB13 | 仅 bsp_motor.c 内部 |

⚠️ **ADC 通道、定时器通道、USART 的引脚 —— CubeMX 都不生成宏**（只有配成 GPIO 的引脚才会给 `KEY1_Pin` 这类宏）。
所以「哪个通道接什么」的对应表写在 BSP 里，那正是 BSP 层的职责：电位器的 4 个 ADC 通道表在 [bsp_pot.c](../bsp/pot/bsp_pot.c)，电机的 TIM2_CH1 与角度传感器的 ADC1_IN8 同理。

### 1.4 初始化顺序（硬依赖，不能乱）

| # | 调用 | 前置条件 | 违反了会怎样 |
| --- | --- | --- | --- |
| 1 | `HAL_Init()` | — | 无 SysTick，`HAL_Delay` 死锁 |
| 2 | `SystemClock_Config()` | — | 72 MHz 与 ADC 时钟分频都靠它 |
| 3 | `MX_GPIO_Init()` | — | 引脚浮空；**GPIOB 时钟也在这里使能**，OLED 与电机方向脚靠它 |
| 4 | `MX_TIM1_Init()` | — | 只写寄存器，定时器还没跑（要 `bsp_tick_init` 才启动） |
| 5 | `MX_ADC2_Init()` | — | 通道 / 采样时间 / 扫描模式没配 |
| 6 | `MX_USART1_UART_Init()` | — | 波特率、字长没配 |
| 7 | `MX_TIM2_Init()` | — | PWM 通道没配（且**定时器还没启动**，见 2.6） |
| 8 | `MX_TIM3_Init()` | — | 编码器接口没配 |
| 9 | `MX_ADC1_Init()` | — | 角度传感器的通道没配 |
| 10 | `bsp_oled_init(&s_oled)` | 步骤 3 | GPIOB 时钟没开 → 屏幕无应答 |
| 11 | `bsp_tick_init()` | 步骤 4 | 节拍不跑 |
| 12 | `bsp_key_init()` | 步骤 3 **和** 11 | 无 tick：注册失败；无 GPIO：读到浮空电平，**开机会凭空多一次按下事件** |
| 13 | `bsp_pot_init()` | 步骤 5 | 自校准要求 ADC 关闭（ADON=0），此刻正好满足 |
| 14 | `bsp_serial_init()` | 步骤 6 | 武装接收中断时串口还没配好 |
| 15 | `bsp_motor_init()` | 步骤 3 **和** 7 | 顺序有意排先：它把电机置为「停」，**越早做摆杆越安全** |
| 16 | `bsp_encoder_init()` | 步骤 8 | 游标要对齐到当前 CNT |
| 17 | `bsp_angle_init()` | 步骤 9 | 初始化时**一次性使能 ADC 并等 ADRDY**（之后永不 Stop），否则中断里没法启动转换 |
| 18 | `control_init()` | 步骤 15/16/17 | 会读一次角度和编码器来对齐内部快照，三个驱动没起来会读到 0 |
| 19 | `console_init()` | 步骤 14 | 只是协议层，不碰 USART 寄存器 |
| 20 | `bsp_tick_register(control_tick)` | 步骤 11 **和 18** | 控制环必须在三个驱动 + 控制器都就绪之后才挂上节拍 |

`bsp_tick_register()` 在步骤 11 之后随时可调，槽位上限见 2.1（**当前已用 2 个：按键 + 控制环**）。

> **唯一一条「顺序反了也返回成功、但行为是错的」是第 12 条**——其余顺序错了都会在 `_init` 返回错误码，能立刻看见。这也是它最值得记的原因。

### 1.5 中断优先级

数值越小优先级越高：

| 中断 | 抢占优先级 | 来源 |
| --- | --- | --- |
| TIM1_UP（1 ms 节拍） | 1 | [stm32f1xx_hal_msp.c](../Core/Src/stm32f1xx_hal_msp.c) |
| USART1 | 2 | 同上 |
| SysTick（`HAL_GetTick` 的时基） | 15（最低） | [stm32f1xx_hal_conf.h](../Core/Inc/stm32f1xx_hal_conf.h) 的 `TICK_INT_PRIORITY` |

⚠️ **直接后果**：TIM1 比 SysTick 优先级高 → **在 1 ms 节拍任务里调用任何依赖 `HAL_GetTick()` 的超时等待都会死等**（`uwTick` 不涨，超时条件永远不满足）。所以「节拍任务必须快进快出」不只是性能建议，是功能性要求。受影响的接口见第三节。

---

## 二、模块接口

### 2.1 系统节拍 —— [bsp/tick/bsp_tick.h](../bsp/tick/bsp_tick.h)

TIM1，PSC=71 / ARR=999 @ 72 MHz = **精确 1 ms**。中断优先级 1。

```c
#include "bsp_tick.h"
```

**常量与类型**

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `BSP_TICK_PERIOD_MS` | 1 | 节拍周期。**与 `.ioc` 里 TIM1 的 PSC/ARR 绑死**，改定时器参数必须同步改这个宏 |
| `BSP_TICK_MAX_HANDLERS` | 4 | 任务槽位上限。**按键 + 控制环已占 2 个**，还剩 2 个 |
| `bsp_tick_fn_t` | `void (*)(void)` | 周期任务原型，无参数无返回值 |

**接口**

| 接口 | 签名 | 返回 | 前置条件 |
| --- | --- | --- | --- |
| 启动 | `error_t bsp_tick_init(void)` | `ERR_OK` / `ERR_NOT_READY` | `MX_TIM1_Init()` 之后 |
| 注册 | `error_t bsp_tick_register(bsp_tick_fn_t fn)` | `ERR_OK` / `ERR_INVALID_PARAM` / `ERR_OVERFLOW` | `bsp_tick_init()` 之后 |

**语义要点**

- 注册的任务运行在**中断上下文**（`TIM1_UP_IRQHandler` → `HAL_TIM_IRQHandler` → `HAL_TIM_PeriodElapsedCallback` → 你的函数）。约束见第三节。
- 槽位耗尽返回 `ERR_OVERFLOW`（不是静默失败），`bsp_key_init()` 会把同样的错误码转发出来。
- ⚠️ `HAL_TIM_PeriodElapsedCallback` 是 HAL 的 `__weak` 符号，**全工程只能有一份非弱定义**——就是 [bsp_tick.c](../bsp/tick/bsp_tick.c) 里那一份。以后启用别的定时器（如 TIM3）要用更新中断时，**不能另写同名函数**，只能在该函数里补 `else if (htim->Instance == TIM3)` 分支。

---

### 2.2 按键 —— [bsp/key/bsp_key.h](../bsp/key/bsp_key.h)

4 路独立按键，**上拉输入、按下为低**。采样与消抖跑在 1 ms 中断里，主循环零参与。

| 按键 | 引脚 |
| --- | --- |
| K1 | PB10 |
| K2 | PB11 |
| K3 | PA11 |
| K4 | PA12 |

```c
#include "bsp_key.h"
```

**类型**

| 类型 | 取值 |
| --- | --- |
| `key_id_t` | `KEY_ID_K1` = 0、`KEY_ID_K2`、`KEY_ID_K3`、`KEY_ID_K4`、`KEY_ID_COUNT` |
| `key_event_t` | `KEY_EVENT_NONE` = 0、`KEY_EVENT_PRESS`、`KEY_EVENT_RELEASE` |

`KEY_ID_COUNT` 是总数、也是遍历上界，**不是**一个可用的按键。

**接口**

| 接口 | 签名 | 返回 | 说明 |
| --- | --- | --- | --- |
| 初始化 | `error_t bsp_key_init(void)` | `ERR_OK` / `ERR_OVERFLOW` | 用当前真实电平做初始状态 + 注册 1 ms 采样 |
| 采样 | `void bsp_key_scan_isr(void)` | — | **中断上下文，由节拍调用；应用永远不要主动调** |
| 取事件 | `key_event_t bsp_key_take_event(key_id_t id)` | 事件枚举 | **读取即清除**（边沿） |
| 查电平 | `bool bsp_key_is_pressed(key_id_t id)` | true = 按下 | 消抖后的**稳定**电平（电平） |

**语义要点**

- **消抖 20 ms**（连续 20 次一致才认）。物理动作到事件产生有约 20 ms 延迟。
- **事件是标志位，不是队列**：每键只有 `press_pending` / `release_pending` 两个 `bool`。一个完整按放周期最多挂起 1 个 PRESS + 1 个 RELEASE；短时间连按两次而没及时取走，会**塌缩成一次**。**不要用它做计数**。
- **必须用 `while` 取干净**。同时挂起 PRESS 与 RELEASE 时先返回 PRESS，下一次调用才返回 RELEASE；只 `if` 一次会漏掉 RELEASE：

  ```c
  key_event_t ev;
  while ((ev = bsp_key_take_event(KEY_ID_K1)) != KEY_EVENT_NONE) {
      if (ev == KEY_EVENT_PRESS) { /* ... */ }
  }
  ```

- `bsp_key_take_event` 内部关中断（PRIMASK 保存/恢复，几十纳秒），可被任何上下文安全调用；但在中断里做按键逻辑不建议。
- `bsp_key_is_pressed` 读消抖后的稳定值，适合「按住加速」这类长按逻辑，配合 `HAL_GetTick()` 计时。
- 越界 `id` 不会崩：`take_event` 返回 `KEY_EVENT_NONE`，`is_pressed` 返回 false。

**若接线反了**（不按为低、按下为高）：现象是「不按显示按下、按了反而没反应」。改两处——[bsp_key.c](../bsp/key/bsp_key.c) 的 `BSP_KEY_ACTIVE_LEVEL` 改成 `GPIO_PIN_SET`，并把 `.ioc` 里四个引脚改成下拉（走 CubeMX，不要手改 `.ioc`）。

---

### 2.3 电位器 —— [bsp/pot/bsp_pot.h](../bsp/pot/bsp_pot.h)

4 路电位器走 **ADC2**，12 位。扫描模式关闭、序列长度 1、采样时间 55.5 周期、ADC 时钟 12 MHz。

| ID | 引脚 | ADC2 通道 |
| --- | --- | --- |
| `POT_ID_RP1` | PA2 | `ADC_CHANNEL_2` |
| `POT_ID_RP2` | PA3 | `ADC_CHANNEL_3` |
| `POT_ID_RP3` | PA4 | `ADC_CHANNEL_4` |
| `POT_ID_RP4` | PA5 | `ADC_CHANNEL_5` |

⚠️ 通道对应表写在 [bsp_pot.c](../bsp/pot/bsp_pot.c) 里（CubeMX 对 ADC 通道不生成引脚宏）。**改接线要同步改两处**：这张表和 `.ioc` 里 PA2~PA5 的信号。

```c
#include "bsp_pot.h"
```

**常量与类型**

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `pot_id_t` | `POT_ID_RP1` = 0 … `POT_ID_RP4`、`POT_ID_COUNT` | `POT_ID_COUNT` 是总数与遍历上界 |
| `BSP_POT_RAW_MAX` | 4095 | 12 位 ADC 满量程 |
| `BSP_POT_VREF_MV` | 3300 | 参考电压（VDDA = 3.3 V） |

**接口**

| 接口 | 签名 | 返回 | 阻塞 | 前置条件 |
| --- | --- | --- | --- | --- |
| 初始化 | `error_t bsp_pot_init(void)` | `ERR_OK` / `ERR_NOT_READY` | 约 83 个 ADC 周期 | `MX_ADC2_Init()` 之后 |
| 采样 | `error_t bsp_pot_sample(void)` | `ERR_OK` / `ERR_NOT_INITIALIZED` / `ERR_NOT_READY` / `ERR_TIMEOUT` | **约 23 µs** | `bsp_pot_init()` 之后 |
| 读原始值 | `uint16_t bsp_pot_raw(pot_id_t id)` | 0~4095（越界返回 0） | 无（纯内存） | — |
| 读电压 | `uint16_t bsp_pot_millivolt(pot_id_t id)` | 0~3300 mV（越界返回 0） | 无 | — |

**语义要点**

- **「采样」与「读数」分开是有意的**：4 路共用一个数据寄存器，`bsp_pot_sample()` 一次性把 4 路各采一遍（得到时间上彼此靠近的快照），之后 `bsp_pot_raw()` 只是读缓存，**不触发任何转换、没有副作用**，旋钮在转时也不会出现「四个值来自四个瞬间」的撕裂。
- **`bsp_pot_sample()` 只应在主循环调用**。它内部用 `HAL_ADC_PollForConversion`，超时依赖 `HAL_GetTick()`——而 TIM1 优先级高于 SysTick，**在节拍任务里调用会死等在超时判断里**（见 1.5）。
- **建议采样周期 20 ms 量级**（人手拧旋钮，50 Hz 足够跟手）。
- **失败时缓存是混合状态**：内部逐路做，某一路失败就立即返回，此时缓存里是「前面几路新值 + 后面几路旧值」。所以必须整批丢弃：

  ```c
  if (bsp_pot_sample() != ERR_OK) {
      return;      /* 本次不要用 bsp_pot_raw() 的值 */
  }
  ```

- `bsp_pot_millivolt` 用 4095 做满量程（不是 4096），先乘后除，中间结果 `4095 × 3300 ≈ 1.35e7` 用 `uint32_t` 装得下。
- **自校准不能省**：`bsp_pot_init` 里做了 `HAL_ADCEx_Calibration_Start`。漏掉的现象是「能读、但数值有几十个 LSB 的固定偏差，且随温度漂移」——**不报任何错**。

**读数异常时先查接线**：只接电位器两端、没接中间抽头 → 读数恒为 0 或恒为满量程；两端 3V3/GND 接反 → 旋转方向相反。中间值不线性通常也不是代码问题。

---

### 2.4 串口 —— [bsp/serial/bsp_serial.h](../bsp/serial/bsp_serial.h)

USART1，PA9 = TX / PA10 = RX，**115200-8-N-1**，无硬件流控。收发都中断驱动 + 环形缓冲。

参数（波特率、字长、校验、GPIO 模式）全在 CubeMX 里，由 `MX_USART1_UART_Init()` / `HAL_UART_MspInit()` 完成。**本模块不配置任何寄存器**，只负责武装接收中断与缓冲管理。

```c
#include "bsp_serial.h"
```

**常量**

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `BSP_SERIAL_TX_BUF_SIZE` | 256 | 发送环形缓冲。**必须是 2 的幂** |
| `BSP_SERIAL_RX_BUF_SIZE` | 256 | 接收环形缓冲。**必须是 2 的幂** |

两处都有编译期 `#error` 拦着（内部用 `& (SIZE-1)` 回绕）。缓冲留一格区分空/满，所以**实际可用 255 字节**。

**接口**

| 接口 | 签名 | 返回 | 阻塞 |
| --- | --- | --- | --- |
| 初始化 | `error_t bsp_serial_init(void)` | `ERR_OK` / `ERR_NOT_READY` | 无 |
| 发送 | `uint32_t bsp_serial_write(const uint8_t *data, uint32_t len)` | 实际放入的字节数；缓冲满时小于 `len`；未初始化或 `data == NULL` 时返回 0 | **无** |
| 接收 | `uint32_t bsp_serial_read(uint8_t *out, uint32_t max)` | 实际取到的字节数；空缓冲返回 0（**不是错误**） | **无** |
| 待读量 | `uint32_t bsp_serial_rx_available(void)` | 缓冲里已积压的字节数 | 无 |
| 可写量 | `uint32_t bsp_serial_tx_free(void)` | 还能再放多少字节（最大 255） | 无 |
| 丢包数 | `uint32_t bsp_serial_rx_dropped(void)` | 因缓冲满而丢弃的字节数。**正常恒为 0** | 无 |
| 冲刷 | `error_t bsp_serial_flush(uint32_t timeout_ms)` | `ERR_OK` / `ERR_TIMEOUT` / `ERR_NOT_INITIALIZED` | **是，最长 `timeout_ms`** |

**语义要点**

- **发送非阻塞，但「返回」≠「已发出」**。`bsp_serial_write` 只把数据拷进缓冲就返回，真正往硬件塞字节由发送中断做。要保证发完，后面接 `bsp_serial_flush()`：

  ```c
  (void)bsp_serial_write(buf, n);
  (void)bsp_serial_flush(100U);     /* 收尾/复位前用；不要在周期任务里调 */
  ```

- **`bsp_serial_flush` 是本模块唯一的阻塞函数**，只在收尾 / 调试验证场合用（例如复位前确保日志已发出）。
- **单生产者 / 单消费者约束**：发送缓冲的 `head` 只允许一个上下文写。所以 **`bsp_serial_write` 只能从主循环一处调用**，`bsp_serial_read` 只能由一个消费者调用。同时从主循环和中断调用会破坏 SPSC 前提，数据会乱。
- **`printf` 可以直接用**（newlib 的 `_write()` 已重定向到发送环形缓冲）：

  ```c
  printf("rp1=%u rp2=%u\r\n", bsp_pot_raw(POT_ID_RP1), bsp_pot_raw(POT_ID_RP2));
  ```

  三条硬约束：
  1. **缓冲满时静默截断**（返回短计数），不会阻塞等待。要保证不丢，先 `bsp_serial_tx_free()` 看空间。
  2. **`%f` 不可用**（链接选项没开 `-u _printf_float`，开了多占约 6 KB Flash）。要打浮点先自己放大成整数。
  3. **绝对不要在中断里调 `printf`**——它不重入，格式化耗时也不可控。

  ⚠️ 它拉进约 10 KB 的 newlib 格式化代码，Flash 只有 64 KB。

- **没有「读一行」接口，这是有意的**：串口是**字节流**，一次 `read` 可能只拿到半个报文，也可能拿到两条半。驱动不假装「一次调用 = 一条消息」。上层要自己维护行缓冲按分隔符切分：

  ```c
  static char    line[64];
  static uint8_t n = 0;
  uint8_t        chunk[32];

  const uint32_t got = bsp_serial_read(chunk, (uint32_t)sizeof(chunk));
  for (uint32_t i = 0U; i < got; i++) {
      if (chunk[i] == '\n') {
          line[n] = '\0';
          handle_command(line);            /* 完整的一条 */
          n = 0U;
      } else if (chunk[i] != '\r') {
          if (n < (uint8_t)(sizeof(line) - 1U)) { line[n] = (char)chunk[i]; n++; }
          else { n = 0U; }                 /* 超长报文：丢弃重来，防溢出 */
      }
  }
  ```

- **错误恢复是自动的**：模块已实现 `HAL_UART_ErrorCallback`，处理 ORE / PE / NE / FE 并重新武装接收。没有这段的话，对端热插拔或主循环取数据太慢触发 ORE 后，串口会**永久性地只能发不能收**，且不报任何错。
- **接收缓冲满时丢的是「新来的」字节**，保留先到的——上层至少还能看到一条完整报文的前半段。

**收不到数据时先查**：TX/RX 是否交叉接线、GND 是否共地、波特率是否 115200。只收到第一行之后就没有了 → 检查接收回调里有没有重新武装 `HAL_UART_Receive_IT()`。

---

### 2.5 OLED

这是唯一跨三层的模块。

#### A. 板级 —— [bsp/oled/bsp_oled.h](../bsp/oled/bsp_oled.h)

软件 I2C，SCL = PB8、SDA = PB9，从机地址 `0x3C`。

```c
#include "bsp_oled.h"          /* 已包含 ssd1306.h */
#include "ssd1306_fonts.h"     /* 用字体时才需要 */
```

**常量**

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `BSP_OLED_I2C_ADDR` | `0x3C` | 模块背面 SA0 接地时的 7 位地址 |
| `BSP_OLED_SCAN_MAX` | 16 | 一次扫描最多返回的器件数 |
| `BSP_OLED_SCAN_ADDR_FIRST` / `_LAST` | `0x08` / `0x77` | 扫描范围（跳过 I2C 保留地址） |

**接口**

| 接口 | 签名 | 返回 | 说明 |
| --- | --- | --- | --- |
| 初始化 | `error_t bsp_oled_init(ssd1306_t *dev)` | `ERR_OK` / `ERR_INVALID_PARAM` / `ERR_COMMUNICATION` | 建立总线 + 填 `dev` 的 `bus`/`addr`/`delay_ms` + 调 `ssd1306_init` |
| 总线扫描 | `uint8_t bsp_oled_bus_scan(uint8_t *found, uint8_t max)` | 扫到的器件数 | 排查接线用，正常只应扫到 `0x3C` |
| 延时 | `void bsp_oled_delay_ms(uint32_t ms)` | — | 作为回调注入设备驱动，应用不用直接调 |

**语义要点**

- `ssd1306_t` 实例的存储**由调用者提供**，通常是文件级 `static`（含 1 KB 显存，不能放栈上）：

  ```c
  static ssd1306_t s_oled;
  ```

- 内部有约 **130 ms 阻塞**（100 ms 上电等待 + 一次整屏刷新），只能在调度起来之前调。
- 扫描功能依赖已建立的总线，**必须在 `bsp_oled_init()` 之后调用**（之前调用返回 0）。

#### B. 设备驱动 —— [driver/device/ssd1306/ssd1306.h](../driver/device/ssd1306/ssd1306.h)

**全部状态都在 `ssd1306_t` 实例里，驱动内部零全局变量**，所以同一条总线上可以并存多块屏。

**颜色与字体**

| 名称 | 说明 |
| --- | --- |
| `SSD1306_BLACK` / `SSD1306_WHITE` | 只有「灭」「亮」两态，**没有灰阶** |
| `Font_6x8` | 6×8，每行最多 **21** 字符 |
| `Font_7x10` | 7×10，每行最多 **18** 字符 |
| `Font_11x18` | 11×18，每行最多 **11** 字符 |

三种都是**等宽字体**（`char_width == NULL`），所以上面的「每行字符数」是精确值。要更大字号在 [ssd1306_conf.h](../driver/device/ssd1306/ssd1306_conf.h) 里打开 `SSD1306_INCLUDE_FONT_16x26`（每多开一种就多占 Flash，11x18 已经占 6.3 KB）。

**初始化 / 显示控制**

| 接口 | 签名 | 说明 |
| --- | --- | --- |
| `ssd1306_init` | `error_t (ssd1306_t *dev)` | **由 `bsp_oled_init` 代劳，应用一般不直接调**。调用前须填好 `bus` / `addr` / `delay_ms` |
| `ssd1306_set_display_on` | `error_t (ssd1306_t *dev, bool on)` | 开关显示，**不影响显存内容**（可做「熄灭但保留画面」） |
| `ssd1306_set_contrast` | `error_t (ssd1306_t *dev, uint8_t value)` | 0x00~0xFF，越大越亮；上电默认已设为最大对比度 |

**显存类**（只改本地缓冲，需 `update_screen` 才上屏）

| 接口 | 签名 | 说明 |
| --- | --- | --- |
| `ssd1306_fill` | `void (ssd1306_t *dev, ssd1306_color_t color)` | 整块显存填黑 / 填白 |
| `ssd1306_update_screen` | `error_t (ssd1306_t *dev)` | 按 8 页把 1 KB 显存刷到屏上；任一分页失败即返回 `ERR_COMMUNICATION` |
| `ssd1306_draw_pixel` | `void (ssd1306_t *dev, uint16_t x, uint16_t y, color)` | **越界静默忽略**（画图时不必自己做边界判断） |
| `ssd1306_set_cursor` | `error_t (ssd1306_t *dev, uint16_t x, uint16_t y)` | **越界返回 `ERR_INVALID_PARAM`** |

**文本类**

| 接口 | 签名 | 返回 |
| --- | --- | --- |
| `ssd1306_write_char` | `char (dev, char ch, const SSD1306_Font_t *font, ssd1306_color_t color)` | 成功返回该字符；越界或不可显示返回 0 |
| `ssd1306_write_string` | `uint16_t (dev, const char *str, const SSD1306_Font_t *font, ssd1306_color_t color)` | 实际写入的字符数 |

**图形类**

| 接口 | 签名 | 说明 |
| --- | --- | --- |
| `ssd1306_draw_line` | `void (dev, x1, y1, x2, y2, color)` | Bresenham 直线 |
| `ssd1306_draw_rectangle` | `void (dev, x1, y1, x2, y2, color)` | 矩形边框 |
| `ssd1306_fill_rectangle` | `void (dev, x1, y1, x2, y2, color)` | 实心矩形（内部逐像素，大矩形耗时可观） |

**控制 OLED 必须知道的语义**

- **显存是离屏缓冲，改了不会自动上屏**：

  ```c
  ssd1306_fill(&s_oled, SSD1306_BLACK);
  (void)ssd1306_set_cursor(&s_oled, 0U, 0U);
  (void)ssd1306_write_string(&s_oled, "Hello", &Font_7x10, SSD1306_WHITE);
  (void)ssd1306_update_screen(&s_oled);      /* ← 这一句才真正上屏 */
  ```

- **一次 `update_screen` 实测约 122 ms**（8 页 × 1024 字节显存经软件 I2C；约 100 kHz 下就是 1024×9 bit ÷ 100 kHz ≈ 92 ms，加命令字节与函数开销正好落到这个量级）。⚠️ **旧文档写的「约 30 ms」是错的**——那对应约 300 kHz 的 I2C，这个软件 I2C 达不到。**没有局部刷新接口**，所以 UI 必须自己降频：本工程整屏重画周期取 500 ms（见 [main.c](../Core/Src/main.c) 的 `DISPLAY_PERIOD_MS`），STREAM 期间干脆不刷。
- **`write_string` 不换行，放不下就停**：`write_char` 在「本行放不下」时返回 0，`write_string` 遇到 0 就 `break`。所以换行要**自己调 `ssd1306_set_cursor`**；`'\n'`（0x0A < 32）**不是换行**，属于不可显示字符，会让字符串输出原地终止；超长字符串**静默截断**（返回的字符数小于 `strlen`）。
- **只支持 ASCII 32~126**。中文显示不了——点阵字库里没有，传进去返回 0 并终止整个字符串。
- **绘制字符会擦掉字符框内的背景**（非亮点被写成 `!color`）。所以「先画进度条、再把数字写上去」会挖出一个黑洞，正确顺序是**先文字后图形**，或让图形避开文字区域。
- 想让数值右对齐不左右跳动，得自己补空格——见 [main.c](../Core/Src/main.c) 里 `oled_u32_to_dec()` + 补空格的写法。

#### C. 总线层 —— [driver/bus/i2c/i2c_soft.h](../driver/bus/i2c/i2c_soft.h)

应用一般碰不到，但要知道存在：

| 接口 | 签名 | 说明 |
| --- | --- | --- |
| `i2c_soft_init` | `error_t (i2c_soft_t *bus, const i2c_soft_cfg_t *cfg)` | 引脚配置成开漏输出并拉高。**调用前须已使能对应 GPIO 端口时钟**（BSP 层负责） |
| `i2c_soft_iface` | `i2c_if_t *(i2c_soft_t *bus)` | 取出抽象接口；入参为空或未初始化返回 NULL |
| `i2c_soft_bus_recover` | `error_t (i2c_soft_t *bus)` | 总线卡死恢复：补发 9 个时钟 + 一个停止条件。`ERR_OK` 已释放 / `ERR_BUSY` 仍被拉低 |

`I2C_SOFT_DEFAULT_FREQ_HZ = 400000`。想改 OLED 刷新速度就改这个（或 BSP 里 `bus_cfg.freq_hz`）。

⚠️ `i2c_soft_bus_recover()` **目前没有任何代码调用**。若遇到「屏幕突然全黑、复位才好」，第一件事就是接一个按键去调它验证。

⚠️ PB8/PB9 的电气参数**配了两遍**（`.ioc` 一遍、`i2c_soft_init()` 一遍），这是本工程唯一有意保留的双份配置：`.ioc` 负责占位与冲突检测，总线驱动负责自包含。**改一处就要改另一处**。其余引脚一律只听 CubeMX 的。

---

### 2.6 电机 —— [bsp/motor/bsp_motor.h](../bsp/motor/bsp_motor.h)

TB6612FNG 的 A 路，驱动 25GA370 直流电机。**20 kHz PWM**（TIM2_CH1，**PSC=1 / ARR=1799**）。

| 功能 | 引脚 |
| --- | --- |
| PWM（速度） | **PA0** = TIM2_CH1 = PWMA |
| 方向 | **PB12 / PB13** = AIN1 / AIN2 |
| STBY | 板上直接接 3V3，**代码不用管** |

```c
#include "bsp_motor.h"
```

**常量**

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `BSP_MOTOR_DUTY_MAX` | 1800 | duty 上限，同时也是 100% 占空比（分辨率 1/1800 = 0.056%） |
| `BSP_MOTOR_PWM_ARR` | 1799 | **与 `.ioc` 里 TIM2 的 ARR 绑死**，改一处要改两处 |

**接口**

| 接口 | 签名 | 返回 | 说明 |
| --- | --- | --- | --- |
| 初始化 | `error_t bsp_motor_init(void)` | `ERR_OK` / `ERR_NOT_READY` | 方向脚置 0、CCR 置 0，**最后才** `HAL_TIM_PWM_Start()` |
| 设转速 | `error_t bsp_motor_set_duty(int16_t duty)` | `ERR_OK` / `ERR_NOT_INITIALIZED` | −1800~+1800，**超出夹紧**（见下） |
| 滑行 | `error_t bsp_motor_coast(void)` | 同上 | AIN1=AIN2=0，输出高阻，**惯性滑行** |
| 刹车 | `error_t bsp_motor_brake(void)` | 同上 | AIN1=AIN2=1，绕组短接，**电磁刹车** |
| 读回 | `int16_t bsp_motor_duty(void)` | −1800~+1800 | 读**夹紧后**的实际命令值 |

**语义要点**

- **duty 的量纲是「写进 CCR 的原始值」**：因为 ARR=1799，占空比 = `CCR/(ARR+1)` = `CCR/1800`，所以 `duty` 直接当 CCR 写、无需换算，**1800 对应 100% 而不是 100**。这个前提绑在 `BSP_MOTOR_PWM_ARR` 上。
- ⚠️ **参考工程的 PID 增益不能照抄**：参考工程的 PWM 量程是 100（ARR=99），本工程是 1800，**差 18 倍**。`control.c` 的默认增益已经乘过 18（见 [motor.md](motor.md#23-为什么-pwm-频率选-20-khz)）。
- **越界是夹紧，不返回错误**：调用者主要是 PID 控制环，其输出瞬态越界属正常现象；此时让电机**停摆**比夹紧到边界危险得多（倒立摆失去力矩会直接倒）。需要严格边界检查的场合请调用前自行判断。
- **`coast()` 与 `set_duty(0)` 等价**（都是 AIN1=AIN2=0 + CCR=0），但**与 `brake()` 电气状态完全不同**——前者电机可自由转动，后者有明显阻力。调试时用手扭输出轴就能区分（见 [motor.md](motor.md#54-第三层方向一致性最要紧的一条)）。
- **`init()` 里的顺序不能调换**：先写方向/CCR，最后 `HAL_TIM_PWM_Start()`，保证上电绝不带非零占空比。
- ⚠️ `HAL_TIM_PWM_Start()` **必须显式调用且只能调一次**。`MX_TIM2_Init()` 只把参数写进寄存器，**不使能输出比较和计数器**——漏掉的现象是「编译烧录全成功、电机纹丝不动」。
- ⚠️ **`duty > 0` 的方向与「编码器正计数」已经实测对齐**（见 2.7）。

---

### 2.7 编码器 —— [bsp/encoder/bsp_encoder.h](../bsp/encoder/bsp_encoder.h)

TIM3 编码器模式，AB 正交**四倍频**。**408 边沿 / 输出轴圈**。

| 功能 | 引脚 |
| --- | --- |
| A 相 | **PA6** = TIM3_CH1 = E1A |
| B 相 | **PA7** = TIM3_CH2 = E1B |

```c
#include "bsp_encoder.h"
```

**常量**

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `BSP_ENCODER_EDGES_PER_REV` | 408 | 四倍频后，输出轴一圈的计数增量 |

**接口**

| 接口 | 签名 | 返回 | 说明 |
| --- | --- | --- | --- |
| 初始化 | `error_t bsp_encoder_init(void)` | `ERR_OK` / `ERR_NOT_READY` | 启动编码器接口 + **把游标对齐到当前 CNT** |
| 采集 | `error_t bsp_encoder_update(void)` | `ERR_OK` / `ERR_NOT_INITIALIZED` | 推进游标、累加位置、记下本段增量 |
| 位置 | `int32_t bsp_encoder_total(void)` | 累计边沿数 | 正负表示方向；未初始化返回 0 |
| 速度 | `int16_t bsp_encoder_delta(void)` | 上次增量 | 除以 update 周期即转速 |
| 清零 | `error_t bsp_encoder_reset(void)` | 同上 | **同时重新对齐游标** |
| 反向 | `error_t bsp_encoder_set_invert(bool invert)` | 同上 | A/B 相接反时的软件兜底，**正常不用** |

**语义要点**

- **接口形状与 `bsp_pot` 一致**：`update()` 对应 `sample()`（采集，有副作用），`total()`/`delta()` 对应 `raw()`（取值，无副作用）。
- ⚠️ **本模块不清零 CNT**，只维护软件游标，增量用**模 2^16 相减**：`(int16_t)(uint16_t)(now - last)`。好处是不丢边沿、采集时机任意、不需要关中断。**代价是 `reset()` 必须显式对齐游标**（否则清零后位置会凭空多出一截）。
- ⚠️ **正确性前提**：两次 `update()` 之间的真实增量 \|d\| < 32768。408 边沿/圈、620 RPM 满速也只有约 4.2k 边沿/秒，**1 ms 采一次是 4 个边沿**，差三个数量级。
- **方向由 `.ioc` 里的 IC1/IC2 极性决定**（当前 IC1=RISING、IC2=FALLING，与参考实现对齐）。`set_invert()` 只是兜底开关。
- **本模块不需要注册节拍任务**（读一个寄存器而已）。当前工程为了显示节奏稳定，在 1 ms 节拍里调了一次 `update()`。
- ⚠️ **电机正转（`duty > 0`）时 `total` 必须增大** —— 已实测确认。**这条错了双环 PID 会变成正反馈。**

---

### 2.8 角度传感器 —— [bsp/angle/bsp_angle.h](../bsp/angle/bsp_angle.h)

倒立摆摆杆角度：**一只 360° 电位器**（不是 MPU6050/AS5600），走 **ADC1_IN8**。

| 功能 | 引脚 |
| --- | --- |
| 角度信号 | **PB0** = ADC1_IN8 = SENSOR1 |

```c
#include "bsp_angle.h"
```

**常量**

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `BSP_ANGLE_RAW_MAX` | 4095 | 12 位 ADC 满量程 |
| `BSP_ANGLE_VREF_MV` | 3300 | 参考电压 |

**接口**

| 接口 | 签名 | 返回 | 阻塞 | 前置条件 |
| --- | --- | --- | --- | --- |
| 初始化 | `error_t bsp_angle_init(void)` | `ERR_OK` / `ERR_NOT_READY` | 约 83 个 ADC 周期 | `MX_ADC1_Init()` 之后 |
| 采样（阻塞） | `error_t bsp_angle_sample(void)` | `ERR_OK` / `ERR_NOT_INITIALIZED` / `ERR_TIMEOUT` | 约 5.7 µs（有界自旋） | `bsp_angle_init()` 之后 |
| 采样（非阻塞·启动） | `error_t bsp_angle_trigger(void)` | `ERR_OK` / `ERR_NOT_INITIALIZED` | 无（几十 ns） | `bsp_angle_init()` 之后 |
| 采样（非阻塞·取值） | `error_t bsp_angle_poll(void)` | `ERR_OK`（拿到新值）/ `ERR_NOT_READY` / `ERR_NOT_INITIALIZED` | 无 | 先 `trigger()` |
| 原始值 | `uint16_t bsp_angle_raw(void)` | 0~4095 | 无 | — |
| 电压 | `uint16_t bsp_angle_millivolt(void)` | 0~3300 mV | 无 | — |

**语义要点**

- ⚠️ **两套接口分工不同，用错上下文会出问题**：
  - `bsp_angle_sample()`（阻塞）**只放主循环**——它要**干等约 5.7 µs**，放 1 ms 节拍里不划算；历史上它更因为走 `HAL_ADC_PollForConversion()` 而在中断里有死等风险（与 `bsp_pot_sample()` 同一条约束）。
  - `bsp_angle_trigger()` / `bsp_angle_poll()`（非阻塞）**可以在中断里用**——纯寄存器操作，启动后立即返回，下次 tick 再来取值。**控制环用的就是这一对。**
- **`poll()` 返回 `ERR_OK` 当且仅当取走了一个新采样**（一次转换只报告一次）。所以「`ERR_OK`」=「拿到新数据」，「`ERR_NOT_READY`」=「还没转完或已取过」。控制环靠这个语义判断数据新鲜度。
- **不需要每次改通道**（只有一个固定通道，配置在 `MX_ADC1_Init()` 里定死），这点与 `bsp_pot` 不同。
- **ADC 在 `init()` 里一次性使能，之后永不 `Stop`**：每次 `HAL_ADC_Start()` 内部都要等一次 ADRDY，那段等待走 `HAL_GetTick()`，中断里没法用。开一次就不关，之后每次转换只置 `CR2.SWSTART`（**必须和 `EXTTRIG` 一起写**——`HAL_ADC_Init()` 不置这一位，只写 SWSTART 会静默无效）。
- ⚠️ **驱动层不提供「角度是多少度」**：竖直零点取决于机械安装相位，属应用层标定。PID 用 `raw - CENTER` 当误差即可，连「度」都不需要。
- **有效角度 333°，两端约有 27° 盲区**，跨过去读数会突变。这是器件特性，**本模块不做补偿**——应用层做范围检查。
- ⚠️ **盲区正对摆杆自然下垂方向**：所以「松手让摆杆垂着」测到的读数（本台实测 **1883**）是**盲区里的无效值**，虽然它稳定不漂。
- **本台设备的实测标定值**（现写在 [control.c](../app/control/control.c) 的 `CONTROL_DEFAULT_CENTER`）：

  | 位置 | `raw` |
  | --- | --- |
  | 摆杆竖直向上（**手扶**） | 2086（重复性 2080~2090） |
  | 水平向左（逆时针 90°） | 960 |
  | 水平向右（顺时针 90°） | 3160 |
  | **自由下垂（松手）** | **1883 —— 盲区无效值，不是竖直** |

  由此得 **12.22 LSB/度**，满量程折算 335°（对上手册的 333°，差 0.6%）。读数随**顺时针转动而增大**。

- ⚠️ **中心值不能用理想的 2048，也不能用「自由下垂」测**。2048 是电位器的**电气中点**，与「摆杆竖直」没有物理必然联系；而「自由下垂」在本套件上读到的是盲区残值（见上）。参考实现给的是**区间**（1900~2200）而非一个数，正说明它要被测出来。用错中心值会让摆杆稳态偏移、并总朝一个方向跑。**正确做法：手扶摆杆到竖直附近读 `raw`，多测几次取平均。**

---

### 2.9 PID 算法 —— [algorithm/pid/pid.h](../algorithm/pid/pid.h)

位置式 PID，**纯算法**：只依赖 `<stdbool.h>`，不含任何 `bsp_*` / HAL 头文件，
因此可以在 PC 上用 gcc 单独编译做单元测试。

**类型**：直接操作 `pid_t` 结构体（参数、限幅、输入输出、运行时状态都在里面）。

| 字段组 | 字段 | 谁写 |
| --- | --- | --- |
| 参数 | `kp` / `ki` / `kd` | 调用者（调参时改） |
| 输出限幅 | `out_min` / `out_max` | 调用者 |
| 积分限幅 | `integ_min` / `integ_max` | 调用者（取 `min==max==0` 等价于禁用积分） |
| 输入 | `target` / `actual` | 调用者 |
| 输出 | `out` | `pid_update()` |
| 运行时状态 | `err0` / `err1` / `integ` | `pid_update()`，**调用者不要动** |

**接口**

| 接口 | 签名 | 说明 |
| --- | --- | --- |
| 复位 | `void pid_reset(pid_t *p)` | 清运行时状态（`err0/err1/integ/out`），**保留参数与限幅**。每次启动控制时必调 |
| 算一步 | `void pid_update(pid_t *p)` | 读 `target`/`actual`，更新 `out`；`p` 为 NULL 时直接返回 |

**语义要点**

- **公式是未归一化到时间的简化形式**（与参考工程一致）：`out = Kp·e + Ki·Σe + Kd·(e − e_prev)`。
  `Ki`/`Kd` 是「每个控制周期」的量纲，**不是「每秒」**。所以
  ⚠️ **改调用周期 = 改 Ki/Kd 的实际效果**（周期减半则 Ki 效果减半、Kd 翻倍）。
  本工程把周期写死成常量（`control.c` 的 `CONTROL_ANGLE_PERIOD_MS`）。详见 [pid.h](../algorithm/pid/pid.h) 文件头。
- `pid_update()` **不含时间概念、不阻塞**，可以在中断里调（`control_tick` 就是这么用的）。
- **`Ki == 0` 时积分被清零**（不是照常累加）：整定时通常先关积分看纯 PD，若积分照常攒，回头把 Ki 调成非 0 会被那个积压值「踹一脚」。这是从参考工程保留的调试便利。
- **积分限幅夹的是积分累加值本身，不是积分项**（`Ki·integ`）：这样限幅范围与 Ki 无关，调参时行为稳定。

---

### 2.10 控制器 —— [app/control/control.h](../app/control/control.h)

旋转倒立摆的双环串级控制器：角度环（内环，5 ms）+ 位置环（外环，50 ms），
外加**状态机**和**倒下保护**。**整个控制环挂在 1 ms 节拍上，运行在中断上下文里。**

> 被控对象、两环为什么这么分工、两个判定窗口的来龙去脉，全写在
> [control.h](../app/control/control.h) 的文件头——那是本工程写得最详细的一份设计说明。

```c
#include "control.h"
```

**常量与类型**

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `CONTROL_POS_TARGET_LIMIT` | 4080 | 位置目标绝对值上限（408 边沿 = 横杆一圈，正反各 10 圈） |
| `control_state_t` | `CONTROL_STATE_STOP` / `CONTROL_STATE_RUN` / `CONTROL_STATE_SWING_UP` | 0 停 / 1 双环 PID / 2 自动启摆中 |
| `control_param_t` | `AKP AKI AKD PKP PKI PKD CENTER RANGE START OFFSET SWP SWT` + `CONTROL_PARAM_COUNT` | 可在线改的参数编号（共 12 个） |
| `control_params_t` | — | 参数集合。**只读**，改参数请走 `control_param_set()` |
| `control_status_t` | — | 一次取齐的状态快照（角度/位置/速度/两环目标与输出/PWM/状态/启摆结局）|
| `control_swing_result_t` | `NONE` / `OK` / `TIMEOUT` / `ABORTED` | 上一次启摆的结局，给串口分「超时停」和「被按停」用 |

**接口**

| 接口 | 签名 | 说明 |
| --- | --- | --- |
| 初始化 | `error_t control_init(void)` | 载默认参数、对齐快照、把电机置滑行。**初始状态是 STOP**；要在电机/编码器/角度三个驱动之后调 |
| 1 ms 任务 | `void control_tick(void)` | **挂在节拍上，中断上下文**。取角度、推进编码器、倒下保护、分频出 5 ms / 50 ms 两环 |
| 启动 | `error_t control_start(void)` | **严格**：角度不在 `CENTER ± START` 内返回 `ERR_NOT_READY`；会清两个环的积分与历史误差 |
| 启动（自动）| `error_t control_swing_up(void)` | 角度已在窗口内 → 等价于 `control_start()`；否则 → 进 `CONTROL_STATE_SWING_UP` 自动启摆。已在 RUN 时幂等返回 |
| 停止 | `error_t control_stop(void)` | 电机**滑行**（不是刹车）；倒下保护内部也走它 |
| 是否运行 | `bool control_is_running(void)` | 唯一的「运行状态」真相，别在别处再存一份 |
| 位置清零 | `error_t control_zero_position(void)` | 位置与位置目标**一起**清零（分两步做容易只做一半） |
| 位置目标 | `int32_t control_position_target(void)` / `error_t control_set_position_target(int32_t)` | 单位是编码器边沿；越界返回 `ERR_INVALID_PARAM` |
| 状态快照 | `void control_get_status(control_status_t *out)` | 一次取齐，避免读到来自不同时刻的值 |
| 参数表 | `const control_params_t *control_params(void)` | 只读 |
| 读写参数 | `error_t control_param_set(control_param_t, float)` / `float control_param_get(control_param_t)` | **在线改参数的唯一正确入口**（带范围检查） |
| 参数名 | `const char *control_param_name(control_param_t)` | 串口层用它拼 `HELP`，避免名字在两处不同步 |

**语义要点**

- ⚠️ **`control_tick()` 在中断里**：全是寄存器操作和几次浮点乘加，**不许阻塞、不许 `HAL_Delay`、不许 `printf`、不许刷屏**。它用的都是中断安全的接口：`bsp_angle_poll/trigger`、`bsp_encoder_update/total/delta`、`bsp_motor_set_duty/coast`。
- ⚠️ **`START` 与 `RANGE` 是两个目的不同的窗口**（见 [control.h](../app/control/control.h) 的「盲区陷阱」）：
  - `START`（默认 150，≈±12°）在**两个时刻**检查：`RUN` 的瞬间，以及**启摆交棒**的瞬间。
    挡的是「摆杆垂着/躺着被当成已经立好」；
  - `RANGE`（默认 500，≈±41°）**运行中每 1 ms** 检查，挡「立着立着倒了」。

  本台实测**自由下垂读数 1883 落在 `RANGE` 内、`START` 外**，所以两个窗口缺一不可。
- ⚠️ **自动启摆**（`CONTROL_STATE_SWING_UP`）：移植自参考工程 `16-倒立摆-自动启摆` 的
  1 ms 状态机——每 40 ms 采样、检测摆动顶点、打一组方向相反的瞬时脉冲（`SWP` 占空比、
  每个 `SWT` 毫秒），把摆杆荡到竖直附近后交给双环。
  期间**两个 PID 环都不算**，`control_is_running()` 返回 false（它问的是"双环在不在跑"）；
  要看总状态请读快照的 `state`。
  - ⚠️ **交棒用的窗口是 `START`（±150）而不是参考的 `RANGE`（±500）**：本台实测
    自由下垂读数 1883 落在 `CENTER ± 500` 里，用 RANGE 会把**垂着的**摆杆判成"已立好"
    而直接交给 PID（默认增益下电机会猛冲）。改用 START 后，**交棒条件 == RUN 的准入条件**。
  - ⚠️ **有 30 秒超时**（参考没有），超时后自动停机并把结局记成 `CONTROL_SWING_TIMEOUT`。
  - 参数 `SWP`（默认 630 = 参考的 35% × 18）与 `SWT`（默认 100 ms）**是按设备标定的量**，
    可在线改。
- `control_param_set()` 会挡 NaN / inf，并对 `CENTER`(≤4095) / `RANGE`(1~2048) / `START`(1~2048) / `OFFSET`(0~max) 做范围检查。**`RANGE = 0` 会退化成「角度永远不在窗口内」，所以下限是 1。**
- 并发：参数是 32 位对齐的 float，Cortex-M3 上单条 `STR` 写入不会撕裂，且一次只改一个字段，**故不加临界区**（理由写在 `control.c`）。

---

### 2.11 串口控制台 —— [app/console/console.h](../app/console/console.h)

ASCII 行协议，**给「调参 agent」用的遥控器**。协议规格完整写在 [console.h](../app/console/console.h) 文件头。

```c
#include "console.h"
```

**接口**

| 接口 | 签名 | 说明 |
| --- | --- | --- |
| 初始化 | `error_t console_init(void)` | 清行缓冲。要在 `bsp_serial_init()` 之后 |
| 主循环任务 | `void console_poll(void)` | 收字节、拆行、执行命令、发响应、必要时吐 STREAM 数据 |
| 是否在串流 | `bool console_is_streaming(void)` | 主循环据此**暂停刷屏**（STREAM 开着时屏没人在看，把 CPU 让给串口） |

**协议一览**（`HELP` 会自己列出来，字段从枚举动态生成，不会和实现对不上）

```
SET <参数名> <值>      → OK <名>=<值>
GET <名> | GET ALL     → OK ...
STAT                   → OK ANGLE=.. POS=.. SPD=.. ATAR=.. AOUT=.. POUT=..
                            PWM=.. RUN=.. ST=.. SWR=..
RUN                    → OK RUN=1        角度不在 START 窗口内则 ERR NOT_READY
SWING                  → OK ST=1|2       在窗口内等价于 RUN，否则先自动启摆
STOP                   → OK RUN=0
ZERO                   → OK POS=0 TARGET=0
TARGET <整数>           → OK TARGET=<n>
STREAM <0|1>           → OK STREAM=<n>   （按 20 ms 周期吐一行 CSV，列序与 STAT 的前 8 列一一对应）
HELP                   → OK CMD=.. PARAM=..
```

`RUN` 与 `SWING` 的分工：

| | 角度已在 `CENTER ± START` 内 | 不在窗口内 |
| --- | --- | --- |
| `RUN` | 进双环 PID，`OK RUN=1` | **拒绝**，`ERR NOT_READY ANGLE_OUT_OF_WINDOW` |
| `SWING` | 同 `RUN`，`OK ST=1` | **自动启摆**，`OK ST=2`，荡进窗口后自动转成双环 |

保留两套而不是让 `RUN` 自动回退，是为了让「角度不对就该拒」这条已验证的判断
仍能被单独测到，也让"会真的甩电机"必须被调用者**显式**要求。

`STAT` 末尾两个状态字段：`ST`（0 停 / 1 双环 / 2 启摆中）与
`SWR`（上次启摆结局：0 没启摆过 / 1 成功 / 2 超时 / 3 被 STOP 打断）。
**`RUN` 只有 0/1，分不出「停着」和「正在启摆」，所以判断状态要用 `ST`。**

响应只有 `OK ` / `ERR ` 两种前缀；`ERR` 后面**必定带原因和出错的那个词**
（`UNKNOWN_PARAM` / `BAD_VALUE` / `OUT_OF_RANGE` / `UNKNOWN_CMD` / `USAGE` / `LINE_TOO_LONG`）——
只回错误码会让 agent 反复重试同一个错误命令。

**语义要点**

- **命令只在主循环里跑**：会调 `control_start()` 等接口，**不要放进中断**。
- ⚠️ **发一条、等一条响应，再发下一条**。OLED 整屏刷新实测约 122 ms，这期间 `console_poll()` 完全拿不到 CPU；115200 下 122 ms 能进来约 1400 字节，而接收环形缓冲只有 256 字节——**连发必定丢，而且丢得不声不响**。
- ⚠️ **`RUN` 会让电机真的转**。想在不扰动机构的前提下测逻辑，把增益和 `OFFSET` 全设 0——控制环照常跑（分频、采样、倒下判定都执行），但输出恒为 0。
- ⚠️ **`RUN` 前必须手扶摆杆到竖直附近**：`RUN` 时用 `START` 检查当前角度，不在窗口内回 `ERR NOT_READY ANGLE_OUT_OF_WINDOW`。**agent 应当在 `RUN` 之前先 `STAT` 确认 `ANGLE` 靠近 `CENTER`**，而不是假设它一定在。**不想手扶就用 `SWING`** —— 它自己把摆杆荡进这个窗口。
- ⚠️ **`SWING` 也会让电机真的转**（比 `RUN` 更凶：按 `SWP` 的占空比反复打脉冲）。要验证状态机而不扰动机构，把 `SWP` 设成 0 —— 状态机照常跑，但每个脉冲的占空比都是 0（`console_test.py` 第 8b 组就是这么做的）。
- `STREAM 1` 期间主循环停刷屏，换来满速 50 行/秒、零空洞的等间距采样；`STREAM 0` 立刻恢复刷屏。

---

## 三、调用上下文总表

| 接口 | 主循环 | 1 ms 节拍任务 | 阻塞时长 | 备注 |
| --- | --- | --- | --- | --- |
| `bsp_tick_register` | ✅（初始化期） | ❌ | 无 | 要在单线程阶段注册 |
| `bsp_key_take_event` | ✅ | ✅（不推荐） | 几十 ns（关中断） | 读取即清除 |
| `bsp_key_is_pressed` | ✅ | ✅ | 无 | 读消抖后电平 |
| `bsp_pot_sample` | ✅ | **❌** | 约 23 µs | 依赖 `HAL_GetTick`，中断里会死等 |
| `bsp_pot_raw` / `bsp_pot_millivolt` | ✅ | ✅ | 无 | 纯内存 |
| `bsp_serial_write` | ✅ | ❌ **仅一个上下文** | 无 | 拷完就返回；满则截断 |
| `bsp_serial_read` | ✅ | ❌ **仅一个上下文** | 无 | 有多少取多少 |
| `bsp_serial_rx_available` / `tx_free` / `rx_dropped` | ✅ | ✅ | 无 | 只读诊断量 |
| `bsp_serial_flush` | ✅（收尾） | ❌ | 最长 `timeout_ms` | 唯一阻塞函数 |
| `printf` | ✅ | **❌ 绝对不行** | 无（但格式化耗 CPU） | 不重入 |
| `ssd1306_fill` / `draw_*` / `write_*` | ✅ | ❌ | 无 | 只改显存 |
| `ssd1306_update_screen` | ✅ | ❌ | **约 122 ms** | 8 页 I2C 传输 |
| `ssd1306_set_display_on` / `set_contrast` | ✅ | ❌ | 一个 I2C 帧 | 屏没应答会返回错误 |
| `bsp_motor_set_duty` / `coast` / `brake` | ✅ | ✅（不推荐） | 无（几次寄存器写） | 纯寄存器操作，中断安全 |
| `bsp_motor_duty` | ✅ | ✅ | 无 | 只读缓存 |
| `bsp_encoder_update` | ✅ | ✅ | 无（读一个寄存器） | **本工程就是在 1 ms 节拍里调的** |
| `bsp_encoder_total` / `delta` | ✅ | ✅ | 无 | 只读缓存 |
| `bsp_encoder_reset` / `set_invert` | ✅ | ❌ | 无 | 会改模块状态，别在中断里做 |
| `bsp_angle_sample` | ✅ | ❌ | 约 5.7 µs | 有界自旋、不依赖 `HAL_GetTick`；但干等 5.7 µs 不划算，别放节拍里 |
| `bsp_angle_trigger` / `bsp_angle_poll` | ✅ | ✅ | 无（纯寄存器） | **控制环用的就是这一对** |
| `bsp_angle_raw` / `millivolt` | ✅ | ✅ | 无 | 纯内存 |
| `control_tick` | ❌（由节拍调用） | ✅ | 无 | 控制环本体；内部只用中断安全的接口 |
| `control_start` / `stop` / `zero_position` / `set_position_target` | ✅ | ❌ | 无 | 会改状态机与 PID 内部状态 |
| `control_get_status` / `is_running` / `params` / `param_get` | ✅ | ✅ | 无 | 只读快照 |
| `control_param_set` | ✅ | ❌ | 无 | 写参数；中断里**读**它是正常的（见 2.10 的并发说明） |
| `console_poll` | ✅ | ❌ | 无 | 会调 `control_start()`、拼长响应 |
| `pid_update` / `pid_reset` | ✅ | ✅ | 无 | 纯计算，无阻塞 |

---

## 四、主循环最小骨架

**核心纪律：对时间敏感的事挂在 1 ms 节拍（中断）里，重活留在主循环。**

本工程的实际骨架（[Core/Src/main.c](../Core/Src/main.c)）：

```c
/* ── 初始化（顺序见 1.4）───────────────────────────────── */
/* ... MX_xxx_Init() ... */
if (bsp_oled_init(&s_oled)          != ERR_OK) { app_fatal_blink(1U);  }
if (bsp_tick_init()                 != ERR_OK) { app_fatal_blink(2U);  }
if (bsp_key_init()                  != ERR_OK) { app_fatal_blink(3U);  }
if (bsp_pot_init()                  != ERR_OK) { app_fatal_blink(4U);  }
if (bsp_serial_init()               != ERR_OK) { app_fatal_blink(5U);  }
if (bsp_motor_init()                != ERR_OK) { app_fatal_blink(6U);  }  /* 先停机，最安全 */
if (bsp_encoder_init()              != ERR_OK) { app_fatal_blink(7U);  }
if (bsp_angle_init()                != ERR_OK) { app_fatal_blink(8U);  }
if (control_init()                  != ERR_OK) { app_fatal_blink(10U); }
if (console_init()                  != ERR_OK) { app_fatal_blink(11U); }
if (bsp_tick_register(control_tick) != ERR_OK) { app_fatal_blink(9U);  }  /* 还剩 2 个槽 */

/* ── 主循环：一律「查时间、时间到了才做」，不写 HAL_Delay ── */
while (1)
{
    console_poll();         /* 串口命令 + STREAM 上报 */
    app_handle_keys();      /* 按键：每个键都要 while 取干净 */
    app_refresh_display();  /* 每 500 ms 整屏重画；STREAM 期间跳过 */

    /* 心跳 LED */
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - last_led_ms) >= 500U) { last_led_ms = now_ms; /* toggle PC13 */ }
}
```

**主循环里没有 `HAL_Delay`**：在循环里死等会让所有周期性动作互相拖累
（比如屏幕每 500 ms 刷一次，若用延时定时，串口命令的响应周期就变成了「刷屏 + 延时」）。
一律用「查时间、时间到了才做」的非阻塞写法。

**主循环里也**没有**「把控制结果写到电机」这一步**——那是 `control_tick()` 在中断里做的，
主循环里再写一次只会和它打架。

**PID 控制环放在 1 ms 节拍里**（`control_tick`），内部再分频到 5 ms / 50 ms。两个方案的对比：

| 方案 | 周期抖动 | 可用接口 |
| --- | --- | --- |
| **放 1 ms 节拍里（本工程）** | 精确 | **只能**用第三节标「✅ 中断」的接口。`bsp_angle_trigger/poll`、`bsp_encoder_*`、`bsp_motor_set_duty/coast` 都可以；`bsp_angle_sample()` / `bsp_pot_sample()` **不行** |
| 放主循环按时间戳定时 | 受刷屏影响（5~135 ms 乱跳） | 全部可用 |

**为什么不能放主循环**：主循环刷一屏 OLED 实测约 122 ms，而角度环周期是 5 ms——
放进去等于给控制器灌了一路巨大的周期噪声，摆杆根本立不住
（见 [control.h](../app/control/control.h)）。
**代价是控制环跑在中断里，必须遵守「快进快出」的规矩**：不许阻塞、不许打印、不许刷屏。

这也正是 `bsp_angle` 要提供 `trigger()`/`poll()` 一对非阻塞接口的原因——
阻塞版要干等 5.7 µs，放进 1 ms 节拍里没有意义（见 2.8）。

节拍槽位目前用了 2 个（按键 + 控制环），**还剩 2 个**。

---

## 五、陷阱清单

按「踩了会浪费多少时间」排序：

1. **`bsp_pot_sample()` 不能进中断**——它内部 `HAL_ADC_PollForConversion` 的超时依赖 `HAL_GetTick()`，而 TIM1（优先级 1）比 SysTick（优先级 15）高，在节拍任务里调用会**永久死等**。
2. **按键事件必须 `while` 取干净**——PRESS/RELEASE 同时挂起时只拿得到前者；而且**事件是标志位不是队列**，短时间连按会塌缩。
3. **`printf` 缓冲满时静默截断**，不是阻塞等待。别拿它做「必须送达」的通道。
4. **`bsp_serial_write` / `bsp_serial_read` 是单生产者单消费者**，只能各从一个上下文调。
5. **`ssd1306_update_screen` 实测约 122 ms，且没有局部刷新接口**。UI 必须自己降频（本工程整屏重画周期 500 ms）；「内容变了就重画」会让主循环被刷屏彻底堵死。
6. **`write_string` 不换行、遇不可显示字符就终止**——`'\n'` 不是换行，中文显示不了，超长静默截断。
7. **绘制字符会擦背景**——先文字后图形。
8. **`BSP_TICK_PERIOD_MS` 与 `.ioc` 里 TIM1 的 PSC/ARR 绑死**——改定时器不同步改宏，按键消抖时间会**静默**算错。
9. **`HAL_TIM_PeriodElapsedCallback` 只能有一份非弱定义**——以后加定时器要在 [bsp_tick.c](../bsp/tick/bsp_tick.c) 里补分支，不要另写同名函数。
10. **节拍槽位只剩 2 个**（上限 4，按键 + 控制环已占 2）。
11. **串口 / ADC 的参数都在 CubeMX 里**——不要在 BSP 里重配波特率或 ADC 通道参数，那会和 `.ioc` 争夺同一份事实。
12. **`bsp_pot_sample()` 失败时缓存是混合状态**——必须整批丢弃，不要用 `bsp_pot_raw()` 的值。
13. **PB8/PB9 的电气参数配了两遍**（`.ioc` + `i2c_soft_init()`），改一处要改另一处。
14. **OLED 的 `'\0'` 之外的不可显示字符会让字符串输出提前结束**，不是被跳过。
15. **`HAL_TIM_PWM_Start()` 必须显式调用**——`MX_TIM2_Init()` 只写寄存器，不使能输出比较和计数器。漏掉的现象是「配置/编译/烧录全成功、电机纹丝不动」。且**只能调一次**，重复调用会因状态是 BUSY 而返回错误。
16. **`HAL_TIM_Encoder_Start()` 必须传 `TIM_CHANNEL_ALL`**——传单个通道只使能一个 `CCxE`，四倍频会**静默少计一半**（编译运行都不报错）。
17. **电机正转（`duty > 0`）必须对应编码器 `total` 增大**。这条错了双环 PID 会变成**正反馈**，现象是「参数怎么调都发散」。**必须用硬件实测确认，不能靠推导**——已实测通过。
18. **编码器模式与输入捕获模式的参数键名不同**（`IC1Polarity` vs `ICPolarity_1`），**且合法取值是两套枚举**。写错时命令回 OK、roundtrip 也过，只有去生成的代码里数字段才能发现。
19. **`bsp_encoder_reset()` 必须重新对齐软件游标**，只清 `s_total` 会让清零后的位置凭空多出一截。
20. **`bsp_angle` 的中心值不能用理想的 2048，也不能用「自由下垂」测**——2048 是电位器**电气中点**，与「摆杆竖直」无关；而盲区正对下垂方向，松手读到的 1883 是**无效值**（稳定 ≠ 有效）。**正确做法：手扶摆杆到竖直附近读 `raw`，多测几次取平均。**
21. **`bsp_angle` 的采样要按上下文选接口**——阻塞的 `bsp_angle_sample()` 干等 5.7 µs，别放节拍里；**中断里用 `bsp_angle_trigger()` + `bsp_angle_poll()`**（纯寄存器，不依赖 `HAL_GetTick`）。两者共用同一个缓存，可混用。
22. **`bsp_angle_trigger()` 必须把 `EXTTRIG` 和 `SWSTART` 一起写**——`HAL_ADC_Init()` 不置 `EXTTRIG`。只写 SWSTART 的后果是**不报错、不置标志、读数永远不变**。
23. **`bsp_motor_set_duty()` 越界是夹紧、不报错**（有意的设计取舍），调用者若依赖错误码做边界检查会落空。
24. **控制环若发散，先查方向符号、别急着调增益**——`control.c` 里 `AnglePID.Target = CENTER − LocationPID.Out` 的那个负号，由「电机转向 ↔ 编码器计数方向」和「摆杆倾斜 ↔ 读数方向」两组约定共同决定。符号错了整个环就是**正反馈**，现象是「参数怎么调都发散」。
25. **`START` 与 `RANGE` 两个窗口不能合并**——只用 `START`（±150）会让正常运行中的摆动被误停机，调参根本做不下去；只用 `RANGE`（±500）挡不住自由下垂的 1883（它落在窗口内）。
26. **改 PWM 量程（`.ioc` 的 ARR）必须同步改 `BSP_MOTOR_DUTY_MAX`**，并重算所有增益——参考工程的增益是按 ARR=99 的，本工程 ×18。
