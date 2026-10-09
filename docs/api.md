# 接口速查（API Reference）

本文是**查阅用**的速查表：每个模块向应用层暴露了哪些函数、什么前置条件、能不能在中断里调、值域是多少。

**各模块的实现原理与踩坑记录不在本文**，见对应详解：

| 模块 | 原理详解 |
| --- | --- |
| OLED | [oled.md](oled.md) |
| 按键 | [key.md](key.md) |
| 电位器 | [pot.md](pot.md) |
| 串口 | [serial.md](serial.md) |
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
- [三、调用上下文总表](#三调用上下文总表)
- [四、主循环最小骨架](#四主循环最小骨架)
- [五、陷阱清单](#五陷阱清单)

---

## 一、全局约定

### 1.1 分层与依赖方向

```
应用层   Core/main.c（以后还会有 app/）
           │  只能往下调，不允许被下层反向调用
BSP 层   bsp/{tick,key,pot,serial,oled}/      「这块板上什么接在哪个脚」
           │  可以碰 HAL、可以碰 CubeMX 生成的引脚宏
设备层   driver/device/ssd1306/                只认 i2c_if_t，零 HAL 符号
总线层   driver/bus/i2c/i2c_soft.*             贴着 HAL，允许 HAL_GPIO_*
公共层   driver/common/error.h                 全工程错误码
```

应用层只应 include `bsp_*.h`。唯一例外是 OLED 的字库表：需要额外 include [ssd1306_fonts.h](../driver/device/ssd1306/ssd1306_fonts.h)（`ssd1306.h` 已经由 `bsp_oled.h` 带进来）。

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

### 1.3 句柄与引脚宏

**句柄**：CubeMX 只在 `main.c` 里**定义** `hadc2` / `htim1` / `huart1`，**不给 `main.h` 生成 extern**。所以每个 BSP 在自己的 `.c` 里补一份：

| 符号 | 类型 | 声明处 |
| --- | --- | --- |
| `htim1` | `TIM_HandleTypeDef` | [bsp/tick/bsp_tick.c:35](../bsp/tick/bsp_tick.c#L35) |
| `hadc2` | `ADC_HandleTypeDef` | [bsp/pot/bsp_pot.c:55](../bsp/pot/bsp_pot.c#L55) |
| `huart1` | `UART_HandleTypeDef` | [bsp/serial/bsp_serial.c:37](../bsp/serial/bsp_serial.c#L37) |

⚠️ 应用层**不要重复定义**这三个符号，也不要改动它们。

**引脚宏**（由 CubeMX 生成在 [main.h](../Core/Inc/main.h)）：

| 宏 | 引脚 | 归谁用 |
| --- | --- | --- |
| `LED_Pin` / `LED_GPIO_Port` | PC13 | 应用层（心跳 / 错误闪烁） |
| `KEY1..KEY4_Pin` / `_GPIO_Port` | PB10 / PB11 / PA11 / PA12 | 仅 bsp_key.c 内部 |
| `OLED_SCL_Pin` / `OLED_SDA_Pin` | PB8 / PB9 | 仅 bsp_oled.c 内部 |

⚠️ **ADC 通道和 USART1 的引脚 CubeMX 不生成宏**（不像 GPIO 会给 `KEY1_Pin`）。电位器的通道对应表写在 [bsp_pot.c](../bsp/pot/bsp_pot.c) 里，那正是 BSP 层的职责。

### 1.4 初始化顺序（硬依赖，不能乱）

| # | 调用 | 前置条件 | 违反了会怎样 |
| --- | --- | --- | --- |
| 1 | `HAL_Init()` | — | 无 SysTick，`HAL_Delay` 死锁 |
| 2 | `SystemClock_Config()` | — | 72 MHz 与 ADC 时钟分频都靠它 |
| 3 | `MX_GPIO_Init()` | — | 引脚浮空；**GPIOB 时钟也在这里使能**，OLED 靠它 |
| 4 | `MX_TIM1_Init()` | — | 只写寄存器，定时器还没跑（要 `bsp_tick_init` 才启动） |
| 5 | `MX_ADC2_Init()` | — | 通道 / 采样时间 / 扫描模式没配 |
| 6 | `MX_USART1_UART_Init()` | — | 波特率、字长没配 |
| 7 | `bsp_oled_init(&s_oled)` | 步骤 3 | GPIOB 时钟没开 → 屏幕无应答 |
| 8 | `bsp_tick_init()` | 步骤 4 | 节拍不跑 |
| 9 | `bsp_key_init()` | 步骤 3 **和** 8 | 无 tick：注册失败；无 GPIO：读到浮空电平，**开机会凭空多一次按下事件** |
| 10 | `bsp_pot_init()` | 步骤 5 | 自校准要求 ADC 关闭（ADON=0），此刻正好满足 |
| 11 | `bsp_serial_init()` | 步骤 6 | 武装接收中断时串口还没配好 |

`bsp_tick_register()` 在步骤 8 之后随时可调，槽位上限见 2.1。

> **唯一一条「顺序反了也返回成功、但行为是错的」是第 9 条**——其余顺序错了都会在 `_init` 返回错误码，能立刻看见。这也是它最值得记的原因。

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
| `BSP_TICK_MAX_HANDLERS` | 4 | 任务槽位上限。**按键已占 1 个**，还剩 3 个 |
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

- **一次 `update_screen` 约 30 ms**（8 页 × 129 字节经软件 I2C；目标速率 400 kHz，实际受 GPIO 翻转开销影响会略低）。**没有局部刷新接口**，所以 UI 必须自己做死区或降频——「改动就整屏重刷」会让静止时也以 50 Hz 空刷。
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
| `ssd1306_update_screen` | ✅ | ❌ | **约 30 ms** | 8 页 I2C 传输 |
| `ssd1306_set_display_on` / `set_contrast` | ✅ | ❌ | 一个 I2C 帧 | 屏没应答会返回错误 |

---

## 四、主循环最小骨架

**核心纪律：节拍任务只做「计数 + 置标志」，所有重活留在主循环。**

```c
/* ── 文件级状态 ─────────────────────────────────────────── */
static ssd1306_t         s_oled;
static volatile uint32_t s_tick_count;          /* 1 ms 任务累加，主循环读 */

/* ── 1 ms 任务：只累加计数，别的都别干 ───────────────────── */
static void app_1ms_task(void)
{
    s_tick_count++;                             /* 32 位变量，单条指令，读写安全 */
}

/* ── 初始化（顺序见 1.4）───────────────────────────────── */
if (bsp_oled_init(&s_oled)             != ERR_OK) { app_fatal_blink(1U); }
if (bsp_tick_init()                    != ERR_OK) { app_fatal_blink(2U); }
if (bsp_key_init()                     != ERR_OK) { app_fatal_blink(3U); }
if (bsp_pot_init()                     != ERR_OK) { app_fatal_blink(4U); }
if (bsp_serial_init()                  != ERR_OK) { app_fatal_blink(5U); }
if (bsp_tick_register(app_1ms_task)    != ERR_OK) { app_fatal_blink(6U); }  /* 还剩 2 个槽 */

/* ── 主循环：一律「查时间、时间到了才做」，不写 HAL_Delay ── */
while (1)
{
    /* 1. 按键：必须 while 取干净 */
    for (uint8_t i = 0U; i < (uint8_t)KEY_ID_COUNT; i++) {
        key_event_t ev;
        while ((ev = bsp_key_take_event((key_id_t)i)) != KEY_EVENT_NONE) {
            if (ev == KEY_EVENT_PRESS) { app_on_key_press((key_id_t)i); }
        }
    }

    /* 2. 电位器：20 ms 一次，失败则整批丢弃 */
    static uint32_t last_pot_ms;
    const uint32_t  now = HAL_GetTick();
    if ((now - last_pot_ms) >= 20U) {
        last_pot_ms = now;
        if (bsp_pot_sample() == ERR_OK) {
            app_pid_set_setpoint((int32_t)bsp_pot_raw(POT_ID_RP1));
        }
    }

    /* 3. 串口：有多少取多少，喂给行解析器 */
    app_serial_poll();

    /* 4. OLED：按脏标记/死区刷，别每圈都刷 */
    if (app_ui_dirty()) { app_ui_redraw(); }
}
```

**PID 控制环放哪**——两个选择：

| 方案 | 周期抖动 | 可用接口 |
| --- | --- | --- |
| 放 1 ms 节拍任务里 | 精确 | **只能**用第三节标「✅ 中断」的接口。`bsp_pot_raw()` 可以，`bsp_pot_sample()` **不行**（会死等） |
| 放主循环按 `s_tick_count` 定时 | 受刷屏影响 | 全部可用 |

现阶段推荐后者：先把算法调通，再谈抖动。

---

## 五、陷阱清单

按「踩了会浪费多少时间」排序：

1. **`bsp_pot_sample()` 不能进中断**——它内部 `HAL_ADC_PollForConversion` 的超时依赖 `HAL_GetTick()`，而 TIM1（优先级 1）比 SysTick（优先级 15）高，在节拍任务里调用会**永久死等**。
2. **按键事件必须 `while` 取干净**——PRESS/RELEASE 同时挂起时只拿得到前者；而且**事件是标志位不是队列**，短时间连按会塌缩。
3. **`printf` 缓冲满时静默截断**，不是阻塞等待。别拿它做「必须送达」的通道。
4. **`bsp_serial_write` / `bsp_serial_read` 是单生产者单消费者**，只能各从一个上下文调。
5. **`ssd1306_update_screen` 约 30 ms，且没有局部刷新接口**。UI 必须自己做死区。
6. **`write_string` 不换行、遇不可显示字符就终止**——`'\n'` 不是换行，中文显示不了，超长静默截断。
7. **绘制字符会擦背景**——先文字后图形。
8. **`BSP_TICK_PERIOD_MS` 与 `.ioc` 里 TIM1 的 PSC/ARR 绑死**——改定时器不同步改宏，按键消抖时间会**静默**算错。
9. **`HAL_TIM_PeriodElapsedCallback` 只能有一份非弱定义**——以后加定时器要在 [bsp_tick.c](../bsp/tick/bsp_tick.c) 里补分支，不要另写同名函数。
10. **节拍槽位只剩 3 个**（上限 4，按键已占 1）。
11. **串口 / ADC 的参数都在 CubeMX 里**——不要在 BSP 里重配波特率或 ADC 通道参数，那会和 `.ioc` 争夺同一份事实。
12. **`bsp_pot_sample()` 失败时缓存是混合状态**——必须整批丢弃，不要用 `bsp_pot_raw()` 的值。
13. **PB8/PB9 的电气参数配了两遍**（`.ioc` + `i2c_soft_init()`），改一处要改另一处。
14. **OLED 的 `'\0'` 之外的不可显示字符会让字符串输出提前结束**，不是被跳过。
