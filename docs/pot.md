# 电位器（BSP Pot）实现详解

本文逐层拆解本仓库 4 路电位器旋钮的实现：ADC 的基础概念、CubeMX 配了什么、
采样循环为什么这么写、以及怎么在没有摄像头的情况下验证它。

**面向读者**：嵌入式初学者。假设你会 C（结构体、指针、位运算），
但不要求懂 ADC 的寄存器细节。

**配套代码**：

| 文件 | 层 | 职责 |
| --- | --- | --- |
| [PID_Pendulum.ioc](../PID_Pendulum.ioc) | CubeMX | ADC2 的声明（4 通道、扫描关闭、序列长度 1） |
| [Core/Src/main.c](../Core/Src/main.c) 生成部分 | CubeMX 生成 | `MX_ADC2_Init()`、ADC 时钟分频 |
| [Core/Src/stm32f1xx_hal_msp.c](../Core/Src/stm32f1xx_hal_msp.c) | CubeMX 生成 | ADC2 时钟使能、PA2~PA5 配成模拟输入 |
| [bsp/pot/bsp_pot.h](../bsp/pot/bsp_pot.h) · [.c](../bsp/pot/bsp_pot.c) | BSP | 自校准、逐路采样、原始值/电压换算 |
| [Core/Src/main.c](../Core/Src/main.c) | 应用 | 采样调度、死区、屏幕绘制 |

---

## 目录

- [一、要解决什么问题](#一要解决什么问题)
- [二、ADC 基础：三个必须先搞清的概念](#二adc-基础三个必须先搞清的概念)
- [三、CubeMX 配了什么](#三cubemx-配了什么)
- [四、手写的部分](#四手写的部分)
- [五、怎么测试](#五怎么测试)
- [六、设计取舍一览](#六设计取舍一览)
- [附：这次踩过的坑](#附这次踩过的坑)

---

## 一、要解决什么问题

### 1.1 需求与约束

4 只卧式电位器，中间抽头分别接 PA2、PA3、PA4、PA5，要读出数值并显示。

拿到需求先把**手上的资源和限制**列清楚，这一步决定后面所有选择：

| 问题 | 答案 |
| --- | --- |
| 这四个脚是哪些 ADC 通道？ | ADC_IN2 / IN3 / IN4 / IN5 |
| 这颗芯片有几个 ADC？ | ADC1 和 ADC2（F103C8T6 中容量，LQFP48） |
| 选哪个 ADC？ | **ADC2** —— 把 ADC1 留给后面的模块 |
| 换 ADC 要改接线吗？ | **不用**。ADC1 和 ADC2 在同一组引脚上都有同名通道 |
| 引脚有没有被别的外设占？ | PA2/PA3 同时是 TIM2_CH3/CH4，但 TIM2 没用到 → 不冲突 |

### 1.2 该放哪一层

本仓库的分层是 `应用层 → bsp → driver/device → driver/bus → HAL`。

电位器不属于任何一层现成的类别：

- 它**不是器件驱动**（`driver/device/`）——电位器没有寄存器手册、没有协议，
  它只是个可变电阻
- 它是"**这块板子上，4 只旋钮接在哪几个脚**"——这正是 BSP 的职责

而且它直接调 `HAL_ADC_*`，和 `bsp_key` 直接调 `HAL_GPIO_ReadPin` 是同一种做法。

→ 放 `bsp/pot/`。

---

## 二、ADC 基础：三个必须先搞清的概念

### 2.1 采样时间：为什么不能随便选

ADC 转换分两段：

```
┌──────────────┬────────────────────────┐
│  采样保持     │  逐次逼近              │
│  采样电容充电  │  12 位逐位比较          │
│  (可配置)     │  (固定 12.5 个 ADC 周期) │
└──────────────┴────────────────────────┘
        ↑
   这一段由 SamplingTime 决定
```

**采样阶段**要做的就是：把内部采样电容充到和输入电压一样。而电容充电需要时间，
时间常数 = `R_源 × C_采样`。

- 如果源阻抗大、采样时间给太短 → 电容**还没充够就进入比较**，读数偏低
- 电位器抽头的源阻抗最高**接近它的标称阻值**（10 kΩ 的电位器，抽头在中位时
  上下各 5 kΩ 并联，约 2.5 kΩ；拧到端点时接近 0）
- 所以取一个偏保守的值：**55.5 个 ADC 周期**

在本模块的 12 MHz ADC 时钟下，55.5 周期 ≈ 4.6 µs；加上固定的 12.5 周期比较段，
**一路转换约 5.67 µs**，4 路共约 23 µs。

> 采样时间不是"越大越好"——它直接决定转换速度。这里 4 路只要 23 µs，
> 而人手拧旋钮的采样周期是 20 ms，余量有 800 倍，所以往保守了取毫无代价。

### 2.2 ADC 时钟：有一个独立的分频器

ADC 的时钟**不是**直接取 PCLK2，中间有一级独立分频（`RCC_CFGR` 的 `ADCPRE` 位）：

```
PCLK2 = 72 MHz ──► ADCPRE 分频 ──► ADCCLK ──► ADC 转换
                    (/2 /4 /6 /8)     ↑
                            必须 ≤ 14 MHz（F103 数据手册上限）
```

72 / 6 = **12 MHz** ✓（/2 就是 36 MHz，超上限 2.5 倍）

超过 14 MHz 的后果不是"不工作"，而是**精度下降、读数不稳**——这种故障最难查，
因为它不报错。

### 2.3 EOC 的语义：本模块最核心的一条

ADC 转换完成后会置一个标志位 **EOC**（End Of Conversion），用来告诉 CPU"我好了"。
多通道时，ADC 按序列（rank 1、rank 2…）依次转换。

**关键问题**：转完一路就置一次 EOC，还是整个序列转完才置一次？

**在 STM32F1 上，答案是后者。** 依据有两处，都是第一手材料：

> **RM0008 对 `SR.EOC` 的定义**：
> "This bit is set by hardware at the end of a **group** channel conversion."
>
> **HAL 源码 `stm32f1xx_hal_adc.c` 里的注释**：
> "If sequence conversion (scan mode enabled and NbrOfConversion >=2),
> flag EOC is set only at the end of the sequence. …
> **As flag EOC is not set after each conversion**, no timeout status can be set."

这带来一个直接后果：**扫描模式下，`DR` 里永远只有最后一路的结果**，
而 EOC 只能告诉你"整组转完了"。想逐路拿到数据，硬件上**只有 DMA 一条路**——
RM0008 明确写：

> "When using scan mode, **DMA bit must be set** and the DMA controller is used to
> transfer the converted data of regular group channels to SRAM after each update
> of the ADC_DR register."

**所以本模块的方案是：不用扫描模式，一次只转一路。**

```
序列长度设为 1  →  "一组"就只有一次转换
                →  EOC 恢复成「这次转换结束就置位」的正常语义
                →  逐路读 DR 才成立
```

每次读之前，用 `HAL_ADC_ConfigChannel()` 把 **rank 1** 换成目标通道即可。

> **为什么不用 DMA？** 因为 STM32F103 的 **ADC2 没有 DMA 请求**。
> RM0008 的 DMA1 请求表里，`DMA1 Channel 1` 只挂了 **ADC1**；
> 而 C8T6 是中容量，**没有 DMA2**、也没有 ADC3。
> 这也是本工程把 ADC1 空出来的原因——**留给将来真的需要 DMA 的多通道采集**。

---

## 三、CubeMX 配了什么

### 3.1 命令行配方

```text
# 激活 ADC2 并挂上 4 个通道（CubeMX 会自动把 PA2~PA5 分配给 ADC2）
set mode ADC2 IN2
set mode ADC2 IN3
set mode ADC2 IN4
set mode ADC2 IN5

# 槽位 0 = IN2，采样时间统一 55.5 周期
set ip parameters ADC2 SamplingTime-0#ChannelRegularConversion ADC_SAMPLETIME_55CYCLES_5

# 追加槽位 1~3，排定序列顺序
set ip parameters ADC2 Channel-1#ChannelRegularConversion ADC_CHANNEL_3
set ip parameters ADC2 Rank-1#ChannelRegularConversion 2
set ip parameters ADC2 SamplingTime-1#ChannelRegularConversion ADC_SAMPLETIME_55CYCLES_5
set ip parameters ADC2 Channel-2#ChannelRegularConversion ADC_CHANNEL_4
set ip parameters ADC2 Rank-2#ChannelRegularConversion 3
set ip parameters ADC2 SamplingTime-2#ChannelRegularConversion ADC_SAMPLETIME_55CYCLES_5
set ip parameters ADC2 Channel-3#ChannelRegularConversion ADC_CHANNEL_5
set ip parameters ADC2 Rank-3#ChannelRegularConversion 4
set ip parameters ADC2 SamplingTime-3#ChannelRegularConversion ADC_SAMPLETIME_55CYCLES_5

# 序列长度 1（见 2.3 节）
set ip parameters ADC2 NbrOfConversion 1

# ADC 时钟 /6 → 12 MHz
clock set ADCPresc 6
```

**几个容易忽略的点**：

| 点 | 说明 |
| --- | --- |
| `set mode ADC2 IN2` 会**自动分配引脚** | 不需要再写 `set pin PA2 ADCx_IN2` |
| **`NbrOfConversion` 必须显式写** | CubeMX 从槽位数推导，但生成出来的是 **1**——扫描模式下等于只转第一路 |
| `clock set` 和 `set ip parameters` **不是一回事** | 时钟树必须走 `clock set`，见 3.4 |
| 配方里**没有** `ScanConvMode` | 序列长度是 1 时 CubeMX 自己会把扫描模式关掉，不需要也不应该手写（见 3.4） |

### 3.2 生成出来的代码

配完必须去生成的 `.c` 里**逐项找到对应代码**——"参数被接受"不等于"代码被生成"。

**① `Core/Src/main.c` 的 `MX_ADC2_Init()`**

```c
hadc2.Init.ScanConvMode       = ADC_SCAN_DISABLE;   /* ← 序列长度 1 时自动关 */
hadc2.Init.ContinuousConvMode = DISABLE;            /* ← 单次转换 */
hadc2.Init.DiscontinuousConvMode = DISABLE;
hadc2.Init.ExternalTrigConv   = ADC_SOFTWARE_START; /* ← 软件触发 */
hadc2.Init.DataAlign          = ADC_DATAALIGN_RIGHT;
hadc2.Init.NbrOfConversion    = 1;                  /* ← 关键 */

HAL_ADC_ConfigChannel(&hadc2, {ADC_CHANNEL_2, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_55CYCLES_5});
HAL_ADC_ConfigChannel(&hadc2, {ADC_CHANNEL_3, ADC_REGULAR_RANK_2, ...});
HAL_ADC_ConfigChannel(&hadc2, {ADC_CHANNEL_4, ADC_REGULAR_RANK_3, ...});
HAL_ADC_ConfigChannel(&hadc2, {ADC_CHANNEL_5, ADC_REGULAR_RANK_4, ...});
```

> 后三次 `ConfigChannel`（rank 2~4）虽然写了 SQR 寄存器，但**序列长度是 1，它们不会被用到**。
> 它们不会报错——`HAL_ADC_ConfigChannel` **不校验 rank 是否超出 `NbrOfConversion`**，
> 它只是往 SQR 寄存器写。所以 `MX_ADC2_Init` 里的 `Error_Handler()` 不会触发。

**② `Core/Src/stm32f1xx_hal_msp.c` 的 `HAL_ADC_MspInit()`**

```c
__HAL_RCC_ADC2_CLK_ENABLE();                        /* ADC2 外设时钟 */
GPIO_InitStruct.Pin  = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;            /* ← PA2~PA5 配成模拟输入 */
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
```

**③ `Core/Src/main.c` 的 `SystemClock_Config()`**

```c
PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;   /* ← 时钟 /6 */
```

**④ `Core/Inc/stm32f1xx_hal_conf.h`**

```c
#define HAL_ADC_MODULE_ENABLED
```

### 3.3 一处 CubeMX 不会做的：CMake 源文件清单

```c
/* cmake/stm32cubemx/CMakeLists.txt —— **必须手工补** */
${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c
${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c
```

CubeMX 会把这两个 `.c` **拷进** `Drivers/`、也会在 `.mxproject` 里记上，
**但不会更新 CMake 的源文件清单**。不补的后果是链接期报：

```
undefined reference to `HAL_ADC_Init'
```

> 这个文件（和顶层 `CMakeLists.txt` 一样）**只在首次生成时产出，之后不会被重写**，
> 所以手工加的行是安全的。已验证过多次重新生成都不会冲掉它。

### 3.4 踩过的两个坑

**坑一：`.ioc` 里有个假键**

`.ioc` 里存在 `RCC.ADCCLKDivider` 这个键，看起来正是设 ADC 分频的。
但**CubeMX 根本不认它**——导入时会报：

```
IP (RCC) : Invalid parameter (ADCCLKDivider)
```

不管把它设成什么值，**生成的代码永远是 `/2`**（36 MHz，超上限），
因为 CubeMX 信的是自己内部的时钟树模型。

正确做法是走**时钟树接口**：

```text
clock set ADCPresc 6      # 值写纯数字，带 RCC_ 前缀会回 KO
```

改完 `.ioc` 里出现的是**另一个键** `RCC.ADCPresc`——那才是真正生效的。

**坑二：写了 CubeMX 不认的参数**

`ScanConvMode` / `EOCSelection` / `DMAContinuousRequests` 这三个参数，
CubeMX 的 F1 ADC 模型里**不存在**（`EOCSelection` 在 F1 的 HAL 里甚至不是
`ADC_InitTypeDef` 的成员）。把它们写进 `.ioc` 会让 CubeMX 把这个外设标成
"参数没配好或值非法"，生成代码时弹警告框。**F1 上不需要手动设它们**。

---

## 四、手写的部分

### 4.1 先定接口

```c
/* bsp_pot.h */
typedef enum {
    POT_ID_RP1 = 0, POT_ID_RP2, POT_ID_RP3, POT_ID_RP4,
    POT_ID_COUNT,               /* 电位器总数，也用作遍历上界 */
} pot_id_t;

error_t  bsp_pot_init(void);
error_t  bsp_pot_sample(void);              /* 采一次，写进缓存 */
uint16_t bsp_pot_raw(pot_id_t id);          /* 0~4095 */
uint16_t bsp_pot_millivolt(pot_id_t id);    /* 0~3300 mV */
```

**关键设计：把"采样"和"读数"拆成两个函数。**

4 路共用 **一个** 数据寄存器 `DR`，而且 ADC 是"一问一答"的：

- 如果让 `bsp_pot_raw()` 自己去触发转换，连续读 4 次就是 4 次硬件动作
- 而且 4 个值来自 **4 个不同瞬间**——旋钮正在转的时候会出现"撕裂"
  （4 个值拼不成同一时刻的快照）

拆开之后：

- `bsp_pot_sample()` 一次性把 4 路都采了，得到一个**时间上彼此靠近**的快照
- `bsp_pot_raw()` 变成纯内存读取，随便调、无副作用

**为什么同时提供 `raw` 和 `millivolt`**：两者都是**客观事实**（一个是 ADC 码值、
一个是折算电压），不是"策略"。像"百分比"这种带主观口径的换算留给应用层——
和 `bsp_key` 只报"边沿"、计数的口径交给应用是同一个原则。

### 4.2 通道对应表

```c
static const uint32_t s_pot_channel[POT_ID_COUNT] = {
    ADC_CHANNEL_2,      /* RP1 ── PA2 */
    ADC_CHANNEL_3,      /* RP2 ── PA3 */
    ADC_CHANNEL_4,      /* RP3 ── PA4 */
    ADC_CHANNEL_5,      /* RP4 ── PA5 */
};
```

**为什么这张表要手写**：CubeMX 对 ADC 通道**不生成引脚宏**（不像 GPIO 会生成
`KEY1_Pin` 那样）。而"这块板子上什么接在什么脚上"**正是 BSP 层的职责**，
放这里名副其实。

> ⚠️ 改接线时要同步改这里，**并且**同步改 `.ioc` 里 PA2~PA5 的信号。

### 4.3 采样循环

```c
error_t bsp_pot_sample(void)
{
    if (!s_pot_ready) return ERR_NOT_INITIALIZED;

    for (uint8_t i = 0U; i < (uint8_t)POT_ID_COUNT; i++) {
        /* ① 把 rank 1 换成目标通道（序列长度是 1，换 rank1 = 换整个序列） */
        ADC_ChannelConfTypeDef cfg = {0};
        cfg.Channel      = s_pot_channel[i];
        cfg.Rank         = ADC_REGULAR_RANK_1;
        cfg.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
        if (HAL_ADC_ConfigChannel(&hadc2, &cfg) != HAL_OK) return ERR_NOT_READY;

        /* ② 启动这一次转换 */
        if (HAL_ADC_Start(&hadc2) != HAL_OK) return ERR_NOT_READY;

        /* ③ 等它转完（自带超时） */
        if (HAL_ADC_PollForConversion(&hadc2, BSP_POT_POLL_TIMEOUT_MS) != HAL_OK) {
            (void)HAL_ADC_Stop(&hadc2);
            return ERR_TIMEOUT;
        }

        /* ④ 读 DR：取回结果，同时自动清掉 EOC */
        s_pot_raw[i] = (uint16_t)HAL_ADC_GetValue(&hadc2);

        /* ⑤ 关掉 ADC，下一次循环才能改通道配置 */
        (void)HAL_ADC_Stop(&hadc2);
    }
    return ERR_OK;
}
```

**⑤ 为什么必须关掉 ADC**：`HAL_ADC_ConfigChannel` 是配置动作，要在 ADC 关闭
（`ADON=0`）时做。`HAL_ADC_Stop()` 正好保证了下一次循环的起点是干净的。

**④ 为什么"读 DR"就够了**：在 F1 上，读 `DR` 会**自动清掉 EOC**。
不读的话下一个 `PollForConversion` 会一直等到超时。

### 4.4 为什么这里可以用 `HAL_ADC_PollForConversion`

这个函数内部分两条路径：

```c
if (SCAN 清零 && SQR1.L 清零)  → 真正等 EOC 标志      ← 本模块命中这条
else                          → 只空转一段最坏情况时间，不查任何标志
```

本模块的配置（**扫描关闭 + 序列长度 1**）正好命中第一条，所以它是可靠的，
而且**自带超时**——比自己写自旋计数循环强。

反过来说：**扫描模式下它是不可用的**，这正是 2.3 节那个坑的一半。

### 4.5 初始化：为什么必须自校准

```c
if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK) {
    return ERR_NOT_READY;
}
```

F1 的 ADC 必须先自校准：它测量芯片内部电容失配并写入校准寄存器，
不做的现象是"**能读、但数值有几十个 LSB 的固定偏差，且随温度漂移**"，
**不报任何错**。

校准要求 ADC 处于关闭状态，而 `MX_ADC2_Init()` 刚跑完时正好满足，所以放在第一件事。

### 4.6 应用层：死区与调度

```c
#define POT_SAMPLE_MS           (20U)   /* 50 Hz，人手拧旋钮足够跟手 */
#define POT_DISPLAY_DEADBAND    (8U)    /* 变化超过 8 LSB 才重画 */
```

**死区解决什么**：ADC 末位总在抖（±1~2 LSB）。如果"一变就重画"，旋钮静止时
屏幕也会以 50 Hz 不停重画**整屏**（每次约 30 ms 的 I2C 传输），既浪费又让末位
数字闪个不停。取 8 LSB = 满量程的 0.2%，远小于人能分辨的转动幅度。

**采样调度是非阻塞的**：

```c
static void app_refresh_pots(void)
{
    static uint32_t last_sample_ms = 0U;

    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - last_sample_ms) < POT_SAMPLE_MS) return;   /* 时间没到就走 */
    last_sample_ms = now_ms;

    if (bsp_pot_sample() != ERR_OK) return;

    /* 死区判断：任何一路变化超过阈值就重画 */
    bool dirty = false;
    for (…) {
        const uint16_t v = bsp_pot_raw((pot_id_t)i);
        const uint16_t d = (v > s_shown_raw[i]) ? (v - s_shown_raw[i])
                                                : (s_shown_raw[i] - v);
        if (d >= POT_DISPLAY_DEADBAND) { dirty = true; break; }
    }
    if (!dirty) return;

    for (…) s_shown_raw[i] = bsp_pot_raw((pot_id_t)i);
    oled_redraw();
}
```

**为什么不用 `HAL_Delay`**：主循环里死等会让所有周期性动作互相拖累——屏幕每次刷
30 ms，若用延时定时，采样周期就变成"30 ms + 延时"。一律用"查时间、时间到了才做"。

**为什么放主循环、不挂 1 ms 节拍**：一次采样要阻塞约 23 µs，放进中断违反
"快进快出"（见 [bsp_tick.h](../bsp/tick/bsp_tick.h)）。

### 4.7 绘制

**① 数字右对齐到 4 位**，位数不足时前面补空格：

```c
for (uint8_t i = 0U; i < (uint8_t)(4U - nd); i++) { msg[k] = ' '; k++; }
```

不补的话，4095 掉到 512 时后面几位会跟着左移，**看起来像在抖**。

**② 进度条先画外框、再填内部**：

```c
ssd1306_draw_rectangle(&s_oled, OLED_POT_BAR_X0, y, OLED_POT_BAR_X1, y + 6U, SSD1306_WHITE);

const uint16_t inner  = OLED_POT_BAR_X1 - OLED_POT_BAR_X0 - 1U;   /* 72 像素 */
uint32_t       filled = ((uint32_t)raw * inner) / BSP_POT_RAW_MAX;
if (filled > inner) filled = inner;                                /* 夹住防越界 */
```

外框让空载时也能看出量程有多宽；**夹住是必须的**——`raw` 一旦超过满量程，
`OLED_POT_BAR_X0 + filled` 就会越界写出屏幕。

### 4.8 初始化顺序

```c
MX_ADC2_Init();                                       /* CubeMX 生成 */
…
if (bsp_pot_init() != ERR_OK) app_fatal_blink(4U);    /* 自校准 */
(void)bsp_pot_sample();                               /* 先采一次，首屏就有真实读数 */
for (…) s_shown_raw[i] = bsp_pot_raw(i);
oled_redraw();
```

错误闪灯码沿用既有约定：1=OLED、2=节拍、3=按键、**4=电位器**。

---

## 五、怎么测试

### 5.1 为什么"编译通过 + 烧录成功"不算验证

这是嵌入式最典型的认知陷阱。这套工具链的失败模式是：

> **所有命令都返回 0，没有一条报错，硬件毫无反应。**

本模块的开发过程就是活生生的例子：配置完全正确、编译零警告、烧录成功，
**但读出来的数据是错的**（4 个值一起跟着最后一路变）。

### 5.2 第一层：静态验证——读寄存器

用 ST-Link 调试口直接读寄存器，**不打断程序运行**（`mode=hotplug` 不复位）：

| 读什么 | 期望 | 实测 | 说明 |
| --- | --- | --- | --- |
| `RCC_CFGR` (0x40021004) | bit15:14 = `10` | `0x001D840A` | ADCCLK = 72/6 = 12 MHz ✓ |
| `ADC2 CR1` (0x40012804) | SCAN = 0 | `0x00000000` | 扫描模式关闭 ✓ |
| `ADC2 SQR1` (0x4001282C) | L = 0 | `0x00000000` | 序列长度 1 ✓ |
| `ADC2 CR2` (0x40012808) | EXTSEL=SWSTART | `0x001E0000` | 软件触发、空闲 ✓ |
| `APB2ENR` (0x40021018) | bit10 = 1 | `0x00000C3D` | ADC2 时钟已开，ADC1 已关 ✓ |

**这一层证明了**：CubeMX 生成的配置真的写进寄存器了。
**没证明**：采样逻辑对不对。

### 5.3 第二层：动态验证——读运行时变量

读 RAM 里的采样缓存：

```
s_pot_raw = 0x0B9A0B23  0x08900672
          → RP1 = 2851、RP2 = 2970、RP3 = 1650、RP4 = 2192
```

四个值**互不相同、且都不为 0**。再连续监视 8 秒（67 次采样）确认是"活的"：

| 通道 | 最小 | 最大 | 摆幅 |
| --- | --- | --- | --- |
| RP1 | 2194 | 2201 | 7 |
| RP2 | 2874 | 2881 | 7 |
| RP3 | 1258 | 1266 | 8 |
| RP4 | 1893 | 1899 | 6 |

摆幅只有 ~7 LSB（纯噪声级别），说明四路各自**稳定**且**在实时更新**。

> ⚠️ **注意：这一层还不够。** 四个值不同，也可能是"碰巧不同"。
> 要证明**独立性**，必须做下一层。

### 5.4 第三层：驱动引脚——决定性证据

思路和 [key.md](key.md) 里验证按键一样：**没有物理手段就用电的手段**——
用调试口**从外部驱动引脚**，看 ADC 跟不跟。

先把 PA3（RP2 的脚）从模拟输入改成推挽输出，然后拉高、拉低：

```bash
CLI=".../STM32_Programmer_CLI.exe"
S="-c port=SWD mode=hotplug"

# ① GPIOA CRL：把 PA3 的 4 位配置从 0x0(模拟) 改成 0x2(推挽输出)
$CLI $S -w32 0x40010800 0x44002044        # 读回确认 0x44002044
# ② ODR bit3 = 1（拉高）
$CLI $S -w32 0x4001080C 0x0000B808
# ③ ODR bit3 = 0（拉低）
$CLI $S -w32 0x4001080C 0x0000B800
# ④ 恢复模拟输入
$CLI $S -w32 0x40010800 0x44000044
```

结果：

| 操作 | RP1 | **RP2** | RP3 | RP4 |
| --- | --- | --- | --- | --- |
| 基线 | 2198 | 2878 | 1898 | 1259 |
| **PA3 拉高 3.3 V** | 2196 | **4082** | 1896 | 1264 |
| **PA3 拉低 0 V** | 2198 | **29** | 1896 | 1263 |

**只有 RP2 跟着 PA3 走（2878 → 4082 → 29），另外三路纹丝不动（±2 LSB 以内）。**

这证明了四路**完全独立**、通道与引脚**一一对应**。做到这一步才可以下结论。

> **关于 `0x0000B800` 这个 ODR 值：必须读-改-写，不能整字覆盖。**
> 因为 PA11/PA12 是上拉按键，它们的 ODR 位决定上拉方向——
> 写 0 会变成下拉，按键就废了。`0xB800` 正是保住那些位。
>
> **安全吗？** 安全。PA3 上接的是电位器抽头，强行驱动只是让电流改由芯片提供，
> 受电位器阻值限制（10 kΩ 约 0.33 mA，远低于引脚的 20 mA 上限）。
> **但如果引脚上接的是电源轨或别的有源输出，就不能这么干。**

### 5.5 第四层：把显存读出来还原成图像

没有摄像头也能确认"屏幕上到底画了什么"——显存在 RAM 里
（方法见 [oled.md](oled.md) 第七节）：

```
Pot Test
RP1 2198  ██████████████████████░░░░░░░░░░░░░░░░░░░░░░░░░░░░
RP2 2878  ████████████████████████████████████████████░░░░░
RP3 1898  ██████████████████░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
RP4 1259  ██████████████░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
```

**自洽性检验**：屏幕上画出的数字（2198/2878/1898/1259）和从 `s_pot_raw` 读到的值
**严格对上**，进度条填充比例也对（2198/4095 ≈ 54%）。这证明整条链路
——ADC → 驱动 → 显存 → I2C → 屏幕——每一环都对。

### 5.6 没验证到什么

| 已验证 | 未验证 |
| --- | --- |
| 寄存器配置、通道映射、四路独立性 | 电位器的**线性度**（廉价电位器两端常有非线性） |
| 采样时序、读数稳定性 | 长期漂移、温度影响 |
| 显示与整条 I2C 链路 | 旋钮的机械寿命、手感 |

**需要动手确认的**：拧到两端时，读数应分别接近 0 和 4095。
如果两端到不了极值，通常是电位器两端的供电没接好（而不是代码问题）。

---

## 六、设计取舍一览

| 决定 | 换来什么 | 代价 |
| --- | --- | --- |
| **不用扫描模式，一次采一路** | F1 上唯一可行的多通道读法（扫描模式必须配 DMA，而 ADC2 没有） | 4 路共约 23 µs 阻塞（相对 20 ms 采样周期可忽略） |
| 用 ADC2 而非 ADC1 | ADC1 空出来留给将来需要 DMA 的采集 | 需要手写通道对应表 |
| 采样时间 55.5 周期 | 对高阻抗源（电位器）有充分余量 | 单路转换 5.67 µs（余量 800 倍，无所谓） |
| 接口拆成 sample / raw | 快照时间一致、读操作无副作用 | 多一个函数、多一个缓存数组 |
| 显示死区 8 LSB | 屏幕不空刷、末位不闪 | 变化小于 0.2% 时不刷新（人的分辨率远大于此） |
| 采样放主循环 | 不违反中断"快进快出" | 采样周期受刷新屏影响（20 ms 量级，够用） |
| 通道对应表写在 BSP | "哪个脚接什么"集中在一处 | 改接线要同时改 `.ioc` 和这张表 |

整个模块约 190 行，其中真正的"逻辑"只有采样循环那 20 行，
其余都是**为正确性付出的成本**——自校准、超时、参数校验、越界夹住、死区。

---

## 附：这次踩过的坑

按"踩到的顺序"记录，都是实测：

| 坑 | 现象 | 根因 |
| --- | --- | --- |
| **扫描模式逐路读 DR** | 四个数一起跟着最后一路变；屏幕数字不动 | F1 的 `EOC` 是"**整组**结束"才置位，不是每路一次（RM0008 + HAL 源码注释） |
| **`NbrOfConversion` 不显式设** | 生成出来是 1，等于只转一路 | CubeMX 从槽位数推导，默认给 1 |
| **`RCC.ADCCLKDivider` 是假键** | 值改了、代码永远 `/2`（36 MHz 超限） | 时钟树不能用 `set ip parameters`，要用 `clock set ADCPresc 6`（纯数字） |
| **ADC2 没有 DMA** | 想用 DMA 绕开 EOC 问题，走不通 | RM0008 的 DMA1 请求表只挂了 ADC1；C8T6 无 DMA2 |
| **漏 `HAL_ADCEx_Calibration_Start`** | 能读但偏差几十 LSB、随温度漂 | F1 的 ADC 必须先自校准，且**不报错** |
| **CMake 源文件清单不更新** | `undefined reference to 'HAL_ADC_Init'` | CubeMX 不更新 `cmake/stm32cubemx/CMakeLists.txt`，要手工补 |
| **写了 F1 不认的 ADC 参数** | 生成时弹 Code Generation 警告 | `ScanConvMode`/`EOCSelection`/`DMAContinuousRequests` 在 F1 的 ADC 模型里不存在 |
| **把"读数为 0"当成"电压为 0"** | 一度误判成接线问题 | 那些数组元素**从没被写过**，是初值——循环卡在第 2 路 |

**最后一条尤其值得记**：`0` 这个值有两种完全不同的含义——
"引脚上真的是 0 V"和"这个变量压根没被更新过"。
区分它们靠的不是看现象，而是**回到代码确认这条路径有没有被执行过**
（当时的办法是加个 `dbg_ok` 计数器，读出来是 0）。
