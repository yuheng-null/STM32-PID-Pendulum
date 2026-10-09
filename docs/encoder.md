# 编码器（BSP Encoder）实现详解

本文逐层拆解本仓库电机编码器的实现：AB 正交信号怎么变成数字、四倍频是什么、
为什么**不清零计数器**反而是更正确的做法、`HAL_TIM_Encoder_Start()` 有一个
会静默少计一半的坑，以及怎么验证它。

**面向读者**：嵌入式初学者。假设你会 C（指针、位运算），
但不要求懂定时器从模式（SMS）的寄存器细节。

**配套代码**：

| 文件 | 层 | 职责 |
| --- | --- | --- |
| [PID_Pendulum.ioc](../PID_Pendulum.ioc) | CubeMX | TIM3 的声明（Encoder Mode、TI12、极性、滤波） |
| [Core/Src/main.c](../Core/Src/main.c) 生成部分 | CubeMX 生成 | `MX_TIM3_Init()` → `HAL_TIM_Encoder_Init()` |
| [Core/Src/stm32f1xx_hal_msp.c](../Core/Src/stm32f1xx_hal_msp.c) | CubeMX 生成 | TIM3 时钟使能、PA6/PA7 配成**带内部上拉**的输入 |
| [bsp/encoder/bsp_encoder.h](../bsp/encoder/bsp_encoder.h) · [.c](../bsp/encoder/bsp_encoder.c) | BSP | 增量计算、位置累加、方向反转兜底 |
| [Core/Src/main.c](../Core/Src/main.c) | 应用 | 1 ms 节拍里推进游标 |

---

## 目录

- [一、要解决什么问题](#一要解决什么问题)
- [二、编码器基础：三个必须先搞清的概念](#二编码器基础三个必须先搞清的概念)
- [三、CubeMX 配了什么](#三cubemx-配了什么)
- [四、手写的部分](#四手写的部分)
- [五、怎么测试](#五怎么测试)
- [六、设计取舍一览](#六设计取舍一览)
- [附：这次踩过的坑](#附这次踩过的坑)

---

## 一、要解决什么问题

### 1.1 需求与约束

倒立摆需要**两个反馈量**，编码器负责其中一个：

| 反馈量 | 来源 | 用途 |
| --- | --- | --- |
| 摆杆角度 | 角度传感器（ADC） | 内环**角度环** |
| **横杆位置 / 电机转速** | **编码器** | 外环**位置环**，以及判断电机实际转没转 |

需求是"读到电机转了多少"，但这句话要拆成两个不同的量：

- **位置**（position）：从开机到现在累计转了多少 → 外环用
- **速度**（speed）：单位时间转了多少 → 内环的微分项、以及安全判断用

**两者都由同一个编码器得到**，区别只在于怎么处理读数（见 4.2）。

先把**手上的资源和限制**列清楚：

| 问题 | 答案 |
| --- | --- |
| 编码器是什么类型？ | **增量式**（AB 两相正交输出），不是绝对式 |
| 接在哪些脚？ | **PA6 / PA7**，原理图上的网络名是 `E1A` / `E1B` |
| 怎么确认的？ | 套件原理图 + CubeMX 的 F103 设备 XML（`PA6 → TIM3_CH1`、`PA7 → TIM3_CH2`） |
| 谁负责计数？ | **TIM3 的编码器接口**（硬件自动加/减计数，CPU 不参与） |
| 分辨率？ | 手册：输出轴转一圈，AB 相共 **408 个边沿** |
| 需要中断吗？ | **不需要**。计数器随时可读，没有"事件"要响应 |

### 1.2 该放哪一层

本仓库的分层是 `应用层 → bsp → driver/device → driver/bus → HAL`。

编码器和 [电位器](pot.md) 是同一类东西：**"这块板子上，什么接在什么脚"**——这正是 BSP 的职责，
而且它直接调 `HAL_TIM_*`，和 `bsp_key` 直接调 `HAL_GPIO_ReadPin` 是同一种做法。

→ 放 `bsp/encoder/`。

---

## 二、编码器基础：三个必须先搞清的概念

### 2.1 增量式编码器怎么工作

25GA370 电机尾部有一个霍尔编码器：**一段多极磁环 + 两个霍尔传感器**。
电机轴转动时，两个霍尔传感器各自输出一路方波，**两路相差 90°**（正交）。

```
         ┌───┐     ┌───┐     ┌───┐
  A  ────┘   └─────┘   └─────┘   └──
             ┌───┐     ┌───┐     ┌───
  B  ────────┘   └─────┘   └─────┘
         ↑90°↑

  正转时 A 领先 B；反转时 B 领先 A —— 相位差的正负就是旋转方向
```

关键点：**只有 A 一路的话，只能知道"转了多少"，不知道"往哪个方向转"**。
加上 B 相之后，通过比较两路的相位先后就能判断方向。

### 2.2 四倍频：408 是怎么来的

一个完整的正交周期里，A、B 各有一次上升沿和一次下降沿，一共有 **4 个边沿**：

```
  A    ‾‾‾‾|____|‾‾‾‾|____
           ↑    ↑    ↑    ↑
  B    ‾‾‾‾‾‾‾‾|____|‾‾‾‾|__
       边长1  边长2  边长3  边长4

  每数一个边沿（而不是每个完整周期），分辨率就提高 4 倍
```

这就是**四倍频**（TI12 模式）。手册给的 **408 边沿/输出轴圈**就是四倍频之后的数，
所以每个边沿对应 **360° / 408 ≈ 0.88°**（输出轴角度）。

STM32 的编码器接口**硬件实现**了这套逻辑：
它把两路信号接到定时器的 CH1/CH2，根据相位关系自动在**加计数**和**减计数**之间切换。
CPU 需要做的只是**读一下 CNT**。

| 模式 | 计数的边沿 | 分辨率 | 本工程用？ |
| --- | --- | --- | --- |
| TI1 | 只数 A 相的边沿 | ×2 | ✗ |
| TI2 | 只数 B 相的边沿 | ×2 | ✗ |
| **TI12** | **A、B 两相的所有边沿** | **×4** | ✅ |

### 2.3 一个必须先想清楚的问题：CNT 会不会溢出

TIM3 的 CNT 是 **16 位**（F103 上 TIM2/TIM3/TIM4 都是 16 位），范围 0~65535，
配成编码器模式后**在 0 和 65535 之间来回环绕**：

```
  65535 ─┐        ┌───────────
         └────────┘
     0   ──────────┘
         正转 →→→   65535 后回到 0
         反转 ←←←   0 之后回到 65535
```

于是就有了本模块**最核心的一个问题**：怎么从 CNT 得到"这一段转了多少"？

最朴素的想法是"读 CNT，然后清零，返回值就是增量"。**但它有两个缺陷**（见 4.2），
本模块用的是更好的办法。

---

## 三、CubeMX 配了什么

### 3.1 命令行配方（这里有个容易写错的键名）

编码器模式在那个 skill 里属于**"还没验证过的机制"**，所以走增量推进：
先只激活 + 配引脚，roundtrip 过了再加其余参数。

**第一步配方**（`mx.scratch/recipe_tim3_a.txt`）：

```text
set mode TIM3 "Encoder Mode"
set pin PA6 TIM3_CH1
set pin PA7 TIM3_CH2
set ip parameters TIM3 EncoderMode TIM_ENCODERMODE_TI12
```

**第二步配方**（`mx.scratch/recipe_tim3_b.txt`）：

```text
set ip parameters TIM3 IC1Polarity TIM_ICPOLARITY_RISING
set ip parameters TIM3 IC2Polarity TIM_ICPOLARITY_FALLING
set ip parameters TIM3 IC1Filter 15
set ip parameters TIM3 IC2Filter 15
set ip parameters TIM3 Period 65535
```

**第三步配方**：给编码器输入加上拉（`mx.scratch/recipe_tim3_c.txt` 的前两行）：

```text
set gpio parameters PA6 GPIO_PuPd GPIO_PULLUP
set gpio parameters PA7 GPIO_PuPd GPIO_PULLUP
```

四处值得记的细节：

**① 编码器模式的参数键名，和输入捕获模式不一样。**

这是最容易写错的地方。同一个定时器上，两种模式对"捕获通道"的命名不同：

| 模式 | 极性键名 | 滤波键名 |
| --- | --- | --- |
| 输入捕获 | `ICPolarity_1` | `ICFilter_1` |
| **编码器** | **`IC1Polarity`** | **`IC1Filter`** |

从 CubeMX 的 F1 定义文件（`db/mcu/IP/TIM1_8F1-gptimer2_v1_x_Cube_Modes.xml`）
里 `Encoder_Interface` 这个 RefMode 的参数列表能直接看到：

```xml
<RefMode Name="Encoder_Interface" HalMode="TIM_Encoder" Group="Encoder">
    <Parameter Name="EncoderMode" />
    <Parameter Name="IC1Polarity" />      ← 不带 _1 后缀
    <Parameter Name="IC1Filter" />
    <Parameter Name="IC2Polarity" />
    <Parameter Name="IC2Filter" />
    ...
```

**② 键名不同，合法取值也不同。**

同样是 F1 的定义文件，两个键的取值范围是**两套不同的枚举**：

```xml
<!-- 编码器模式用的 -->
<Item Name="IC1Polarity" ... Value="TIM_ICPOLARITY_RISING"/>
<Item Name="IC1Polarity" ... Value="TIM_ICPOLARITY_FALLING"/>

<!-- 输入捕获模式用的 -->
<Item Name="ICPolarity_1" ... Value="TIM_INPUTCHANNELPOLARITY_RISING"/>
```

写错的表现是"命令回 OK、roundtrip 也过、生成的代码里字段缺失或值不对"——
和 [serial.md](serial.md) 里那次 F1 参数取值踩的坑是同一类。

**③ 为什么 IC1 上升沿、IC2 下降沿。**

**这一对极性决定计数方向**（不是决定"边沿类型"）。本工程取值与参考实现
`TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling)`
严格对齐，这样两边的方向约定一致。换电机或 A/B 相接反时要重新确认（见 4.4）。

**④ 输入滤波取最大档 15。**

电机是**有刷**的，碳刷换向会产生噪声，编码器线又长，不滤波会让 CNT 出现
偶发多计/少计——在 PID 里的表现是"位置环莫名跳动"，极难查。

查 F1 定义文件可以确认各档的含义，15 是最强的一档：

```xml
<Item Name="IC1Filter" ... Value="0"/>    → LL_TIM_IC_FILTER_FDIV1        （不滤波）
<Item Name="IC1Filter" ... Value="15"/>   → LL_TIM_IC_FILTER_FDIV32_N8   （fDTS/32 采样，连续 8 次一致才认）
```

代价是**响应变慢**：滤波会延迟边沿的确认，理论上会丢极窄的脉冲。
但 408 边沿/圈的信号在 620 RPM 下也只有约 4.2 kHz，脉冲宽度远大于滤波窗口，
所以这个代价在本应用里可以忽略。

**⑤ `Period = 65535` 和 `IC1Polarity = RISING` 没落进 `.ioc`。**

这不是失败——CubeMX **不写等于默认值的键**。`Period` 的默认值就是 65535、
极性的默认值就是 RISING，所以它们被省略了，也**不会出现在 `IPParameters` 白名单里**。

**这正是"看不出问题"的地方**，所以必须去生成的代码里核对（见 5.2）。
如果不核对，你无法区分"因为是默认值所以没写"和"写错了被丢弃"。

### 3.2 生成出来的代码

**`MX_TIM3_Init()`**（[Core/Src/main.c](../Core/Src/main.c)）：

```c
htim3.Instance = TIM3;
htim3.Init.Prescaler = 0;
htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
htim3.Init.Period = 65535;
htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;      /* 四倍频 */
sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;     /* ← 默认值，被补上了 */
sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
sConfig.IC1Filter    = 15;                        /* 最强滤波 */
sConfig.IC2Polarity  = TIM_ICPOLARITY_FALLING;    /* 决定方向 */
sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
sConfig.IC2Filter    = 15;
if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) { Error_Handler(); }

sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { ... }
```

**九个字段一个不少** —— 这就是 5.2 要数的东西。少任何一个都说明配置没生效。

**`HAL_TIM_Encoder_MspInit()`**：

```c
if(htim_encoder->Instance==TIM3)
{
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /**TIM3 GPIO Configuration
  PA6     ------> TIM3_CH1
  PA7     ------> TIM3_CH2
  */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;        /* ← 第三份配方加的 */
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
```

**注意 `GPIO_PULLUP` 是手工加的**：CubeMX 默认生成 `GPIO_NOPULL`，
而参考实现用的是 `GPIO_Mode_IPU`（上拉输入）。

**为什么必须上拉**：如果编码器输出是**开漏**结构，没有上拉就永远读不到高电平——
现象是"编码器完全不计数的同时，也完全不报错"。而即使是推挽输出，
上拉也无害（只是多耗一点电流）。**这是一个代价为零、收益明确的保险。**

**另外注意它用的是 `HAL_TIM_Encoder_MspInit()` 而不是 `HAL_TIM_Base_MspInit()`**。
三个 MSP 函数按 HAL 的"模式"分工：

| HAL 调用 | 对应的 MSP |
| --- | --- |
| `HAL_TIM_Base_Init()` | `HAL_TIM_Base_MspInit()`（TIM1 走这条） |
| `HAL_TIM_PWM_Init()` | `HAL_TIM_PWM_MspInit()`（TIM2 走这条） |
| `HAL_TIM_Encoder_Init()` | `HAL_TIM_Encoder_MspInit()`（TIM3 走这条） |

### 3.3 一处 CubeMX 不会做的：CMake 源文件清单

新增 `bsp/encoder/bsp_encoder.c` 要在顶层 [CMakeLists.txt](../CMakeLists.txt)
的 `target_sources` 和 `target_include_directories` 各加一行（该文件没有用 `GLOB`，
而且 CubeMX 只生成它一次、不会覆盖）。

`cmake/stm32cubemx/CMakeLists.txt` **不需要动**：`stm32f1xx_hal_tim.c` /
`hal_tim_ex.c` 在配 TIM1 时就已经在清单里了。

### 3.4 踩过的坑

**① roundtrip 报的"白名单缺口"是误报**

```
!! 这些键没被列进 IPParameters 白名单，CubeMX 会完全无视它们:
     [TIM2] 缺: Channel-PWM\ Generation1\ CH1
```

（这次是 TIM2 的键，但同一个 bug 会命中**所有键名含空格的**参数，包括 TIM3 的。）

去 `.ioc` 里看，白名单条目其实在——只是**键名里的空格被转义成 `\ `，而白名单值里的没有**，
检查脚本拿转义过的键名去比未转义的条目，必然对不上。

**证伪的办法不是再看一遍 roundtrip，而是去看生成的代码**：
`MX_TIM2_Init()` 里 `HAL_TIM_PWM_ConfigChannel(..., TIM_CHANNEL_1)` 出现了，
说明通道配置确实生效。

**② 编码器模式确实"一圈都没端到端跑过"**

那个 skill 的 `verified-recipes.md` 明确把 `Encoder Mode` 列在**"还没验证过的"**清单里，
并给出建议顺序：**先只激活 → roundtrip → 再加参数 → 再 roundtrip**。
本文档记录的就是**第一次把它跑通**的过程，配方可以直接复用。

---

## 四、手写的部分

### 4.1 先定接口

```c
error_t bsp_encoder_init(void);
error_t bsp_encoder_update(void);   /* 采集：推进游标、累加 */
int32_t bsp_encoder_total(void);    /* 累计位置（边沿） */
int16_t bsp_encoder_delta(void);    /* 上一次的增量，可当速度 */
error_t bsp_encoder_reset(void);
error_t bsp_encoder_set_invert(bool invert);
```

**接口形状刻意和 `bsp_pot` 对齐**：

| bsp_pot | bsp_encoder | 角色 |
| --- | --- | --- |
| `bsp_pot_sample()` | `bsp_encoder_update()` | 采集（有副作用、返回 `error_t`） |
| `bsp_pot_raw()` | `bsp_encoder_total()` / `delta()` | 读缓存（无副作用、返回数值） |

这样"采集"和"取值"的分工在两个模块里是同一种心智模型，读代码时不用切换。

### 4.2 为什么不清零计数器

**先说常见写法**（参考实现就是这么写的）：

```c
int16_t Encoder_Get(void)
{
    int16_t Temp = TIM_GetCounter(TIM3);   /* 读 */
    TIM_SetCounter(TIM3, 0);               /* 清零 */
    return Temp;
}
```

**它有两个缺陷**：

| 缺陷 | 后果 |
| --- | --- |
| 读与清零之间到来的边沿，会被那次清零抹掉 | 丢计数。这个窗口关不掉——清零是写寄存器，不是原子的"读-改-写" |
| 只在被调用时才去看 CNT | 若调用间隔内 CNT 绕了一整圈，`Temp` 只反映绕圈后的残余值 |

第二个缺陷对本工程不现实（408 边沿/圈、620 RPM 满速也只有 4.2k 边沿/秒，
1 ms 内不可能绕完 65536 个），但**第一个缺陷是实打实的**。

**本模块的做法：不清零，自己记住上次的值，用模 2^16 相减求增量。**

```c
const uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

/* 先在 16 位无符号里环绕相减，再把结果重新解释成有符号 */
int32_t d = (int32_t)(int16_t)(uint16_t)(now - s_last);
s_last = now;

s_total += d;
s_delta  = (int16_t)d;
```

**为什么这样能正确处理回绕**：假设 CNT 从 65535 走到 0（正转一个边沿）：

```
now   = 0x0000
last  = 0xFFFF
now - last（按 uint16_t）  = 0x0001        ← 环绕相减，自动对 2^16 取模
(int16_t)0x0001            = +1            ← 正确地表示"正转了 1 个边沿"

若换成先转 int16_t 再在 int 里相减：
(int16_t)0x0000 - (int16_t)0xFFFF = 0 - (-1) = +1   ← 这个例子碰巧对
但 last=0x8000(= -32768)、now=0x7FFF(= +32767) 时：
32767 - (-32768) = +65535                          ← 完全错误
```

**必须"先按无符号环绕相减、再转有符号"**，顺序不能颠倒。

于是本模块有三个好处：

| 好处 | 说明 |
| --- | --- |
| **不丢边沿** | 全程不写 CNT，读操作是纯读取，没有"读-改-写"的窗口 |
| **采集时机任意** | 因为累计的是增量之和，采集频率再低，位置也精确（只要两次之间 \|增量\| < 32768） |
| **不需要关中断** | 没有非原子操作。`__HAL_TIM_GET_COUNTER` 就是读一个寄存器，单条指令 |

**这也意味着它不需要注册节拍任务** —— 但为了显示刷新有稳定的节奏，
本工程还是在 1 ms 节拍里调了一次 `bsp_encoder_update()`（纯寄存器读，几十个周期，
符合"快进快出"）。节拍槽位上限 4 个，按键 + 编码器占了 2 个，**还剩 2 个留给 PID 控制环**。

### 4.3 `HAL_TIM_Encoder_Start()` 必须传 `TIM_CHANNEL_ALL`

**这是本模块最容易静默出错的地方**，值得单列一节。

看 HAL 源码（`Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c`）：

```c
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
  ...
  /* Enable the encoder interface channels */
  switch (Channel)
  {
    case TIM_CHANNEL_1:
      TIM_CCxChannelCmd(htim->Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);   /* 只使能 CC1E */
      break;

    case TIM_CHANNEL_2:
      TIM_CCxChannelCmd(htim->Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);   /* 只使能 CC2E */
      break;

    default :
      TIM_CCxChannelCmd(htim->Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);   /* 两个都使能 */
      TIM_CCxChannelCmd(htim->Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
      break;
  }
  __HAL_TIM_ENABLE(htim);
  return HAL_OK;
}
```

而通道标识符的取值是（`stm32f1xx_hal_tim.h`）：

```c
#define TIM_CHANNEL_1          0x00000000U
#define TIM_CHANNEL_2          0x00000004U
#define TIM_CHANNEL_ALL        0x0000003CU
```

**结论**：

- 传 `TIM_CHANNEL_1` → 只置 `CC1E` → **只有 A 相在计数**
- 传 `TIM_CHANNEL_2` → 只置 `CC2E` → 只有 B 相在计数
- 传 `TIM_CHANNEL_ALL` → 落到 `default` 分支 → `CC1E | CC2E` 都置 → **四倍频成立**

**传错的后果**：少一半分辨率（只数一相的边沿），或者方向判断失效。
而它**编译通过、运行不报错、`HAL_OK` 照常返回**。
如果你的位置环增益是按 408 边沿/圈算的，实际拿到的却是 204，
整个标定就错了，而且现象只是"增益感觉和理论算的不一样"。

所以本模块里是：

```c
if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
    return ERR_NOT_READY;
}
```

### 4.4 方向由极性决定，但留一个软件兜底

编码器的**计数方向**由两件事共同决定：

1. CubeMX 里 `IC1Polarity` / `IC2Polarity` 的配置
2. 实际接线时 A/B 两相有没有接反

参考实现用 `(Rising, Falling)`，本工程与之对齐。但**万一 A/B 相接反了**
（换电机、或者线序不同），改 `.ioc` 重新生成比较麻烦，所以留一个软件开关：

```c
error_t bsp_encoder_set_invert(bool invert);
```

它只是在累加前把增量取反：

```c
if (s_invert) { d = -d; }
```

> ⚠️ **但真正的方向基准不是它，也不是 `.ioc`，而是"电机正转时计数必须增大"（见 5.4）。**
> 极性、接线、invert 三者只要求**互相自洽**，最终以实测为准。

### 4.5 `reset()` 为什么要重新对齐游标

```c
error_t bsp_encoder_reset(void)
{
    if (!s_ready) { return ERR_NOT_INITIALIZED; }

    s_last  = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);   /* ← 这一句不能省 */
    s_total = 0;
    s_delta = 0;
    return ERR_OK;
}
```

**只把 `s_total` 置 0 是不够的**。因为 `s_last` 还停在旧值上，
下一次 `update()` 会把"上次采样到这次清零之间"累积的边沿也算进去——
位置就会凭空多出一截（或者少一截）。

**这也是"不清零 CNT"方案的代价**：软件游标和硬件计数器要显式对齐。
相比之下"读一次清一次"的方案没有这个问题（因为它每次都对齐了），
但它有丢边沿的问题。**两害相权，这个代价是更小的、而且是可以显式处理的。**

同样的事情在 `init()` 里也要做一次：

```c
s_last = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
```

不做的现象是：第一次 `update()` 算出的增量是"0 减去开机时的 CNT"，
是个无意义的巨大值，会让位置一开始就偏一大截。

---

## 五、怎么测试

### 5.1 为什么「编译通过 + 烧录成功」不算验证

对编码器这个模块尤其要注意——它的失败模式**特别安静**：

| 失败 | 现象 |
| --- | --- |
| 通道参数传错 | 编译通过、`HAL_OK`、**计数少一半** |
| 上拉没配 | 完全不计数，**也不报错** |
| 极性配错 | 计数正常、**方向相反** |
| 字段被丢弃 | 初始化函数**少几行**，其余照常 |

**这四种没有一种会报错**，只能靠下面四层证据分别排除。

### 5.2 第一层：去生成的代码里数「字段够不够」

```bash
sed -n '/^static void MX_TIM3_Init/,/^}/p' Core/Src/main.c
grep -n "TIM3 GPIO Configuration" -A 6 Core/Src/stm32f1xx_hal_msp.c
```

**要逐项看到（九个字段一个不能少）**：

```c
sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
sConfig.IC1Filter    = 15;
sConfig.IC2Polarity  = TIM_ICPOLARITY_FALLING;
sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
sConfig.IC2Filter    = 15;
```

**"数一数字段够不够"是这一步的关键动作**。当 CubeMX 因为取值非法而把外设标成
not-ready 时，生成的初始化函数会**少字段**——而单看"函数存在"是看不出来的。
（这就是 [serial.md](serial.md) 里那次"四层检查全绿、值却是错的"的教训。）

同时确认 MSP 里是 `HAL_TIM_Encoder_MspInit`、PA6/PA7 配成 `GPIO_MODE_INPUT` + **`GPIO_PULLUP`**。

### 5.3 第二层：读运行时变量

```bash
arm-none-eabi-nm build/Debug/PID_Pendulum.elf | grep -E "s_total|s_delta|s_last"
# 2000085c b s_total       ← 累计位置
# 20000860 b s_delta       ← 上次增量
```

`mx.scratch/pend_watch.py` 可以连续监视（`mode=hotplug` 不复位连接，读到的是实时值）。

### 5.4 第三层：方向一致性（决定性证据）

**判据：给电机一个正占空比，`s_total` 必须增大。**

这一条把"电机方向"和"编码器方向"锁在一起，是双环 PID 能收敛的前提
（详细说明见 [motor.md 5.4](motor.md#54-第三层方向一致性最要紧的一条)）。

实测：

| 操作 | `s_total` 变化 | 结论 |
| --- | --- | --- |
| `duty = +20` | 685 → 3105（**递增**） | 正向一致 ✅ |
| `duty = −20` | 28235 → 26252（**递减**） | 反向一致 ✅ |

### 5.5 第四层：分辨率与静止稳定性

**① 分辨率（408 边沿/圈）—— 用转速反推验证**

本机**没有做**"手转一圈精确数 408"这个直接测试，而是用**与电机联动的间接方法**验证：

```
实测：0.5 秒内 s_total 变化 393 → 786 边沿/秒
理论：620 RPM × 20% = 124 RPM = 2.07 圈/秒 → 2.07 × 408 = 843 边沿/秒
实测 / 理论 = 786 / 843 ≈ 93%
```

93% 来自有刷电机的摩擦和内阻（"空载 620 RPM"本身是理想值），量级合理。

**如果 408 这个常数错了**（比如实际是 204），那么这个比值会变成约 47% —— 
一眼就能看出不对。

> ⚠️ 诚实地说：这是**间接验证**。要直接验证应该"手转输出轴整一圈，看
> `s_total` 是否恰好变化 408"。本次没做，因为手转很难恰好一圈、也难保证不。
> 有需要时可以用转盘刻度辅助。

**② 静止无噪声 —— 决定性证据**

不碰任何东西，连续读 `s_delta`：

```
    时刻  angle  delta   total  duty   cmd run
   0.6   1168      0  -11397     0   -10 off
   1.2   1172      0  -11397     0   -10 off
   1.7   1168      0  -11397     0   -10 off
   ...
   5.9   1192      0  -11397     0   -10 off
```

`s_delta` 恒为 0、`s_total` 纹丝不动。**这说明输入滤波（15 档）和上拉都生效了** ——
否则有刷电机的碳刷噪声和悬空输入会让 CNT 缓慢地乱爬，
而这个现象在 PID 里会表现为"位置环莫名跳动"，极难定位。

### 5.6 没验证到什么

| 项目 | 状态 |
| --- | --- |
| "手转一圈 = 408"直接计数 | ⬜ 未做（用转速反推间接验证，见 5.5） |
| 最高转速下的计数准确性 | ⬜ 未做（只在 20% 占空比下测过） |
| 编码器线较长时的抗噪能力 | ⬜ 未做（只在台面上的原装线上测过） |

---

## 六、设计取舍一览

| 决定 | 换来什么 | 代价 |
| --- | --- | --- |
| **只读不清零 CNT** | 不丢边沿、采集时机任意、不需要关中断 | `reset()` 要显式对齐游标；依赖"两次之间 \|增量\| < 32768" |
| 模 2^16 相减（先无符号再有符号） | 回绕自动处理、不会算错 | 写法反直觉，必须配注释（本模块写了） |
| 输入滤波取最大档 15 | 碳刷噪声不会污染计数 | 边沿确认有延迟（对 4.2 kHz 的信号可忽略） |
| 输入加上拉 | 开漏输出的编码器也能读 | 推挽输出时多耗一点电流（可忽略） |
| 接口拆成 update / total / delta | 与 bsp_pot 形状一致、采集与取值分工清晰 | 多两个函数 |
| 不注册独立的节拍任务 | 一次读几十个周期，顺手在已有任务里做完 | 位置累加的快慢不独立（但方案本身不依赖采样率） |
| 留 `set_invert()` | A/B 接反时不用改 `.ioc` 重新生成 | 多一个可能被误用的开关（头文件里写明正常不用它） |

整个模块约 150 行，其中真正的"逻辑"只有 `update()` 里的四行，
其余都是**为正确性付出的成本**——游标对齐、模运算、`s_ready` 守卫、方向兜底。

---

## 附：这次踩过的坑

按"踩到的顺序"记录，都是实测：

| 坑 | 现象 | 根因 |
| --- | --- | --- |
| **`IC1Polarity` 写成 `ICPolarity_1`** | 命令回 OK，但参数不生效 | 编码器模式与输入捕获模式的键名不同（见 3.1 ①）。**键名和取值都要按模式查定义文件** |
| **`HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_1)`** | 计数正常但**只有一半分辨率** | 传单通道只置一个 `CCxE`，四倍频需要两个都使能 → 必须传 `TIM_CHANNEL_ALL` |
| **CubeMX 默认给 `GPIO_NOPULL`** | 开漏输出的编码器完全不计数 | 参考实现用 `GPIO_Mode_IPU`。上拉要手工加，且**加了不报错、不加也不报错** |
| **`Period`/`IC1Polarity` 没落进 `.ioc`** | 以为参数被丢弃了 | 等于默认值的键 CubeMX 不写。**必须去生成代码里确认值是对的** |
| **把 CNT 当有符号数相减** | 跨半区时增量算成 ±65535 | 必须"先按 uint16 环绕相减、再转 int16" |
| **`reset()` 只清 `s_total`** | 清零后位置凭空多出一截 | `s_last` 没跟着对齐，下一段增量多算了 |

**第一条最值得记**：CubeMX 的**参数名本身**也带模式语义。
同一个定时器、同一个物理概念（捕获通道极性），在两种模式下的键名和取值**都是两套**。
写错时所有命令照样返回成功，只有去定义文件里对照、或者去生成代码里数
字段才能发现。
