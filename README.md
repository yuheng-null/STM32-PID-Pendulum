# STM32-PID-Pendulum

基于 **STM32F103C8T6** 的倒立摆学习工程，用 **HAL 库 + CMake** 实现。

学习路线参考 **江协科技** 的 PID 系列教程及其倒立摆项目。原教程使用 STM32 标准库（SPL），本工程在学习过程中改用 **HAL 库** 重新实现，因此代码属于独立编写，而非教程代码的移植。

目标是把这个仓库当成一份**可复现的学习记录**：每个阶段一个可编译、可烧录、可调试的状态，而不是一次性堆出一个能跑的成品。

---

## 当前进度

已完成的模块：

- CubeMX 工程搭建（.ioc）、时钟树配置、SWD 调试引脚
- CMake + Ninja 构建系统，可由 STM32CubeIDE for VSCode 直接构建
- 板载 LED（PC13）1 Hz 闪烁 —— 用于验证「编译 → 烧录 → 运行」链路通了
- **0.96 寸 OLED（SSD1306，软件 I2C）** —— 第一个功能模块
- **4 路按键（K1~K4，1 ms 采样 + 消抖）** —— 第二个功能模块
- **TIM1 1 ms 系统节拍** —— 按键消抖挂在其上，后续 PID 控制环也会复用
- **4 路电位器（RP1~RP4，ADC2 逐路采样）** —— 第三个功能模块，同时也是第一个模拟量输入
- **串口（USART1，双向中断 + 环形缓冲）** —— 第四个功能模块，供后续调参与上报数据

倒立摆本体的机械、传感器、执行器部分**尚未开始**，后续按教程推进。

> 📖 **接口速查见 [docs/api.md](docs/api.md)** —— 每个模块向应用层暴露了哪些函数、什么前置条件、
> 能不能在中断里调、值域是多少，含一张「调用上下文总表」和主循环最小骨架。
> **写代码时查它，理解原理时看各模块的 `docs/*.md` 详解。**

---

## 硬件平台

| 项目 | 说明 |
| --- | --- |
| MCU | STM32F103C8T6（LQFP48，Cortex-M3） |
| 主频 | 72 MHz |
| 外部晶振 | 8 MHz（HSE） |
| 调试接口 | SWD（PA13 / PA14） |
| 板载 LED | PC13（低电平点亮） |
| OLED | 0.96 寸 128×64，SSD1306，4 针 I2C，地址 `0x3C` |
| 按键 | 4 路独立按键 K1~K4，另一端接 GND，上拉输入（低电平有效） |
| 电位器 | 4 只卧式旋钮 RP1~RP4，中间抽头接 ADC，两端接 3V3 与 GND |
| 串口 | USART1，PA9/PA10，115200-8-N-1，接 USB-TTL（CH340） |

### 引脚分配

| 功能 | 引脚 | 电气模式 |
| --- | --- | --- |
| OLED SCL | PB8 | 开漏输出 + 上拉（软件 I2C） |
| OLED SDA | PB9 | 开漏输出 + 上拉（软件 I2C） |
| 按键 K1 | PB10 | 输入 + 上拉 |
| 按键 K2 | PB11 | 输入 + 上拉 |
| 按键 K3 | PA11 | 输入 + 上拉 |
| 按键 K4 | PA12 | 输入 + 上拉 |
| 电位器 RP1 | PA2 | 模拟输入（ADC2_IN2） |
| 电位器 RP2 | PA3 | 模拟输入（ADC2_IN3） |
| 电位器 RP3 | PA4 | 模拟输入（ADC2_IN4） |
| 电位器 RP4 | PA5 | 模拟输入（ADC2_IN5） |
| 串口 TX | PA9 | 复用推挽（USART1_TX） |
| 串口 RX | PA10 | 浮空输入（USART1_RX） |

**OLED 接线**：SCL → PB8，SDA → PB9，VCC → 3V3，GND → GND。
PB8/PB9 不是 STM32F103 的默认 I2C 引脚（I2C1 默认在 PB6/PB7），本项目用**软件模拟 I2C** 驱动。

**按键接线**：一端接引脚，另一端接 GND。引脚配成上拉输入，因此不按为高、按下为低。

**电位器接线**：每只电位器**三个脚都要接**——两端分别接 3V3 与 GND，中间抽头接 ADC 引脚。
抽头对地电压随旋钮在 0~3.3 V 之间连续变化，ADC 把它量化成 0~4095。
两端接反则读数方向相反；只接两端不接抽头则读数恒为 0 或恒为满量程。

**串口接线**：MCU 的 PA9(TX) 接 USB-TTL 的 RX，PA10(RX) 接 USB-TTL 的 TX，**GND 必须共地**。
TX 接 TX 的现象是「两边都收不到，且不报任何错」。

> 以上 12 个引脚**全部在 [PID_Pendulum.ioc](PID_Pendulum.ioc) 里声明**，由 CubeMX 生成配置代码。
> `bsp/` 里的模块只引用生成的 `KEY1_Pin` / `OLED_SCL_Pin` 之类的宏，不自己写引脚定义——
> 这样以后在 CubeMX 里加外设时，CubeMX 会在配置阶段就发现引脚冲突，
> 而不是等到运行时才发现两个功能抢同一个脚。

### 时钟树

```
HSE 8 MHz ──► PLL ×9 ──► SYSCLK 72 MHz
                          ├─ AHB  /1 ─► HCLK  72 MHz
                          ├─ APB1 /2 ─► PCLK1 36 MHz
                          └─ APB2 /1 ─► PCLK2 72 MHz
Flash latency: 2 WS
```

配置在 [Core/Src/main.c](Core/Src/main.c) 的 `SystemClock_Config()` 中，改时钟请改 [PID_Pendulum.ioc](PID_Pendulum.ioc) 后用 CubeMX 重新生成。

### 资源占用

Debug 构建下：FLASH 29616 B / 64 KB（45.2%），RAM 3912 B / 20 KB（19.1%）。

其中串口模块贡献约 +8.7 KB Flash（主要是 newlib 的 `printf` 格式化代码）
和 +1 KB RAM（两个 256 字节环形缓冲 + HAL 句柄）。

---

## 开发环境

本项目在 **VSCode + STM32CubeIDE for VSCode 扩展** 下开发，构建系统为 CMake + Ninja，不用 STM32CubeIDE 的 Eclipse 工程。

| 组件 | 版本 |
| --- | --- |
| STM32CubeMX | 6.15.0 |
| STM32Cube FW_F1 固件包 | V1.8.7 |
| GNU Tools for STM32 | 14.3.1+st.2 |
| CMake | 4.3.1+st.1 |
| Ninja | 1.13.2+st.1 |
| clangd | st-arm-clangd 21.1.0+st.2 |

上表中除 CubeMX 与固件包外，均由 STM32CubeIDE for VSCode 扩展以 bundle 形式自动安装，版本记录在 [.settings/bundles-lock.store.json](.settings/bundles-lock.store.json)。

---

## 目录结构

分层结构（从下往上）：

```
应用层     Core/              业务逻辑、main
             ↓
BSP 层     bsp/               板级支持：本板接了什么、接在哪
             ↓
器件驱动   driver/device/     某个具体芯片的驱动（ssd1306），只依赖抽象总线接口
             ↓
总线驱动   driver/bus/        I2C / SPI 等总线实现，直接贴着 HAL
             ↓
公共       driver/common/     全工程共用的类型（error_t）
             ↓
HAL        Drivers/           ST 的 HAL 与 CMSIS
```

```
.
├── Core/
│   ├── Inc/                    应用头文件（main.h、HAL 配置、中断声明）
│   └── Src/                    应用源码（main.c、中断处理、syscalls 等）
├── bsp/
│   ├── oled/                   板载 OLED 的组装
│   ├── key/                    4 路按键：消抖、事件上报
│   ├── pot/                    4 路电位器：ADC2 逐路采样、原始值/电压换算
│   ├── serial/                 串口：双向环形缓冲、中断收发、printf 重定向
│   └── tick/                   TIM1 1ms 系统节拍，供各模块挂载周期任务
├── driver/
│   ├── common/error.h          统一错误码 error_t
│   ├── bus/i2c/                软件模拟 I2C 总线
│   └── device/ssd1306/         SSD1306 器件驱动 + 字库
├── docs/                       模块详解（oled.md / key.md / pot.md / serial.md）+ 接口速查（api.md）
├── Drivers/
│   ├── CMSIS/                  ARM CMSIS 内核与设备头文件（第三方，Apache-2.0）
│   └── STM32F1xx_HAL_Driver/   ST HAL 驱动（第三方，BSD-3-Clause）
├── cmake/
│   ├── gcc-arm-none-eabi.cmake CMake 工具链文件（当前使用）
│   ├── starm-clang.cmake       ST 的 clang 工具链文件（备选）
│   └── stm32cubemx/            CubeMX 自动生成的源文件清单
├── .vscode/                    编辑器配置（已脱敏，见下）
├── CMakeLists.txt              顶层构建脚本
├── CMakePresets.json           Debug / Release 两个 preset
├── PID_Pendulum.ioc            CubeMX 工程定义
└── STM32F103XX_FLASH.ld        链接脚本
```

`Core/` 与 `cmake/stm32cubemx/` 中的多数文件由 CubeMX 生成。**手写代码请写在 `USER CODE BEGIN` / `USER CODE END` 注释块之间**，否则下次用 CubeMX 重新生成代码时会被覆盖。

`bsp/`、`driver/` 目录是手写的，CubeMX 不管理它们，不受重新生成影响。

---

## 模块：OLED（SSD1306）

> **完整的实现原理讲解见 [docs/oled.md](docs/oled.md)** —— 从 I2C 协议基础讲到每一段代码在做什么，面向嵌入式初学者。

### 为什么是软件模拟 I2C

1. **引脚**：OLED 接在 PB8/PB9，而 STM32F103 的 I2C1 默认在 PB6/PB7。想用硬件外设还得额外开 AFIO 重映射。
2. **可靠性**：STM32F103 的 I2C1 外设有已知 errata，总线异常时容易卡死。
3. **学习价值**：位翻转把起止条件、应答位、时钟同步这些 I2C 核心机制摆在明面上。

总线**目标**速率 400 kHz（[i2c_soft.h](driver/bus/i2c/i2c_soft.h) 的 `I2C_SOFT_DEFAULT_FREQ_HZ`）。
软件模拟下实际频率会受 GPIO 翻转开销影响而偏离目标值，**本项目没有用示波器实测过**。
刷新一屏（1024 字节）约 30 ms，对显示用途完全够用。

**PB8/PB9 这两个引脚在 `.ioc` 里是有声明的**（`Signal=GPIO_Output`，模式开漏 + 上拉），由 CubeMX 生成到 `MX_GPIO_Init()`。声明的作用是**占位**：让 CubeMX 在配置阶段就知道这两个脚已被占用，以后加外设时不会静默抢走它们。

而软件 I2C 在 `i2c_soft_init()` 里会**按同样的电气参数再配一遍**。看起来重复，这是有意的取舍：

| | 谁配 | 为什么 |
| --- | --- | --- |
| `.ioc` / `MX_GPIO_Init()` | CubeMX | 声明的唯一来源，负责占位与冲突检测 |
| `i2c_soft_gpio_config()` | 总线驱动 | 「开漏 + 上拉」是 I2C 协议的一部分，总线驱动必须自包含，才能脱离 CubeMX 换引脚复用、也才能在 PC 上用假总线做单元测试 |

代价是**两处电气参数必须保持一致**（开漏 / 上拉 / 高速），改一处就要改另一处。这是本工程唯一一处有意保留的双份配置，其余引脚一律只听 CubeMX 的。

> 用位翻转读 SDA 依赖 STM32F1 的一个特性：GPIO 处于输出模式时输入通道依然有效，因此把 SDA 配成开漏输出后可以直接读回线路真实电平，不必在输入/输出模式间来回切换。

### 分层怎么分的

| 文件 | 层 | 职责 |
| --- | --- | --- |
| [driver/common/error.h](driver/common/error.h) | 公共 | 全工程统一的 `error_t` |
| [driver/bus/i2c/i2c_if.h](driver/bus/i2c/i2c_if.h) | 接口 | 抽象 I2C 总线契约（`i2c_if_t`） |
| [driver/bus/i2c/i2c_soft.c](driver/bus/i2c/i2c_soft.c) | 总线驱动 | 位翻转实现，唯一允许碰 `HAL_GPIO_*` 的地方 |
| [driver/device/ssd1306/](driver/device/ssd1306/) | 器件驱动 | SSD1306 命令、显存、文本与图形，**不含任何 HAL 符号** |
| [bsp/oled/bsp_oled.c](bsp/oled/bsp_oled.c) | BSP | 指定 SCL=PB8 / SDA=PB9、从机地址 0x3C、延时实现 |

这样分的直接好处：`ssd1306.c` 只认 `i2c_if_t`，换 MCU 平台不用改；在 PC 上注入一个 mock 总线就能跑单元测试，不需要真硬件。

字库（[ssd1306_fonts.c](driver/device/ssd1306/ssd1306_fonts.c)）原样取自 [afiskon/stm32-ssd1306](https://github.com/afiskon/stm32-ssd1306)（MIT），未作修改。目前启用 6×8、7×10、11×18 三种字号，需要更大字号可在 [ssd1306_conf.h](driver/device/ssd1306/ssd1306_conf.h) 里打开。

### 烧录后屏幕应该显示什么

```
┌────────────────────────────┐
│ PID Pendulum               │  Font_7x10
│ scan: 0x3C ok              │  Font_6x8
│ ─────────────────────────  │  分隔线
│                            │
│       OLED OK              │  Font_11x18
│                            │
│ PB8=SCL PB9=SDA            │  Font_6x8
│ loop: 159                  │  Font_6x8，每秒 +2
└────────────────────────────┘
```

- **`scan: 0x3C ok`** 是开机时的总线自检结果。这一行是整个模块最有信息量的地方：能扫到 0x3C 说明 PB8/PB9 接线、上拉、位翻转时序、地址全部正确。若这里是 `scan: 0 dev!` 或整屏空白，就是接线或时序有问题。
- **`loop: N`** 每个主循环节拍 +1，与 PC13 LED 同节奏（500 ms 一次）。LED 闪一下、数字跳一下；两者节奏不一致就说明主循环某处被阻塞了。
- 若屏幕**完全没有反应**，固件会让 LED 以 10 Hz 快闪作为错误指示（`bsp_oled_init` 返回失败）——这样"黑屏"就能区分是屏幕没通、还是代码没跑到。

---

## 模块：按键（K1~K4）

> **完整的实现原理讲解见 [docs/key.md](docs/key.md)** —— 电气原理、消抖算法推演、跨上下文的事件机制、以及怎么在没有摄像头的情况下验证。

### 三个必须解决的问题

按键看似最简单，实际要处理三件事，每件都有陷阱：

| 需求 | 陷阱 | 做法 |
| --- | --- | --- |
| 读出「是否按下」 | 引脚不按时**悬空**，电平不确定 | 引脚内部上拉，按下接 GND |
| 过滤机械弹跳 | 触点闭合瞬间抖动 1~5 ms，一次按下读成十几次 | 1 ms 采样 + 连续 20 次一致才认 |
| 让应用知道「发生了什么」 | 只给电平，应用会漏掉快速按放 | 中断捕获**边沿**，主循环取走 |

### 为什么采样必须在 1 ms 中断里

刷一次屏要 30 ms，这期间主循环一次都没跑。而人按键约 50~200 ms——**如果整段落在刷屏期间，这次按下就被完全漏掉**，表现为偶发的「按键没反应」。

挂到 TIM1 的 1 ms 节拍上之后，采样间隔恒定，与主循环在忙什么无关。这也是**消抖能成立的前提**：消抖算的是「连续 20 次」，采样间隔如果忽长忽短，「20 次」对应的物理时间就不固定了。

### 分层

| 文件 | 职责 |
| --- | --- |
| [bsp/key/bsp_key.c](bsp/key/bsp_key.c) | 消抖状态机、事件上报。**不配置引脚** |
| [bsp/tick/bsp_tick.c](bsp/tick/bsp_tick.c) | TIM1 的 1 ms 中断与任务分发 |
| [PID_Pendulum.ioc](PID_Pendulum.ioc) | 4 个引脚的声明（上拉输入 + 标签） |
| `Core/Src/main.c` 生成部分 | `MX_GPIO_Init()` 里真正的引脚配置 |

**1 ms 节拍单独抽一层**，是因为它是系统基础设施而非按键的私产——后续 PID 控制环、传感器采样都会挂上来。接口就一个：

```c
error_t bsp_tick_register(bsp_tick_fn_t fn);   /* 挂一个 1ms 周期任务 */
```

### API

```c
key_event_t bsp_key_take_event(key_id_t id);   /* 取走事件（边沿，读取即清除） */
bool        bsp_key_is_pressed(key_id_t id);   /* 当前是否按着（电平） */
```

两种查询各有用途：`is_pressed` 回答「现在按着没有」（长按、组合键），`take_event` 回答「刚刚发生了什么」（计次、翻页）。只给电平的话，应用得自己记住上次状态来推断边沿——每个应用都要重写一遍，而且会漏事件。

### 烧录后屏幕应该显示什么

```
┌────────────────────────────┐
│ Key Test                   │
│ K1[ ] K2[ ] K3[ ] K4[ ]    │  ← 按下时方括号里显示 X
│ ──────────────────────────  │
│ K1:0        K2:0           │  ← 各键按下次数
│ K3:0        K4:0           │
│ down:-  up:-               │  ← 最近按下 / 松开的键
│ debounce 20 ms             │
└────────────────────────────┘
```

- 按下 K1：屏幕变成 `K1[X]` / `K1:1` / `down:K1`；松开后 `K1[X]` 变回 `K1[ ]`，并显示 `up:K1`
- **屏幕静止是正常的**——只在按键事件发生时重画，不按键就不动
- PC13 LED 每 500 ms 翻转一次，是「主循环还活着」的心跳
- 若某个模块初始化失败，LED 用闪烁次数指示是哪个（1 次=OLED，2 次=节拍，3 次=按键）

**如果实现反了**（不按显示 X、按了反而空）：说明按键是「按下接高电平」，改 [bsp_key.c](bsp/key/bsp_key.c) 里 `BSP_KEY_ACTIVE_LEVEL` 为 `GPIO_PIN_SET`，并同步把 `.ioc` 里四个引脚改成下拉。

---

## 模块：电位器（RP1~RP4）

> **完整的实现原理讲解见 [docs/pot.md](docs/pot.md)** —— 从 ADC 基础讲到每一段代码在做什么，含这次踩过的全部坑。

### 核心难点：STM32F1 的多通道 ADC 不能靠轮询逐路读

这是本模块最值得记的一条，依据是两处第一手材料：

> **RM0008 对 `SR.EOC` 的定义**是 "end of a **group** channel conversion"；
> **HAL 源码**里也写着 `As flag EOC is not set after each conversion`。
>
> 即：**扫描模式下 EOC 是"整组转换结束"才置位一次，不是每路一次**。
> `DR` 里永远只有最后一路的结果。想逐路取数，硬件上**只有 DMA 一条路**
> （RM0008：`When using scan mode, DMA bit must be set…`）。

而 **STM32F103 的 ADC2 没有 DMA 请求**（DMA1 请求表只挂了 ADC1；C8T6 又是中容量，
没有 DMA2）。所以本模块的方案是：**不用扫描模式，序列长度设为 1，一次只转一路**，
读之前用 `HAL_ADC_ConfigChannel()` 把 rank 1 切成目标通道。

顺带一个好处：序列长度是 1 时，`HAL_ADC_PollForConversion()` 内部会命中
"真正等 EOC"那条分支，可以直接用（扫描模式下它是不可用的）。

### 分层

| 文件 | 职责 |
| --- | --- |
| [bsp/pot/bsp_pot.c](bsp/pot/bsp_pot.c) | 自校准、逐路采样、原始值/电压换算 |
| [PID_Pendulum.ioc](PID_Pendulum.ioc) | ADC2 的声明（4 通道、扫描关闭、序列长度 1） |
| `Core/Src/main.c` 生成部分 | `MX_ADC2_Init()`、ADC 时钟分频 |
| `Core/Src/stm32f1xx_hal_msp.c` | ADC2 时钟使能、PA2~PA5 配成模拟输入 |

### API

```c
error_t  bsp_pot_init(void);                 /* 自校准，必须在 MX_ADC2_Init() 之后 */
error_t  bsp_pot_sample(void);               /* 采一次，4 路写进缓存（约 23 µs） */
uint16_t bsp_pot_raw(pot_id_t id);           /* 0~4095 */
uint16_t bsp_pot_millivolt(pot_id_t id);     /* 0~3300 mV */
```

**为什么"采样"和"读数"分开**：4 路共用一个 `DR`，ADC 是一问一答的。
若让 `bsp_pot_raw()` 自己触发转换，连续读 4 次就是 4 次硬件动作，
而且 4 个值来自 4 个不同瞬间——旋钮在转时会出现"撕裂"。

### 烧录后屏幕应该显示什么

```
┌────────────────────────────┐
│ Pot Test                   │
│ RP1 2198  ████████████████░ │  ← 数字右对齐，进度条按比例
│ RP2 2878  ████████████████░ │
│ RP3 1898  █████████████░░░░ │
│ RP4 1259  ██████████░░░░░░░ │
│ 0..4095  Vref 3.3V         │
└────────────────────────────┘
```

- **拧某个旋钮，只有它那一行的数字和进度条变**（四路独立）
- 不拧的时候数字**不抖**——死区 8 LSB 滤掉了 ADC 末位噪声，屏幕也不会空刷
- 拧到两端分别显示约 0 和 4095

若某个旋钮拧了没反应：先确认它是**三个脚都接好**（两端 3V3/GND、中间抽头接 ADC 引脚）。
只接两端不接抽头、或 3V3 那端没接上，都会读数异常。

---

## 模块：串口（USART1）

> **完整的实现原理讲解见 [docs/serial.md](docs/serial.md)** —— 从串口的时间预算讲到每一段并发代码在做什么，含这次踩过的全部坑。

### 核心难点：接收必须中断，而中断里不能等

115200 bps 下一帧 10 位，算下来**每个字节只隔约 87 µs**。
而本工程主循环里挂着 OLED 整屏刷新（软件 I2C 写 1024 字节显存），耗时是**毫秒**量级——
差两个数量级。所以：

- **接收**：必须中断驱动，没有商量余地；靠轮询必然丢数据。
- **发送**：做成中断驱动是为了**不阻塞主循环**。发 32 字节要 2.8 ms，
  用轮询发送会让主循环停住这么久，按键不响应、电位器不采样。

代价是 `bsp_serial_write()` 返回时数据**可能一个字节都还没发出去**。
这是非阻塞的固有语义，不是 bug——需要「返回即已发完」时用 `bsp_serial_flush()`。

### 环形缓冲为什么可以不加锁

```
发送：  主循环 ──写──> s_tx_head       中断 ──写──> s_tx_tail
接收：  中断   ──写──> s_rx_head       主循环 ──写──> s_rx_tail
```

**每个索引都只有一个写者**（SPSC 模型），两个写者写的是不同变量，
不存在「同时改一个变量」的竞态；而 Cortex-M3 对 16 位对齐变量的读写是单条指令。
所以绝大部分代码**不需要关中断**——这正是它能做到非阻塞的基础。

唯一需要关中断的是 `serial_tx_kick()`：它是 check-then-act，
`s_tx_busy` 同时被发送中断改写，不保护会出现「两次 `HAL_UART_Transmit_IT` 并发」，
窗口只有几条指令，**极难复现**。

### 分层

| 文件 | 职责 |
| --- | --- |
| [bsp/serial/bsp_serial.c](bsp/serial/bsp_serial.c) | 双向环形缓冲、中断回调、`printf` 重定向 |
| [PID_Pendulum.ioc](PID_Pendulum.ioc) | USART1 的声明（异步、PA9/PA10、115200-8-N-1） |
| `Core/Src/main.c` 生成部分 | `MX_USART1_UART_Init()` |
| `Core/Src/stm32f1xx_hal_msp.c` | USART1 时钟使能、PA9/PA10 复用配置、NVIC |
| `Core/Src/stm32f1xx_it.c` | `USART1_IRQHandler()` → `HAL_UART_IRQHandler()` |

### API

```c
error_t  bsp_serial_init(void);                                  /* 武装接收中断 */
uint32_t bsp_serial_write(const uint8_t *data, uint32_t len);    /* 非阻塞，返回实际写入数 */
uint32_t bsp_serial_read(uint8_t *out, uint32_t max);            /* 非阻塞，有多少取多少 */
uint32_t bsp_serial_rx_available(void);
uint32_t bsp_serial_tx_free(void);
uint32_t bsp_serial_rx_dropped(void);                            /* 诊断量，应恒为 0 */
error_t  bsp_serial_flush(uint32_t timeout_ms);                  /* 唯一的阻塞函数 */
```

**为什么没有「读一行」接口**：串口是**字节流**，一次 `read` 可能只拿到半个报文，
也可能拿到两条半。驱动不假装「一次调用 = 一条消息」，由上层处理这个事实。

`printf` 也可以直接用（`_write()` 已重定向到发送缓冲）：

```c
printf("rp1=%u rp2=%u\r\n", bsp_pot_raw(POT_ID_RP1), bsp_pot_raw(POT_ID_RP2));
```

⚠️ 缓冲满时 printf 会**截断输出**（返回短计数），不会阻塞；`%f` 默认不可用
（没开 `-u _printf_float`，开了多占约 6 KB Flash）；**绝不要在中断里调 printf**。

### 烧录后串口应该收到什么

```
PID_Pendulum tick=1 rp1=1952 rp2=2276 rp3=1101 rp4=1746
PID_Pendulum tick=2 rp1=1948 rp2=2279 rp3=1098 rp4=1743
...
```

- **每 500 ms 一行**；`rp1~rp4` 跟着电位器变化，且**静止时轻微跳动**（真实 ADC 噪声）
- **往串口发什么就收回什么**（回显）——用来验证接收链路
- 用的是临时自检代码 `app_serial_service()`，在 [Core/Src/main.c](Core/Src/main.c) 里，验证完可整块删掉

若收不到任何东西：先查 TX/RX 是否**交叉接线**、GND 是否共地、波特率是否 115200。
若只能收到第一行之后就没有了，检查是否漏了回调里的 `HAL_UART_Receive_IT()` 重新武装。

---

## 修改 CubeMX 配置

[PID_Pendulum.ioc](PID_Pendulum.ioc) 是**硬件配置的唯一来源**：引脚分配、电气模式、外设参数、中断使能都记在里面，`Core/` 下的初始化代码全部由它生成。要加外设、改引脚，都是改它然后重新生成。

### 总原则：CubeMX 能生成的，一律让 CubeMX 生成

**绝不手写会被重新生成覆盖的内容。** 手写代码和生成代码争夺同一份事实（引脚模式、外设参数、初始化顺序）时，两边必然漂移；而重新生成是**静默**的——不报错、不警告，直接改回去或丢掉，事后极难排查。

具体到本工程，分工是这样划的：

| 内容 | 归谁 |
| --- | --- |
| 引脚分配 / 电气模式 / 标签 | `.ioc`（唯一来源） |
| 外设参数（PSC、ARR……）、时钟树 | `.ioc` |
| `MX_xxx_Init()`、MSP、`HAL_NVIC_EnableIRQ()` | CubeMX 生成 |
| `HAL_xxx_MODULE_ENABLED`（[stm32f1xx_hal_conf.h](Core/Inc/stm32f1xx_hal_conf.h)） | CubeMX 生成 |
| [cmake/stm32cubemx/CMakeLists.txt](cmake/stm32cubemx/CMakeLists.txt) 源文件清单 | CubeMX 生成 |
| 中断服务函数（`TIM1_UP_IRQHandler` 等） | CubeMX 生成 |
| 模块逻辑（消抖、显存、PID……） | 手写，放 `bsp/` `driver/`，或生成文件的 `USER CODE` 块 |

引脚一律用生成的宏（`KEY1_Pin` / `KEY1_GPIO_Port`）引用，不硬编码端口和位号。

**判断某个文件会不会被覆盖**：CubeMX 用根目录的 [.mxproject](.mxproject) 记录它生成过的文件（`[PreviousGenFiles]` 段）。不在那个名单里的文件——比如顶层 [CMakeLists.txt](CMakeLists.txt)——不会被重新生成覆盖，可以放心手写。目前名单里是这 6 个：

```
Core/Inc/main.h            Core/Src/main.c
Core/Inc/stm32f1xx_it.h    Core/Src/stm32f1xx_it.c
Core/Inc/stm32f1xx_hal_conf.h   Core/Src/stm32f1xx_hal_msp.c
```

> 注：`Drivers/`、`startup_stm32f103xb.s`、`STM32F103XX_FLASH.ld` 不在 `[PreviousGenFiles]` 里，但同样会被生成覆盖，不要改。

### ⚠️ 不要手改 `.ioc` 来「加外设」

在 `.ioc` 里手写 `Mcu.IP3=TIM1` + `TIM1.Prescaler=71` 这种写法，**CubeMX 加载后会把整块静默丢弃**——不报错、不警告，`Mcu.IPNb` 还会退回原值。实测换键名、换位置、按字母序排列都没用。

原因：`.ioc` 是 CubeMX **内部状态的序列化**。`Mcu.IPn` 不是开关，而是「该外设已被激活」这个事实的记录。手写这个记录，CubeMX 重建不出对应状态就扔掉。

**外设必须通过 CubeMX 自己的接口激活。** 手改只适用于「改已有外设的参数」（改个 PSC、改个时钟源），而且改完必须验证。

### 两条可行路径

**路径 A：CubeMX GUI** —— 打开 `.ioc`，改配置，点 GENERATE CODE。最省心，GUI 会自己维护 `.ioc` 里那些白名单。

**路径 B：命令行** —— CubeMX 的脚本模式（`-q <脚本>`）支持 `set` 命令，可以全自动完成：

```text
config load "<工程目录>\PID_Pendulum.ioc"
set mode TIM1 "Internal Clock"            # 激活外设（模式名有空格要加引号）
set ip parameters TIM1 Prescaler 71
set ip parameters TIM1 Period 999
set pin PB10 GPIO_Input
set gpio parameters PB10 GPIO_PuPd GPIO_PULLUP
set gpio parameters PB10 GPIO_Label KEY1
config saveas "<工程目录>\PID_Pendulum.ioc"
project generate
exit                                      # 漏了这行 CubeMX 会挂住不退出
```

两个会让 CubeMX 卡住的坑：**路径含空格必须加引号**（不加会静默返回 KO）；**最后一行必须是 `exit`**。

> 想确认某个命令的语法，可以写个只有 `help`（或 `set`、`get`）的脚本跑一遍，CubeMX 会把用法打出来——比搜网页权威。
> `get modes <外设>` 能列出该外设所有可选模式名，**别猜名字**。

### 改完必须验证（三步，缺一不可）

无论走哪条路径，改完 `.ioc` 后都要做这三步：

**① 往返（load → save）** —— 看有没有键被丢弃或改写：

```bash
# stm32-cubemx skill 的脚本（工程目录和 .ioc 路径都收）
python <skills>/stm32-cubemx/scripts/cubemx.py roundtrip "<工程目录>"
```

看到 `OK: 写的参数全部被 CubeMX 原样保留了` 才算参数被接受。

**② 看 CubeMX 日志里有没有 `invalid value`**

这一步 **`roundtrip` 替代不了**：它比的是「键在不在、有没有被改写」，**比不出「值合不合法」**——CubeMX 对不认识的取值照样原样存进 `.ioc`。能说明问题的只有日志原话（`roundtrip` 会自动把这几行打出来，也可以自己看 `~/.stm32cubemx/STM32CubeMX.log`，每次运行覆盖写）：

```
IP not ready for code generation: USART1
Parameter (WordLength) has invalid value (UART_WORDLENGTH_8B)
```

出现这个就说明**那个外设的取值写错了**：它会进入 not-ready 状态，生成的 `MX_xxx_Init()` 会**少字段**。而零初始化后常常碰巧还是对的——于是错误被彻底掩盖，直到有人要改校验位才会炸。

**③ 去生成的 `.c` 里逐项找到对应代码** —— `MX_xxx_Init()`、`HAL_NVIC_EnableIRQ()`、`GPIO_InitStruct.Mode`，特别是**数一数字段够不够**（少了就是第②步没做到位）。

> ⚠️ **「参数被接受」≠「取值合法」≠「代码被生成」**——这三件事各自独立，而且失败时的表现都是「所有命令返回成功、硬件毫无反应」。只要有一道检查是「通过即放行」，就要再找一条独立通道去证伪。

### CubeMX 重新生成时，什么会保留

| 内容 | 结果 |
| --- | --- |
| `/* USER CODE BEGIN */` ~ `END` 之间的内容 | ✅ 保留 |
| 自己新增的目录（`bsp/`、`driver/`、`docs/`） | ✅ 保留（CubeMX 完全不管） |
| 顶层 `CMakeLists.txt` / `CMakePresets.json` | ✅ 保留（CubeMX 只生成一次） |
| `.vscode/` / `build/` | ✅ 保留 |
| `Core/`、`Drivers/`、`cmake/stm32cubemx/` | ⚠️ **会被重写** |

**由此得出的纪律**：业务代码要么写进 `USER CODE` 段，要么放进自己新增的目录。直接改 `Core/` 里 `USER CODE` 之外的地方，下次生成就没了。

---

## 构建与烧录

### 方式一：VSCode（推荐）

用 VSCode 打开本仓库根目录（**工作区根 == CMake 工程目录**，ST 的全套工具都假设这一点，用外层目录当工作区会出现找不到工程、调试起不来等问题）。

ST 扩展会自动完成 CMake 配置与构建；调试按 F5，用的是仓库内的 [.vscode/launch.json](.vscode/launch.json)（cortex-debug + ST-Link）。

### 方式二：命令行

```bash
cmake --preset Debug
cmake --build --preset Debug
```

产物为 `build/Debug/PID_Pendulum.elf`。要求 `arm-none-eabi-gcc` 在 `PATH` 中（见 [cmake/gcc-arm-none-eabi.cmake](cmake/gcc-arm-none-eabi.cmake)）。

烧录（需要 STM32CubeProgrammer 或 OpenOCD）：

```bash
STM32_Programmer_CLI -c port=SWD -w build/Debug/PID_Pendulum.elf -rst
```

---

## 关于 `.vscode/` 配置

`.vscode/` 里的 [launch.json](.vscode/launch.json) 与 [settings.json](.vscode/settings.json) 里**不包含任何写死的个人绝对路径**，工具位置统一用 `${env:CUBE_BUNDLE_PATH}` 引用。

这个环境变量由 STM32CubeIDE for VSCode 扩展在激活时设置，指向 `%LOCALAPPDATA%\stm32cube\bundles`。所以：

- 装了 ST 扩展的话开箱即用；
- 若 bundle 升级导致 `launch.json` 里写死的版本号（如 `stlink-gdbserver/7.14.0+st.2`）失效，去 `%LOCALAPPDATA%\stm32cube\bundles\` 看一眼实际版本号改掉即可；
- 若用别的方式管理工具链（比如自己装的 arm-none-eabi-gcc），需要把 `launch.json` 里那几行改成自己的路径。

---

## 许可

本工程自有代码以 [MIT 协议](LICENSE) 开源，Copyright (c) 2026 yuheng-null。

仓库内含第三方代码，**不适用 MIT**，各自遵循上游协议：

| 路径 | 版权方 | 协议 |
| --- | --- | --- |
| `Drivers/STM32F1xx_HAL_Driver/` | STMicroelectronics | BSD-3-Clause |
| `Drivers/CMSIS/` | ARM Limited | Apache-2.0 |
| `startup_stm32f103xb.s`、`STM32F103XX_FLASH.ld`、`Core/` 中由 CubeMX 生成的文件 | STMicroelectronics | BSD-3-Clause |

具体条款见各目录下的 `LICENSE.txt`，完整归属说明见 [NOTICE](NOTICE)。

> 注：本仓库不含任何江协科技教程的代码。教程使用标准库（SPL）实现，本项目为学习过程中用 HAL 库独立编写的实现。
