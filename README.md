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
- **TIM1 1 ms 系统节拍** —— 按键消抖与双环控制环都挂在其上
- **4 路电位器（RP1~RP4，ADC2 逐路采样）** —— 第三个功能模块，同时也是第一个模拟量输入
- **串口（USART1，双向中断 + 环形缓冲）** —— 第四个功能模块，供调参与上报数据
- **电机（TB6612 + TIM2 20 kHz PWM）** —— 倒立摆执行器
- **编码器（TIM3 编码器模式，AB 四倍频）** —— 横杆位置与转速反馈
- **角度传感器（电位器 + ADC1）** —— 摆杆角度反馈，倒立摆内环的输入
- **纯算法 PID（`algorithm/pid`）** —— 位置式，含积分限幅；不含任何硬件依赖，可在 PC 上单独编译
- **双环串级控制器（`app/control`）** —— 角度环 5 ms + 位置环 50 ms，状态机 + 倒下保护，挂在 1 ms 节拍上
- **串口调参控制台（`app/console`）** —— ASCII 行协议，供 agent 经串口在线改增益、看状态、串流数据
- **自动启摆（`app/control` 的 `SWING_UP` 状态）** —— 不用手扶，机构自己把摆杆荡起来再交给双环

**倒立摆的三个执行/传感驱动、双环 PID 的代码骨架都已在真硬件上验证**
（串口协议 30 项断言全过、STREAM 50.0 行/秒零空洞、倒下保护自动停机、
零增益 RUN 电机不动、串级方向确认为负反馈）。
**本轮只搭结构，增益尚未整定**——调参是下一阶段的任务。

> 当前屏幕上显示的是**双列调参界面**（左列角度环、右列位置环），
> 不再是最早那些单模块验证界面。

> 📖 **接口速查见 [docs/api.md](docs/api.md)** —— 每个模块向应用层暴露了哪些函数、什么前置条件、
> 能不能在中断里调、值域是多少，含一张「调用上下文总表」和主循环最小骨架。
>
> **写代码时查它；理解原理时看各模块的详解**：[oled.md](docs/oled.md) ·
> [key.md](docs/key.md) · [pot.md](docs/pot.md) · [serial.md](docs/serial.md) ·
> [motor.md](docs/motor.md) · [encoder.md](docs/encoder.md) · [angle.md](docs/angle.md)

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
| 电机 | 25GA370 有刷减速电机，12 V / 空载 620 RPM，经 **TB6612FNG** 的 A 路驱动 |
| 电机编码器 | AB 相正交霍尔编码器，四倍频后 **408 边沿 / 输出轴圈** |
| 角度传感器 | 360° 旋转电位器（SV01A103AEA01R00，10K），**有效角度 333°**，约 27° 盲区 |

> 电机、编码器、角度传感器来自**江协科技 PID 入门套件**的控制板与倒立摆结构件。
> 板上已固定接线（TB6612 的 STBY 直接接 3V3、角度传感器经导电滑环接到 SENSOR1），
> 引脚由 PCB 决定，不能自选。

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
| 电机 PWM | PA0 | 复用推挽（TIM2_CH1 → TB6612 PWMA） |
| 电机方向 A | PB12 | 推挽输出（TB6612 AIN1） |
| 电机方向 B | PB13 | 推挽输出（TB6612 AIN2） |
| 编码器 A 相 | PA6 | 输入 + 上拉（TIM3_CH1） |
| 编码器 B 相 | PA7 | 输入 + 上拉（TIM3_CH2） |
| 角度传感器 | PB0 | 模拟输入（ADC1_IN8） |

**OLED 接线**：SCL → PB8，SDA → PB9，VCC → 3V3，GND → GND。
PB8/PB9 不是 STM32F103 的默认 I2C 引脚（I2C1 默认在 PB6/PB7），本项目用**软件模拟 I2C** 驱动。

**按键接线**：一端接引脚，另一端接 GND。引脚配成上拉输入，因此不按为高、按下为低。

**电位器接线**：每只电位器**三个脚都要接**——两端分别接 3V3 与 GND，中间抽头接 ADC 引脚。
抽头对地电压随旋钮在 0~3.3 V 之间连续变化，ADC 把它量化成 0~4095。
两端接反则读数方向相反；只接两端不接抽头则读数恒为 0 或恒为满量程。

**串口接线**：MCU 的 PA9(TX) 接 USB-TTL 的 RX，PA10(RX) 接 USB-TTL 的 TX，**GND 必须共地**。
TX 接 TX 的现象是「两边都收不到，且不报任何错」。

**电机与编码器接线**：插在控制板的 **A 路**电机接口（25GA370 是 PH2.0 接头，插 `M3`）。
编码器 A/B 相经板上布线接到 PA6/PA7，引脚上拉**是必须的**——
若编码器输出是开漏结构，没有上拉就完全不计数，而且不报任何错。

**角度传感器接线**：插在控制板的 **J1（SENSOR1）**，另一端经导电滑环接摆杆。
**安装方向有要求**：摆杆自然下垂时，法兰联轴器的切面必须朝上；装反了倒立摆无法工作。

> 以上 18 个引脚**全部在 [PID_Pendulum.ioc](PID_Pendulum.ioc) 里声明**，由 CubeMX 生成配置代码。
> `bsp/` 里的模块只引用生成的 `KEY1_Pin` / `OLED_SCL_Pin` / `MOTOR_AIN1_Pin` 之类的宏，
> 不自己写引脚定义——这样以后在 CubeMX 里加外设时，CubeMX 会在配置阶段就发现引脚冲突，
> 而不是等到运行时才发现两个功能抢同一个脚。
>
> ⚠️ **例外**：ADC 通道、定时器通道、USART 的引脚 **CubeMX 不生成宏**
> （只有配成 GPIO 的引脚才有宏）。所以「哪个通道接什么」的对应表写在 BSP 里，
> 这本来就是 BSP 的职责。软件 I2C 的 PB8/PB9 是另一处例外，理由见 OLED 那节。

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

Debug 构建下：FLASH 38188 B / 64 KB（58.27%），RAM 3928 B / 20 KB（19.18%）。

其中：

- **串口模块**贡献约 +8.7 KB Flash（主要是 newlib 的 `printf` 格式化代码）
  和 +1 KB RAM（两个 256 字节环形缓冲 + HAL 句柄）
- **倒立摆三件套**（电机 / 编码器 / 角度传感器）贡献约 +2.5 KB Flash
  （TIM2/TIM3/ADC1 的 HAL 模块代码），RAM 几乎不变
- 从最初那份 29720 B 到现在，**新增的双环 PID + 串口控制台贡献了余下的约 8.5 KB**
  （`algorithm/`、`app/` 两个模块的代码）

> 注意：早先把测试程序从「电位器显示」换成「倒立摆驱动验证」时，
> **Flash 反而降了 2.4 KB** —— 因为新程序不再调用 `printf`，
> 链接器把 newlib 的格式化代码整块回收了。**"删掉 printf 调用"确实能省 Flash**，
> 前提是没有其它地方再用它。（串口控制台是**自己写的定点打印**，也没引入 `printf` 的浮点格式化。）

> 数据用 `arm-none-eabi-size build/Debug/PID_Pendulum.elf` 取：Flash = `text + data`，RAM = `data + bss`。

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
应用层     Core/ + app/       业务逻辑：main、双环控制器、串口调参控制台
             │
             ├──► 算法层  algorithm/   纯算法（PID）：不依赖任何硬件，可在 PC 上单独编译
             │
             └──► BSP 层  bsp/         板级支持：本板接了什么、接在哪
                    ↓
器件驱动   driver/device/     某个具体芯片的驱动（ssd1306），只依赖抽象总线接口
             ↓
总线驱动   driver/bus/        I2C / SPI 等总线实现，直接贴着 HAL
             ↓
公共       driver/common/     全工程共用的类型（error_t）
             ↓
HAL        Drivers/           ST 的 HAL 与 CMSIS
```

依赖方向是单向的：上层可以往下依赖，反之不行。特别是
**`algorithm/` 不许 include 任何 `bsp_*` 或 HAL 头文件**——这是它能在 PC 上单独编译的前提。

```
.
├── Core/
│   ├── Inc/                    应用头文件（main.h、HAL 配置、中断声明）
│   └── Src/                    应用源码（main.c、中断处理、syscalls 等）
├── algorithm/
│   └── pid/                    纯 PID 算法（位置式 + 积分限幅），零硬件依赖
├── app/
│   ├── control/                双环串级控制器：状态机、倒下保护、可在线改的参数表
│   └── console/                串口调参协议（SET/GET/STAT/RUN/STOP/ZERO/TARGET/STREAM/HELP）
├── bsp/
│   ├── oled/                   板载 OLED 的组装
│   ├── motor/                  电机：TB6612 方向 + TIM2 20kHz PWM
│   ├── encoder/                编码器：TIM3 四倍频、位置累加
│   ├── angle/                  角度传感器：ADC1 单通道采样
│   ├── key/                    4 路按键：消抖、事件上报
│   ├── pot/                    4 路电位器：ADC2 逐路采样、原始值/电压换算
│   ├── serial/                 串口：双向环形缓冲、中断收发、printf 重定向
│   └── tick/                   TIM1 1ms 系统节拍，供各模块挂载周期任务
├── driver/
│   ├── common/error.h          统一错误码 error_t
│   ├── bus/i2c/                软件模拟 I2C 总线
│   └── device/ssd1306/         SSD1306 器件驱动 + 字库
├── docs/                       接口速查（api.md）+ 模块详解（oled / key / pot / serial / motor / encoder / angle）
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

`bsp/`、`driver/`、`algorithm/`、`app/` 目录是手写的，CubeMX 不管理它们，不受重新生成影响。

---

## 模块：OLED（SSD1306）

> **完整的实现原理讲解见 [docs/oled.md](docs/oled.md)** —— 从 I2C 协议基础讲到每一段代码在做什么，面向嵌入式初学者。

### 为什么是软件模拟 I2C

1. **引脚**：OLED 接在 PB8/PB9，而 STM32F103 的 I2C1 默认在 PB6/PB7。想用硬件外设还得额外开 AFIO 重映射。
2. **可靠性**：STM32F103 的 I2C1 外设有已知 errata，总线异常时容易卡死。
3. **学习价值**：位翻转把起止条件、应答位、时钟同步这些 I2C 核心机制摆在明面上。

总线**目标**速率 400 kHz（[i2c_soft.h](driver/bus/i2c/i2c_soft.h) 的 `I2C_SOFT_DEFAULT_FREQ_HZ`）。
软件模拟下实际频率会受 GPIO 翻转开销影响而偏离目标值，**本项目没有用示波器实测过**。
**实测刷新一屏（1024 字节）约 122 ms** —— 软件 I2C 实际只能跑到约 100 kHz
（1024×9 bit ÷ 100 kHz ≈ 92 ms，加命令字节与函数开销正好是这个量级）。
⚠️ 早先文档里写的「约 30 ms」是错的，那对应约 300 kHz，这个软件 I2C 达不到。
这个数**直接决定主循环的节奏**（刷屏周期 500 ms、STREAM 期间停刷屏），见下文各节。

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

> ⚠️ 这是 OLED 模块**独立验证阶段**的界面。当前主程序已换成双列 PID 调参界面
> （见 [双环控制器](#模块双环控制器appcontrol) 一节），要复现上面这个界面得把显示逻辑改回去。

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

刷一次屏要 122 ms，这期间主循环一次都没跑。而人按键约 50~200 ms——**如果整段落在刷屏期间，这次按下就被完全漏掉**，表现为偶发的「按键没反应」。

挂到 TIM1 的 1 ms 节拍上之后，采样间隔恒定，与主循环在忙什么无关。这也是**消抖能成立的前提**：消抖算的是「连续 20 次」，采样间隔如果忽长忽短，「20 次」对应的物理时间就不固定了。

### 分层

| 文件 | 职责 |
| --- | --- |
| [bsp/key/bsp_key.c](bsp/key/bsp_key.c) | 消抖状态机、事件上报。**不配置引脚** |
| [bsp/tick/bsp_tick.c](bsp/tick/bsp_tick.c) | TIM1 的 1 ms 中断与任务分发 |
| [PID_Pendulum.ioc](PID_Pendulum.ioc) | 4 个引脚的声明（上拉输入 + 标签） |
| `Core/Src/main.c` 生成部分 | `MX_GPIO_Init()` 里真正的引脚配置 |

**1 ms 节拍单独抽一层**，是因为它是系统基础设施而非按键的私产——后来的 PID 控制环就挂在它上面。接口就一个：

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

> ⚠️ 这是按键模块**独立验证阶段**的界面。当前主程序已换成双列 PID 调参界面
> （见 [双环控制器](#模块双环控制器appcontrol) 一节），要复现上面这个界面得把显示逻辑改回去。
> 按键的**语义**现在也变了：K1 启停控制，K2/K3 调位置目标，K4 位置清零。

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

> ⚠️ 这是电位器模块**独立验证阶段**的界面。当前主程序已换成双列 PID 调参界面
> （见 [双环控制器](#模块双环控制器appcontrol) 一节），要复现上面这个界面得把显示逻辑改回去。
> 电位器现在只在主循环里采（没有进控制环），`bsp/pot` 也**没有**非阻塞接口。

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

> 上面那段回显/周期打印是串口模块自己的**临时自检代码**，已经删掉了
> （当前主程序在做倒立摆驱动验证）。要复现把 `bsp_serial_read/write` 接回主循环即可。
> `bsp_serial_init()` 仍然调用，`printf` 重定向仍然可用。

若收不到任何东西：先查 TX/RX 是否**交叉接线**、GND 是否共地、波特率是否 115200。
若只能收到第一行之后就没有了，检查是否漏了回调里的 `HAL_UART_Receive_IT()` 重新武装。

---

## 模块：电机（25GA370 + TB6612）

> **完整的实现原理讲解见 [docs/motor.md](docs/motor.md)** —— 从 PWM 调速原理讲到
> 每一段代码在做什么，含 TB6612 的四态真值表与这次踩过的全部坑。

### 核心难点：TB6612 有四种输出状态，「停」不是一个

电机由三个信号控制：一路 **PWM**（速度）+ 两个**方向脚**（方向）。
两个方向脚的组合决定四种状态，**其中两种都叫"停"，但电气状态完全不同**：

| AIN1 | AIN2 | 状态 | 电机 |
| :---: | :---: | --- | --- |
| 0 | 0 | Stop | **输出高阻 → 惯性滑行**（可自由转动） |
| 1 | 1 | Short brake | **绕组短接 → 电磁刹车**（阻力很大） |
| 0 | 1 | 正转 | 转速 ∝ 占空比 |
| 1 | 0 | 反转 | 转速 ∝ 占空比 |

所以驱动提供了 `bsp_motor_coast()` 和 `bsp_motor_brake()` 两个**语义不同**的停机接口。
参考实现里 `Motor_SetPWM(0)` 走的是"正转 0%"分支，严格说既不是滑行也不是刹车。

### PWM 为什么是 20 kHz

```
TIM2 时钟 72 MHz ÷ (PSC+1) ÷ (ARR+1)
           = 72e6 ÷ 2 ÷ 1800 = 20 000 Hz
```

20 kHz 在人耳上限之上——频率落在音频段电机会持续啸叫。

**为什么 PSC/ARR 取 1 / 1799 而不是常见的 35 / 99**：两者都是 20 kHz，
但占空比分辨率差 18 倍（**1% → 0.056%**）。控制环在平衡点附近只输出零点几个百分点，
1% 的步长会让电机在正反 1% 之间来回跳（量化抖动）。

⚠️ **代价：参考工程的 PID 增益不能照抄，要乘 18**（参考量程 ±100，本工程 ±1800）。
`control.c` 里的默认增益已经乘过 18。

### API

```c
error_t bsp_motor_init(void);
error_t bsp_motor_set_duty(int16_t duty);   /* ±1800（CCR 原始值，1800 = 100%），超出夹紧 */
error_t bsp_motor_coast(void);              /* 高阻滑行 */
error_t bsp_motor_brake(void);              /* 绕组短接刹车 */
int16_t bsp_motor_duty(void);
```

⚠️ **`HAL_TIM_PWM_Start()` 必须显式调用** —— `MX_TIM2_Init()` 只把参数写进寄存器，
不使能输出比较和计数器。漏掉的现象是「编译烧录全成功、电机纹丝不动」。

### 怎么验证它在工作

见 [docs/motor.md](docs/motor.md) 的「怎么测试」——四层证据：生成代码 → 运行时变量 →
方向一致性 → 用转速反推占空比。

> ⚠️ 当前屏幕上显示的**不是**电机测试界面，而是双列 PID 调参界面，见下面的
> [双环控制器](#模块双环控制器appcontrol) 一节。

---

## 模块：编码器（TIM3 四倍频）

> **完整的实现原理讲解见 [docs/encoder.md](docs/encoder.md)** —— 从 AB 正交信号讲到
> 模 2^16 求增量的写法，含 `HAL_TIM_Encoder_Start()` 那个会静默少计一半的坑。

### 核心难点：为什么不清零计数器

常见写法是「读 CNT，然后清零」，用返回值当增量。**它会在读与清零之间丢边沿**，
而且这个窗口关不掉。

本工程改成**只读不清**，自己记住上次的值，用**模 2^16 相减**求增量：

```c
int16_t d = (int16_t)(uint16_t)(now - s_last);   /* 先按 uint16 环绕相减，再转有符号 */
```

好处：不丢边沿、采集时机任意（累计的是增量之和，采样频率再低位置也精确）、
不需要关中断（没有"读-改-写"）。

### API

```c
error_t bsp_encoder_init(void);
error_t bsp_encoder_update(void);    /* 采集：推进游标、累加（形状同 bsp_pot_sample） */
int32_t bsp_encoder_total(void);     /* 累计位置（边沿），正负表示方向 */
int16_t bsp_encoder_delta(void);     /* 上次增量，除以周期即转速 */
error_t bsp_encoder_reset(void);     /* 清零（会同时对齐游标） */
error_t bsp_encoder_set_invert(bool);/* A/B 接反时的软件兜底，正常不用 */
```

⚠️ **`HAL_TIM_Encoder_Start()` 必须传 `TIM_CHANNEL_ALL`** ——
传单个通道只使能一个 `CCxE`，四倍频会**静默少计一半**。

⚠️ **电机正转（`duty > 0`）必须对应 `total` 增大**。这条错了双环 PID 会变成正反馈。

---

## 模块：角度传感器（电位器 + ADC1）

> **完整的实现原理讲解见 [docs/angle.md](docs/angle.md)** —— 从"为什么用电位器"讲到
> 标定方法，含「为什么不能用理想值 2048」的完整论证。

### 核心难点：它是一只电位器，不是陀螺仪

套件的角度传感器**不是** MPU6050、**也不是** AS5600，而是**一只 360° 旋转电位器**
（手册原话：「本套件的角度传感器采用的是电位器测角度的方案」）。
摆杆转动带动抽头，PB0 上就是一个随角度连续变化的电压。

**这带来两个必须知道的特性**：

- **有效角度只有 333°**，两端之间有一小段约 27° 的**盲区**，跨过去读数会突变。
  倒立摆的工作范围就在竖直附近，永远不会用到盲区，**驱动层不做任何补偿**
- **驱动层不提供"角度是多少度"** —— 竖直零点取决于机械安装相位，属应用层标定

### 本台设备的实测标定值

| 位置 | `raw` |
| --- | --- |
| 摆杆竖直向上（**手扶**） | 2086（重复性 2080~2090） |
| 水平向左（逆时针 90°） | 960 |
| 水平向右（顺时针 90°） | 3160 |
| **自由下垂（松手）** | **1883 —— 盲区里的无效值，不是竖直** |

由此得 **12.22 LSB/度**，满量程折算 **335°** —— 对上手册标称的 **333°**（差 0.6%）。
这是一条很强的交叉验证：ADC 通道、线性、传感器型号三者同时被印证。

### API

```c
error_t  bsp_angle_init(void);

/* 阻塞式：主循环 / 测试用 */
error_t  bsp_angle_sample(void);        /* 约 5.7 µs，有界自旋 */

/* 非阻塞式（两步）：中断 / 控制环用 */
error_t  bsp_angle_trigger(void);       /* 启动转换，立即返回 */
error_t  bsp_angle_poll(void);          /* ERR_OK 才代表拿到新值 */

uint16_t bsp_angle_raw(void);           /* 0 ~ 4095 */
uint16_t bsp_angle_millivolt(void);     /* 0 ~ 3300 mV */
```

⚠️ **两套接口按上下文选**：`bsp_angle_sample()` 要干等 5.7 µs，**只放主循环**；
中断里用 `bsp_angle_trigger()` + `bsp_angle_poll()`（纯寄存器操作，不依赖 `HAL_GetTick()`）。
**控制环用的就是这一对**——它必须挂在 1 ms 节拍上（主循环刷一屏 122 ms 会毁掉 5 ms 的周期）。

⚠️ **盲区正对摆杆自然下垂的方向**，所以「松手让它垂着」读到的（1883）是**无效值**——
它稳定不漂，看起来像个正常读数，其实不是。

⚠️ **中心值不能用理想的 2048，也不能用「自由下垂」测**。
2048 是电位器的**电气中点**，与「摆杆竖直」没有任何物理必然联系；
而「自由下垂」在本套件上读到的是盲区残值（见上表）。
参考实现给的是一个**区间**（1900~2200）而不是一个数，正说明它要被测出来。
**正确做法：手扶摆杆到竖直附近读 `raw`，多测几次取平均。**

---

## 模块：纯算法 PID（`algorithm/pid`）

> **实现说明见 [docs/api.md](docs/api.md) 的 2.9 与 [pid.h](algorithm/pid/pid.h) 文件头。**

### 为什么单独抽一层

它**不包含任何硬件相关的头文件**，输入是 float、输出是 float，只依赖 `<stdbool.h>`。好处：

- 能在 **PC 上用 gcc 直接编译、跑单元测试**——积分饱和、符号写反这类错误在 PC 上跑一遍阶跃响应就能看出来，比烧板子快得多
- 换被控对象（平衡车、云台）时这一层原封不动

### 用位置式，不用增量式

增量式的输出是「上一次输出 + 增量」，本身带积分性质：一旦因限幅、丢步、模式切换导致输出漂移，
误差会**永久累积**在输出上，而且没法「直接接管输出」。倒立摆需要「停下时输出必须真的是 0」，
位置式更合适。

### 比参考实现多两件事

| 增强 | 为什么 |
| --- | --- |
| **积分限幅**（抗饱和） | 只有输出限幅不够：输出被夹住后误差仍在，积分项会继续无上限累加；等误差反向时要花很久才退回来，表现为「该停时刹不住、过冲大」 |
| 限幅做成**结构体字段** | 让调用者（或调参的 agent）能直接控制，而不是塞在算法内部 |

保留了参考工程的「**Ki == 0 时把积分清零**」——整定时先把 Ki 设 0 看纯 PD，
不清的话回头把 Ki 调非 0 会被积压值「踹一脚」。

> ⚠️ 公式是**未归一化到时间**的简化形式（`out = Kp·e + Ki·Σe + Kd·Δe`）。
> `Ki`/`Kd` 是「每个控制周期」的量纲——**改调用周期 = 改 Ki/Kd 的实际效果**。

---

## 模块：双环控制器（`app/control`）

> **完整的设计说明见 [control.h](app/control/control.h) 文件头**——那是本工程写得最详细的一份，包含被控对象、串级分工、两个判定窗口的来龙去脉。

### 串级结构

```
外环（位置环，50 ms）—— "横杆该在哪"   → 输出**角度修正量**（±100 码）
内环（角度环，  5 ms）—— "摆杆该在什么角度" → 输出 **PWM**

AnglePID.Target = CENTER − LocationPID.Out     ← 外环去挪内环的目标
```

物理含义：横杆偏离目标越远，外环就把「摆杆该立的角度」偏得越多；内环为了够到那个偏掉的角度，
就得让横杆转过去——于是摆杆始终立着，而横杆慢慢挪回目标位置。

**内环必须比外环快 10 倍以上**（这里 5 ms vs 50 ms）：外环把内环当成「说去哪就去哪」的理想执行器，
内环跟不上，整个串级结构就不成立。

### 为什么挂在 1 ms 中断里

主循环里刷一次 OLED 实测 **122 ms**，而角度环周期是 5 ms。放在主循环里，实际周期会在 5~135 ms
之间乱跳，等于给控制器灌了一路巨大的噪声，根本立不住。所以控制环挂在 1 ms 节拍上，
由它分频出 5 ms / 50 ms。**代价是控制环跑在中断里，必须快进快出**：不许阻塞、不许打印、不许刷屏。

### ⚠️ 两个判定窗口（本工程最要紧的一处安全设计）

电位器盲区正对摆杆自然下垂方向。本台实测：竖直向上 **2086**，自由下垂 **1883**——
**只差 203 码**，而官方的倒下窗口 `±500` 是 `[1586, 2586]`，**1883 落在里面**。
也就是说「角度在 ±500 内」根本不能证明摆杆是立着的。

所以拆成两个目的不同的窗口：

| 窗口 | 何时检查 | 半宽度 | 防什么 |
| --- | --- | --- | --- |
| `START` | 只在 `RUN` 瞬间 | ±150（≈12°） | 垂着/躺着被误启动（1883 被排除在外 ✓） |
| `RANGE` | 运行中每 1 ms | ±500（≈41°） | 立着立着倒了 |

**只用 ±150**：正常运行中的摆动就被误停，调参做不下去。**只用 ±500**：躺着启动挡不住。

**残留风险**：若整台支架不竖直，「自由下垂」读数会随横杆方位变化，±150 未必总能把它排除掉。
所以**规矩是：用 `RUN` 启动前要先用手把摆杆扶到竖直附近**——这一条对自动化脚本同样成立。
（不想手扶就用 `SWING`：它自己把摆杆荡进这个窗口，见下一节。）

### 自动启摆（swing-up）

**不需要再用手扶摆杆了。** 算法移植自江协科技 `16-倒立摆-自动启摆` 的 1 ms 状态机：
每 40 ms 采一次角度，检测到摆杆到达**摆动顶点**就打一组瞬时脉冲（先正向 `SWT` 毫秒、
再反向 `SWT` 毫秒）给摆杆注能量，反复几次把摆杆荡到竖直附近，进窗口后自动交给双环 PID。

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `SWP` | 630 | 脉冲占空比（630 = 参考的 **35%** × 18，同样是量程换算） |
| `SWT` | 100 ms | 单个脉冲的宽度（参考的推荐区间 80~120） |

两个都能串口在线改——参考自己也注明这两个值**要按设备标定**。

**启动方式三种都行**：按 K1、串口发 `SWING`、或扶好之后发 `RUN`。
`SWING` 会先判断角度：已在 `CENTER ± START` 内就直接进双环，否则先启摆。

> ⚠️ **移植时改正了参考的一处条件，这个坑必须记着。**
> 参考用 `CENTER ± CENTER_RANGE`（±500）判断"摆杆已经立起来了"，但本台实测
> **自由下垂读数 1883 落在 [1586, 2586] 里面** —— 照搬的结果是启摆刚开始判定
> 就把垂着的摆杆判成"立好了"，直接交给默认增益的双环、电机猛冲。
> （2026-10-09 实测踩到过。）本工程改用 `CENTER ± START`（±150），
> 于是**交棒条件 == RUN 的准入条件**，两个窗口在这里合流。

> ⚠️ **超时兜底**：参考会一直泵到成功为止，本工程加了 30 秒超时 —— 无人看管时
> 不能让电机一直以 35% 占空比反复冲击。超时后自动停机，`STAT` 的 `SWR=2` 说明是超时停的。

### 默认增益

`control.c` 的默认值 = 参考工程 Mode3 的官方值 **× 18**（PWM 量程换算，见上面电机那节），
`OFFSET` 5×18 = 90。**这些只是整定的起点，本轮没有调参。**

### 当前屏幕显示什么

刷屏周期 500 ms，**双列**（左列角度环、右列位置环）：

```
┌────────────────────────────┐
│ Pendulum              RUN  │
│ AKP   4.50   PKP   9.36    │  ← 两个环的增益（串口改的就是这几项）
│ AKI   0.16   PKI   0.18    │
│ AKD   7.38   PKD  82.08    │
│ Ang   2086   Loc      0    │  ← 摆杆角度 / 横杆累计位置
│ ATr   2086   LTr      0    │  ← 两环的目标值
│ AOu    123   POu   0.00    │  ← 两环的输出
└────────────────────────────┘
```

- **按键**：K1 启停控制，K2/K3 位置目标 ±408（横杆一圈），K4 位置清零并停机
- 未按 K1 之前电机**不会转**
- **`ATr` 在运行中会随外环输出微动**——那是正常的，正是串级结构在「挪动内环目标」；
  它一直等于 CENTER 反而说明外环没在干活
- 真正写到电机上的 `PWM` 不在屏上，要看用串口 `STAT`

---

## 模块：串口调参控制台（`app/console`）

> **协议规格完整写在 [console.h](app/console/console.h) 文件头。**

ASCII 行协议，**给「调参 agent」用的遥控器**。参考工程里没有任何串口改参数的实现
（15-倒立摆 的注释原文写着「当前程序暂未使用串口」），所以这套协议是本工程自己设计的。

```
SET <参数名> <值> | GET <名>|ALL | STAT | RUN | SWING | STOP | ZERO
TARGET <整数> | STREAM <0|1> | HELP
参数：AKP AKI AKD PKP PKI PKD CENTER RANGE START OFFSET SWP SWT
```

`RUN` 与 `SWING` 的区别：`RUN` **严格**（角度不在 `START` 窗口内就回 `ERR NOT_READY`），
`SWING` **自动**（在窗口内则等价于 RUN，否则先启摆）。保留两套是为了让
「角度不对就该拒」这条判断仍能被单独测到，也让"会真的甩电机"必须被显式要求。

`STAT` 末尾有两个状态字段：`ST`（0 停 / 1 双环 / 2 启摆中）和
`SWR`（上次启摆的结局：0 没启摆过 / 1 成功 / 2 超时 / 3 被打断）。
分两个字段是必要的——**`RUN` 只有 0/1，它分不出「停着」和「正在启摆」**。

响应只有 `OK ` / `ERR ` 两种前缀；`ERR` 后面必定带原因和出错的那个词
（如 `ERR UNKNOWN_PARAM AKQ`、`ERR OUT_OF_RANGE START`），让 agent 能自我纠正——
只回错误码会让它反复重试同一个错误命令。

### 三条必须知道的约定

1. **发一条、等一条响应，再发下一条**。刷屏那一下主循环有 122 ms 不取数据，
   而接收环形缓冲只有 256 字节——**连发必定丢，而且丢得不声不响**。
2. ⚠️ **`RUN` 会让电机真的转**。想在不扰动机构的前提下测逻辑，把增益和 `OFFSET` 全设 0——
   控制环照常跑（分频、采样、倒下判定都执行），但输出恒为 0。
3. ⚠️ **`RUN` 前必须手扶摆杆到竖直附近**。`RUN` 时会用 `START` 检查当前角度，
   不在窗口内回 `ERR NOT_READY ANGLE_OUT_OF_WINDOW`。**agent 应先 `STAT` 确认
   `ANGLE` 靠近 `CENTER`，而不是假设它一定在。**
   （不想手扶就用 `SWING` —— 它自己把摆杆荡起来。）

### STREAM

`STREAM 1` 后按 20 ms 周期吐一行 CSV（列序与 `STAT` 一致），供上位机画曲线。
串流期间主循环**完全停刷屏**——把这 122 ms 让给串口，换来满速 **50 行/秒、零空洞、
等间距**的采样（对 PID 调参有实际价值：横轴时间戳均匀，才能从波形的周期和衰减判断增益是否合适）。
`STREAM 0` 立刻恢复刷屏。

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
| 模块逻辑（消抖、显存、PID、双环控制……） | 手写，放 `algorithm/` `app/` `bsp/` `driver/`，或生成文件的 `USER CODE` 块 |

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
| 自己新增的目录（`bsp/`、`driver/`、`algorithm/`、`app/`、`docs/`） | ✅ 保留（CubeMX 完全不管） |
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
