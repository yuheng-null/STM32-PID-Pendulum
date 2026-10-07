# STM32-PID-Pendulum

基于 **STM32F103C8T6** 的倒立摆学习工程，用 **HAL 库 + CMake** 实现。

学习路线参考 **江协科技** 的 PID 系列教程及其倒立摆项目。原教程使用 STM32 标准库（SPL），本工程在学习过程中改用 **HAL 库** 重新实现，因此代码属于独立编写，而非教程代码的移植。

目标是把这个仓库当成一份**可复现的学习记录**：每个阶段一个可编译、可烧录、可调试的状态，而不是一次性堆出一个能跑的成品。

---

## 当前进度

初始工程骨架，已完成：

- CubeMX 工程搭建（.ioc）、时钟树配置、SWD 调试引脚
- CMake + Ninja 构建系统，可由 STM32CubeIDE for VSCode 直接构建
- 板载 LED（PC13）1 Hz 闪烁 —— 用于验证「编译 → 烧录 → 运行」链路通了
- **0.96 寸 OLED（SSD1306，软件 I2C）** —— 第一个功能模块，见下文

倒立摆本体的机械、传感器、执行器部分**尚未开始**，后续按教程推进。

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

### OLED 接线

| OLED 引脚 | 接到 | 说明 |
| --- | --- | --- |
| SCL | **PB8** | 软件模拟 I2C 时钟 |
| SDA | **PB9** | 软件模拟 I2C 数据 |
| VCC | 3V3 | 模块内部有电荷泵，接 3.3V 即可 |
| GND | GND | |

PB8/PB9 并不是 STM32F103 的默认 I2C 引脚（I2C1 默认在 PB6/PB7），本项目用**软件模拟 I2C** 驱动，原因见下文。

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

Debug 构建下：FLASH 4704 B / 64 KB（7.18%），RAM 1584 B / 20 KB（7.73%）。

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
│   └── oled/                   板载 OLED 的接线与组装
├── driver/
│   ├── common/error.h          统一错误码 error_t
│   ├── bus/i2c/                软件模拟 I2C 总线
│   └── device/ssd1306/         SSD1306 器件驱动 + 字库
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

总线速度约 400 kHz，刷新一屏（1024 字节）约二十几毫秒，对显示用途完全够用。

软件 I2C 直接接管 PB8/PB9 的 GPIO 配置（开漏 + 上拉），**没有**在 CubeMX 里把这两个引脚配成 GPIO_Output —— 位翻转总线的引脚时序属于总线实现的一部分，由总线驱动自己配置更内聚，也避免 CubeMX 重新生成时把模式改回推挽。

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
