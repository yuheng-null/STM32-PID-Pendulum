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

```
.
├── Core/
│   ├── Inc/                    应用头文件（main.h、HAL 配置、中断声明）
│   └── Src/                    应用源码（main.c、中断处理、syscalls 等）
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
