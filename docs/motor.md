# 电机（BSP Motor）实现详解

本文逐层拆解本仓库倒立摆电机的实现：有刷直流电机怎么调速、TB6612 的四种输出状态、
PWM 频率为什么选 20 kHz、初始化顺序为什么不能调换，以及怎么在没有示波器的情况下验证它。

**面向读者**：嵌入式初学者。假设你会 C（结构体、指针），
但不要求懂定时器的比较输出（OC）细节。

**配套代码**：

| 文件 | 层 | 职责 |
| --- | --- | --- |
| [PID_Pendulum.ioc](../PID_Pendulum.ioc) | CubeMX | TIM2 的声明（PWM CH1、PSC/ARR）、PB12/PB13 的 GPIO 声明 |
| [Core/Src/main.c](../Core/Src/main.c) 生成部分 | CubeMX 生成 | `MX_TIM2_Init()`、`MX_GPIO_Init()` 里的方向脚配置 |
| [Core/Src/stm32f1xx_hal_msp.c](../Core/Src/stm32f1xx_hal_msp.c) | CubeMX 生成 | TIM2 时钟使能、**PA0 配成复用推挽** |
| [Core/Inc/main.h](../Core/Inc/main.h) | CubeMX 生成 | `MOTOR_AIN1_Pin` / `MOTOR_AIN2_Pin` 宏 |
| [bsp/motor/bsp_motor.h](../bsp/motor/bsp_motor.h) · [.c](../bsp/motor/bsp_motor.c) | BSP | 占空比/方向映射、两种停机语义 |
| [Core/Src/main.c](../Core/Src/main.c) | 应用 | 按键控制启停与占空比 |

---

## 目录

- [一、要解决什么问题](#一要解决什么问题)
- [二、电机驱动基础：三个必须先搞清的概念](#二电机驱动基础三个必须先搞清的概念)
- [三、CubeMX 配了什么](#三cubemx-配了什么)
- [四、手写的部分](#四手写的部分)
- [五、怎么测试](#五怎么测试)
- [六、设计取舍一览](#六设计取舍一览)
- [附：这次踩过的坑](#附这次踩过的坑)

---

## 一、要解决什么问题

### 1.1 需求与约束

倒立摆的**执行器**：通过控制一个直流电机的转速和转向，反过来驱动摆杆保持平衡。

先把**手上的资源和限制**列清楚，这一步决定后面所有选择：

| 问题 | 答案 |
| --- | --- |
| 电机是什么？ | 25GA370 有刷直流减速电机，12 V、空载 620 RPM |
| 谁驱动它？ | 套件控制板上的 **TB6612FNG**（双路 H 桥，本套件只用 A 路） |
| 控制板上电机接在哪些 MCU 脚？ | **PA0**（PWMA，速度）+ **PB12/PB13**（AIN1/AIN2，方向） |
| 怎么确认的？ | 逐脚读套件原理图 `PID电机驱动及控制板-V1.0.pdf`，并用 CubeMX 的 F103 设备 XML 核对引脚复用能力 |
| 需要几个信号？ | 2 个方向脚 + 1 路 PWM |
| STBY 引脚要管吗？ | **不用** —— 板上直接接了 3V3，芯片常使能 |
| 占空比给多少档？ | 0~100%，直接当 CCR 用（见 4.2） |

关于**引脚是怎么确定的**，值得单独说一句，因为这里是本项目第一次出现"同一根线有两个可能角色"的情况：

- PA0 在 STM32F103 上的复用能力是 `TIM2_CH1` 和 `TIM2_ETR`，原理图上的网络名是 `PWMA`，两者对上
- 原理图上 **PB0 是 `SENSOR1`**（角度传感器），而 PB0 恰好也能做 `TIM3_CH3`。
  项目早期的手稿曾建议"PB0 留给电机 PWM"，**那条是错的** —— 查设备 XML 后确认 PB0 在这个套件上已经分给了角度传感器

> 教训：引脚的角色不能靠"这个脚能干这个活"来推，
> 要看**原理图上它实际接到了什么**。这里两件事都对上了才算数。

### 1.2 该放哪一层

本仓库的分层是 `应用层 → bsp → driver/device → driver/bus → HAL`。

电机驱动模块放哪一层？它其实**一半是 BSP、一半是器件驱动**：

- TB6612 是一颗有明确接口的**器件**（有真值表、有时序）
- 但"PWMA 接在 PA0、AIN1 接在 PB12"是**这块板子的接线**

本工程把它整体放在 `bsp/motor/`，理由是：

| 理由 | 说明 |
| --- | --- |
| 只有一根 PWM、两个 GPIO | 拆成 `driver/device/tb6612/` + `bsp/motor/` 两层，接口开销大于收益（对比 OLED：那边三层是有意义的，因为 SSD1306 有 1 KB 显存、有字库、有复杂命令集） |
| 没有抽象总线的必要 | OLED 需要 `i2c_if_t` 是因为"换 MCU 只换总线实现"；电机这边换平台要改的是引脚，本来就是 BSP 的职责 |
| 以后真需要再说 | 出现第二块驱动板、或要支持多种驱动芯片时，再抽器件层不迟 |

→ 放 `bsp/motor/`。

---

## 二、电机驱动基础：三个必须先搞清的概念

### 2.1 有刷直流电机怎么调速

直流电机的转速近似正比于**电枢两端的平均电压**。要连续调这个电压，有两种办法：

| 办法 | 缺点 |
| --- | --- |
| 串联可变电阻 | 效率极低，电阻上白白发热 |
| **PWM 斩波** | 无（开关管工作在饱和/截止，几乎不发热） |

PWM 斩波的思路：把电源以固定频率在"接通/断开"之间切换，**平均电压 = 电源电压 × 占空比**。
电机绕组的电感会把方波平滑掉，转子上感受到的就是那个平均电压。

```
占空比 25%：  ┌─┐   ┌─┐   ┌─┐
              │ │   │ │   │ │
        ──────┘ └───┘ └───┘ └────     平均 ≈ 3 V（12 V 电源）
              |←1个周期→|

占空比 100%： ┌──────────────────     平均 = 12 V
        ──────┘
```

本模块的 `duty` 就是这个占空比的百分数。

### 2.2 H 桥与 TB6612 的四态真值表

**为什么要 H 桥**：电机要正反转，就得能**反转电流方向**。H 桥用 4 个开关组成"桥"形，
对角导通是两个方向，同侧导通是刹车，全开是滑行。

TB6612 把两路 H 桥、驱动电路、保护电路集成在一起，对外只暴露 3 个控制脚：

```
        ┌──────────── TB6612（A 路）────────────┐
PWMA ──►│ PWM                                   │
AIN1 ──►│ 方向逻辑        ┌────────┐            │──► A01/A02 ──► 电机
AIN2 ──►│                 │  H 桥  │            │
        └─────────────────┴────────┘────────────┘
```

真值表（**本模块的全部行为都来自这张表**）：

| AIN1 | AIN2 | PWMA | 输出 | 说明 |
| :---: | :---: | :---: | --- | --- |
| 0 | 0 | × | **Stop** | 输出高阻，电机两端悬空 → **惯性滑行** |
| 1 | 1 | × | **Short brake** | 电机两端被短接 → **电磁刹车**，停得快 |
| 0 | 1 | PWM | 正转 | 转速 ∝ 占空比 |
| 1 | 0 | PWM | 反转 | 转速 ∝ 占空比 |

**这张表里最关键的一行是第一行和第四行的区别**：
`AIN1=AIN2=0` 与 `AIN1=0, AIN2=1, 占空比=0` 在"电机不转"这点上一样，
但**电机的电气状态完全不同**——前者两端悬空（可以自由转动），后者是被驱动的 0 V。
摆杆需要自由摆动时必须用前者，这就是本模块要区分 `coast()` 和 `set_duty(0)` 的原因（见 4.4）。

### 2.3 为什么 PWM 频率选 20 kHz

PWM 频率不能随便选，有上下两个约束：

| 方向 | 约束 | 选了会怎样 |
| --- | --- | --- |
| 太低（< 1 kHz） | 进入音频范围，电机绕组会**发出啸叫** | 「嗞——」的持续噪声 |
| 太低 | 每个周期内电流脉动大，**低速时转矩不平滑** | 电机会一顿一顿地转 |
| 太高（> 50 kHz） | H 桥的开关损耗上升 | 驱动芯片发热 |
| 太高 | 死区时间占比变大，**占空比分辨率变差** | 小占空比时控制不线性 |

**20 kHz 刚好在人耳上限之上**，是电机驱动的经典取值。本工程由 TIM2 产生：

```
TIM2 挂在 APB1 上。APB1 预分频 = 2，而 STM32 的规则是
「APB 预分频大于 1 时，定时器时钟 = 2 × PCLKx」：

    72 000 000 Hz ÷ (PSC+1) ÷ (ARR+1)
  = 72 000 000    ÷   36    ÷   100     = 20 000 Hz = 20 kHz

  → PSC = 35（写 36-1）   ARR = 99（写 100-1）
```

> **ARR 为什么是 100-1**：计数器从 0 数到 ARR，数 `ARR+1` 个数。
> 少减这个 1 是最常见的低级错误（和 [bsp_tick.c](../bsp/tick/bsp_tick.c) 里
> TIM1 的 `Period = 999` 是同一条）。

**顺带一个好处**：ARR = 99 时，占空比 = `CCR / (ARR+1)` = `CCR / 100`。
也就是说 **CCR 的数值直接就是百分比**，`duty = 50` 对应 50% 占空比，
不需要任何换算系数。这是本模块接口设计得这么简洁的前提。

---

## 三、CubeMX 配了什么

### 3.1 命令行配方

用 `stm32-cubemx` skill 的 CLI，**一次只加一个外设**（PWM 在那个 skill 里属于
"还没验证过的机制"，所以走增量推进：先只激活 + 配一两个参数，roundtrip 过了再加其余的）。

**第一步配方**（`mx.scratch/recipe_tim2_a.txt`）：

```text
# TIM2 CH1 PWM —— 第一步：只激活 + 时基参数
set mode TIM2 "PWM Generation CH1"
set pin PA0-WKUP TIM2_CH1
set ip parameters TIM2 Prescaler 35
set ip parameters TIM2 Period 99
```

**第二步配方**：加入 PB12/PB13（`mx.scratch/recipe_tim3_c.txt` 的后半段）：

```text
set pin PB12 GPIO_Output
set gpio parameters PB12 GPIO_PuPd GPIO_NOPULL
set gpio parameters PB12 GPIO_Label MOTOR_AIN1
set gpio parameters PB12 GPIO_ModeDefaultOutputPP GPIO_MODE_OUTPUT_PP
set gpio parameters PB12 GPIO_Speed GPIO_SPEED_FREQ_LOW

set pin PB13 GPIO_Output
set gpio parameters PB13 GPIO_PuPd GPIO_NOPULL
set gpio parameters PB13 GPIO_Label MOTOR_AIN2
set gpio parameters PB13 GPIO_ModeDefaultOutputPP GPIO_MODE_OUTPUT_PP
set gpio parameters PB13 GPIO_Speed GPIO_SPEED_FREQ_LOW
```

有三处值得记的细节：

**① 引脚名要写 `PA0-WKUP` 而不是 `PA0`。**
STM32F103 上 PA0 的规范名带后缀（它是唤醒引脚），类似地 `PC13` 写作
`PC13-TAMPER-RTC`、`PB2` 写作 `PB2-BOOT1`。名字写错 CubeMX 静默返回 KO。

**② 输出模式的参数名是 `GPIO_ModeDefaultOutputPP`，不是 `GPIO_Mode`。**
这个要从工程里已有的、CubeMX 自己生成的键去对照（`PB8` 那条就是模板），
不能猜。

**③ `Pulse`（初始占空比）不用显式设。** 默认就是 0，CubeMX 对等于默认值的键不落盘
——这是正常现象，别误判成"参数没生效"。

### 3.2 生成出来的代码

**`MX_TIM2_Init()`**（[Core/Src/main.c](../Core/Src/main.c)）：

```c
htim2.Instance = TIM2;
htim2.Init.Prescaler = 35;
htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
htim2.Init.Period = 99;
htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) { Error_Handler(); }

sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

sConfigOC.OCMode = TIM_OCMODE_PWM1;
sConfigOC.Pulse = 0;
sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { ... }

HAL_TIM_MspPostInit(&htim2);      /* ← 引脚配置在这里面 */
```

**注意最后一行**：F1 的 HAL 把引脚配置放在 `HAL_TIM_MspPostInit()` 里而不是
`HAL_TIM_PWM_MspInit()`。原因是 PWM 引脚必须在**通道配置之后**才能配成复用，
否则复用功能可能会在配置过程中被清零。这是 F1 HAL 特有的分工，别的系列不一定一样。

**`HAL_TIM_MspPostInit()`**（[Core/Src/stm32f1xx_hal_msp.c](../Core/Src/stm32f1xx_hal_msp.c)）：

```c
if(htim->Instance==TIM2)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /**TIM2 GPIO Configuration
  PA0-WKUP     ------> TIM2_CH1
  */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;      /* 复用推挽 —— 交给定时器控制 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
```

**`MX_GPIO_Init()` 里的方向脚**：

```c
/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOB, MOTOR_AIN1_Pin|MOTOR_AIN2_Pin|..., GPIO_PIN_RESET);
                                    /* ↑ 上电先把它们拉低 = 滑行，不是刹车 */

/*Configure GPIO pins : MOTOR_AIN1_Pin MOTOR_AIN2_Pin */
GPIO_InitStruct.Pin = MOTOR_AIN1_Pin|MOTOR_AIN2_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
```

**CubeMX 自动加的那句初始电平写 0，是本模块"上电不转"的第一道保险**（第二道在 `bsp_motor_init()` 里，见 4.3）。

### 3.3 一处 CubeMX 不会做的：CMake 源文件清单

`cmake/stm32cubemx/CMakeLists.txt` 是 CubeMX 生成的 **HAL 源文件清单**。
本工程顶层 [CMakeLists.txt](../CMakeLists.txt) 是**手写的显式列表**（没有用 `file(GLOB)`），
所以新增 `bsp/motor/bsp_motor.c` 时要在**两个地方各加一行**：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ...
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/motor/bsp_motor.c      # ← 加这行
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ...
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/motor                  # ← 和这行
)
```

好消息是**这一次不需要动 `cmake/stm32cubemx/` 那份**：`stm32f1xx_hal_tim.c` 和
`stm32f1xx_hal_tim_ex.c` 在配 TIM1 节拍的时候就已经在清单里了。
（这份清单 CubeMX 会不会自动更新，README 与项目记事的说法不一致，所以每次
重新生成后还是 `git diff` 看一眼最稳。）

### 3.4 踩过的坑

**① roundtrip 报了一个"白名单缺口"，但它是误报**

`roundtrip` 会主动扫描 CubeMX 的两个逗号分隔白名单（`IPParameters` / `GPIOParameters`），
把"写了但没列进白名单"的键点名出来。加了 TIM2 之后它报了：

```
!! 这些键没被列进 IPParameters 白名单，CubeMX 会完全无视它们:
     [TIM2] 缺: Channel-PWM\ Generation1\ CH1
```

看起来像个真问题，但**去 `.ioc` 里一看，白名单条目其实在**：

```ini
TIM2.Channel-PWM\ Generation1\ CH1=TIM_CHANNEL_1        # 键：空格被转义了
TIM2.IPParameters=Channel-PWM Generation1 CH1,Prescaler,Period   # 白名单：没转义
```

`.ioc` 的**键名**里空格要转义成 `\ `，而**值**（白名单列表）里的空格不用转义。
`roundtrip` 的检查脚本拿"还带反斜杠的键名"去比"不带反斜杠的白名单条目"，
凡是键名含空格的（正是所有 TIM 通道键）都会误报。

**怎么确认它是误报的**：不看 roundtrip，去看**生成的代码**。
`MX_TIM2_Init()` 里 `HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)` 实实在在出现了，
说明通道配置**确实生效了**。这就是"验证手段本身也会有假阳性"的又一例。

**② 引脚复用方向和"这个脚能干这个活"无关**

见 1.1 最后那段。PB0 既能做 `TIM3_CH3` 又能做 `ADC1_IN8`，
但套件上它接的是角度传感器。**决定角色的是原理图，不是芯片能力表。**

---

## 四、手写的部分

### 4.1 先定接口

```c
error_t bsp_motor_init(void);
error_t bsp_motor_set_duty(int16_t duty);   /* -100 ~ +100 */
error_t bsp_motor_coast(void);
error_t bsp_motor_brake(void);
int16_t bsp_motor_duty(void);
```

四条设计决定，每条都有理由：

| 决定 | 理由 |
| --- | --- |
| 用 `int16_t` 而不是 `uint8_t` + 单独的 dir 引脚 | **把方向和速度合成一个数**，控制环输出直接就是它，不用先判断符号再分别设两个参数。PID 的输出天然是带符号的 |
| 量纲是「百分点」而不是原始 CCR | 因为 ARR=99，两者数值恰好相同（见 2.3），所以取更有物理意义的那个 |
| 有 `coast()` 和 `brake()` 两个停机接口 | 两者电气状态完全不同（见 2.2），混用会出问题 |
| `bsp_motor_duty()` 返回**夹紧后**的值 | 显示和诊断需要知道"电机实际收到的命令"，而不是"应用想要的" |

### 4.2 为什么 duty 直接就是 CCR

```c
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint16_t)duty);
```

这行看起来太简单了，值得解释为什么**不需要换算**：

- 占空比 = `CCR / (ARR+1)`
- 本工程 `ARR = 99`，所以 `ARR+1 = 100`
- 于是占空比 = `CCR / 100` = **CCR 的百分数表示**

**这是一个刻意选的巧合**，不是巧合——如果 ARR 取 999，`duty` 到 CCR 就要乘 10；
如果取任意值 `N`，就得 `CCR = duty * (N+1) / 100`，多一次乘除、多一处取整误差、多一个可能写错的地方。

代价是 ARR 被绑死在 99。所以头文件里专门写了：

```c
#define BSP_MOTOR_PWM_ARR       (BSP_MOTOR_DUTY_MAX - 1)
```

这是**和 `.ioc` 里 TIM2 的 ARR 绑死**的常量（做法与 `BSP_TICK_PERIOD_MS` 绑 TIM1 的 PSC/ARR 一致）。
改 `.ioc` 里的 ARR 就必须同步改这里，否则占空比会**静默**算错一个倍数。

> `__HAL_TIM_SET_COMPARE()` 是 HAL 的宏，直接写 `CCR1` 寄存器，**不加锁、不校验**，
> 可以在中断里调用。因为 CubeMX 生成的 `HAL_TIM_PWM_ConfigChannel()` 会把 `OC1PE`
> （预装载使能）置上，所以新值会在**下一个更新事件**生效——20 kHz 下就是 50 µs，
> 对电机完全无所谓。

### 4.3 初始化顺序：为什么方向脚一定要在 Start 之前写

```c
error_t bsp_motor_init(void)
{
    s_ready = false;
    s_duty  = 0;

    motor_set_dir(GPIO_PIN_RESET, GPIO_PIN_RESET);  /* ① 先滑行 */
    motor_set_ccr(0U);                              /* ② 占空比也归零 */

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) {   /* ③ 最后才启动 */
        return ERR_NOT_READY;
    }

    s_ready = true;
    return ERR_OK;
}
```

**顺序不能调换**，理由是倒立摆的物理特性：

- 如果先 `Start` 再写占空比，那么在 `Start` 到"写 0"之间的这一小段时间里，
  CCR 里是**复位值**（这里是 0，所以碰巧安全）。
- 但 `.ioc` 里的 `Pulse` 是**可以改的**。一旦有人把它改成非 0（比如调试时想省一步），
  反序就会让电机在初始化中途**窜一下**——摆杆装在上面的时候，这一窜可能就是撞坏零件。

**"先写状态、再使能输出"是电机驱动的通例**，不依赖"复位值恰好是 0"这种运气。

另外两个细节：

- **`HAL_TIM_PWM_Start()` 必须显式调用，而且只能调一次。**
  `MX_TIM2_Init()` 里只做了 `HAL_TIM_PWM_Init()` + `ConfigChannel()`，
  它们**只把参数写进寄存器**——既没有使能输出比较（`CC1E`），也没有使能计数器（`CEN`）。
  不调 `Start` 的现象是"编译通过、烧录成功、电机纹丝不动"。
  而重复调用会因为通道状态已经是 `BUSY` 而返回 `HAL_ERROR`（它不只是重写寄存器）。
- **`s_ready` 最后才置位**：中间任何一步失败就 return，`s_ready` 保持 false，
  之后 `set_duty` 会拒绝执行。这样不会留下一个"看起来能用、实际没启动"的状态。

### 4.4 两个「停」不是一回事

```c
error_t bsp_motor_coast(void)   /* AIN1=AIN2=0 → 输出高阻，惯性滑行 */
error_t bsp_motor_brake(void)   /* AIN1=AIN2=1 → 绕组短接，电磁刹车 */
```

这是本模块最"反直觉"的一处设计——**为什么不能只有一个 `stop()`**。

回到 2.2 的真值表：`AIN1=AIN2=0` 时 H 桥的四个开关全断，电机两端**悬空**，
转子可以自由转动（只受轴承摩擦）。而 `AIN1=AIN2=1` 时电机两端被**短接**，
转动的反电动势会在绕组里产生短路电流，形成与转向相反的力矩——这就是电磁刹车。

| | 电机状态 | 适用场合 |
| --- | --- | --- |
| `coast()` | 自由转动 | 正常停机、急停断电、**PID 停机**（要让摆杆自由摆动） |
| `brake()` | 阻力很大 | 需要快速停转、需要位置锁定 |

**参考实现里 `Motor_SetPWM(0)` 走的是"正转 0% 占空比"那条分支**
（`AIN1=0, AIN2=1, CCR=0`），严格来说既不是 coast 也不是 brake，
而是"用 0 V 驱动电机"。本模块把它明确拆成了两个语义清晰的接口。

> **调试用途**：`coast` 和 `brake` 也是排查"电机转不动"的好工具——
> 用手扭输出轴，`coast` 时应该很轻松，`brake` 时应该明显有阻力。
> 这两只手感不同就说明方向脚真的在起作用（见 5.4）。

### 4.5 越界为什么夹紧而不是报错

```c
if (duty > (int16_t)BSP_MOTOR_DUTY_MAX) {
    duty = (int16_t)BSP_MOTOR_DUTY_MAX;
} else if (duty < -(int16_t)BSP_MOTOR_DUTY_MAX) {
    duty = -(int16_t)BSP_MOTOR_DUTY_MAX;
}
```

本工程的约定是"做事函数返回 `error_t`，取值器返回数值"。
那这里遇到越界，为什么**不返回 `ERR_INVALID_PARAM`**？

因为**调用它的是 PID 控制环**，而控制环的输出越界是**正常现象**：

- PID 的限幅（`OutMax`/`OutMin`）是在它内部做的，但**限幅之前**的瞬态可能越界
- 参数没调好时，输出冲出去是常态
- 这时让电机**停摆**比夹紧到边界**危险得多**——倒立摆失去控制力矩会直接倒

所以这里的取舍是：**安全性交给夹紧，不返回错误**。
需要严格边界检查的场合，调用者自己判断。

这个取舍**必须写在头文件里**（本模块确实写了），否则调用者会以为越界会报错，
从而漏掉自己的检查。

---

## 五、怎么测试

### 5.1 为什么「编译通过 + 烧录成功」不算验证

这三件事各自独立、而且**失败时的表现都是"一切正常、电机不动"**：

| 检查 | 真正证明了什么 | **没有**证明什么 |
| --- | --- | --- |
| `cmake --build` 通过 | 语法、符号没问题 | 生成的逻辑是你要的 |
| `MX_TIM2_Init()` 里有 `HAL_TIM_PWM_ConfigChannel` | 通道参数写进了句柄 | PWM 有没有真的输出 |
| 烧录返回 "Download complete" | Flash 写进去了 | 程序跑起来了 |
| 电机转了 | 某条路径通了 | 它走的是**你设计的**路径 |

所以下面按"证据强度"分四层。

### 5.2 第一层：去生成的代码里找证据

```bash
grep -n "htim2.Init.Prescaler\|htim2.Init.Period\|HAL_TIM_PWM_ConfigChannel" Core/Src/main.c
grep -n "TIM2 GPIO Configuration" -A 6 Core/Src/stm32f1xx_hal_msp.c
```

要看到：

- `htim2.Init.Prescaler = 35;` / `htim2.Init.Period = 99;` —— 20 kHz 的两个因子
- `HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)` —— 通道确实配了
- MSP 里 `PA0-WKUP ------> TIM2_CH1` 且 `GPIO_MODE_AF_PP`

**这一步就足以排除"白名单缺口"这类静默失效**（见 3.4）。

### 5.3 第二层：读运行时变量（不需要人动手）

用 SWD **不复位**连接读 RAM，能拿到程序里的真值：

```bash
B="/c/Users/0isno/AppData/Local/stm32cube/bundles"
export PATH="$B/gnu-tools-for-stm32/14.3.1+st.2/bin:$PATH"

arm-none-eabi-nm build/Debug/PID_Pendulum.elf | grep -E "s_duty|s_duty_cmd|s_run"
# 20000856 b s_duty          ← 驱动层实际写的占空比
# 200005cc b s_duty_cmd      ← 应用层命令的占空比
# 200005ce b s_run           ← 应用层运行状态

CLI="$B/programmer/2.23.0/bin/STM32_Programmer_CLI.exe"
"$CLI" -c port=SWD mode=hotplug -r16 0x20000856 2
```

`mode=hotplug` 是**不复位**连接，读到的是正在运行的程序的实时值。

这里能验证一件**只在代码里看不出来的事**——"停机时电机真的被强制成 0"：

```
    时刻  angle  delta   total  duty   cmd run
   0.6   1168      0  -11397     0   -10 off     ← 命令行是 -10，实际占空比是 0
   1.2   1172      0  -11397     0   -10 off
```

`cmd = -10` 而 `duty = 0`、`run = off`：说明"未运行时强制滑行"这条逻辑生效了
（`app_apply_motor()` 里 `if (s_run) set_duty(...) else coast()`）。
这正是当初把 `s_duty_cmd`（命令值）和 `s_duty`（实际值）分开的目的。

> **踩过的坑（本机真实发生）**：一开始用 `sed` 从 CLI 输出里抓读数，
> 结果抓到的是 ST-LINK 序列号里的 `37FF`（读出来 14335，**超过 12 位 ADC 的量程上限**，
> 一眼就看出不对）。教训是解析外部工具输出时要**锚定在地址前缀上**，
> 而不是"取第一个十六进制串"。

### 5.4 第三层：方向一致性（最要紧的一条）

**这条必须在驱动阶段坐实，不能等调 PID 才发现。**

为什么：双环 PID 里角度环的输出直接喂给电机，而反馈来自编码器/角度传感器。
**如果"电机正转"对应"编码器负计数"，整个环就是正反馈**——
偏差不但不收敛，还会被不断放大，一给输出就飞。这个符号错了，
现象是"参数怎么调都发散"，而看起来像是参数问题。

**方法**：给一个固定正占空比，读编码器累计位置，看它增大还是减小。

```bash
# 操作：K2 ×3（命令行 20）→ K1 启动 → 等约 1 秒 → K1 停止
python mx.scratch/pend_watch.py 6
```

实测结果（`s_total` 单调递增）：

| 时刻 (s) | 时刻 | s_total | 增量 |
| --- | --- | --- | --- |
| 0.6 | | 685 | |
| 1.2 | | 1107 | +422 |
| 2.4 | | 1921 | |
| 3.6 | | 2712 | |
| 4.1 | | 3105 | **+393 / 0.5 s** |

`duty = +20` → `s_total` **增大** → **方向一致** ✅

再用负占空比测一遍**代码里另一条分支**（`duty < 0` 时的方向脚对调）：

| | 基线 | 测试后 | 变化 |
| --- | --- | --- | --- |
| `s_total` | 28235 | **26252** | **−1983** |
| `s_duty_cmd` | +20 | **−20** | |

`duty = −20` → `s_total` 减小 → **负分支也正确** ✅

> ⚠️ 这一步**必须两个方向都测**：正负分支走的是**不同的代码路径**
> （`motor_set_dir(AIN1, AIN2)` 与 `motor_set_dir(AIN2, AIN1)`），
> 只测一个方向，另一条分支可能是错的而你不知道。

### 5.5 第四层：用转速反推占空比映射

没有示波器也能间接验证"占空比 → 转速"是对的——**用编码器反推**：

```
实测：0.5 秒内 s_total 变化 393
      → 786 边沿/秒
      → 786 ÷ 408 边沿/圈 = 1.93 圈/秒
      → 116 RPM

理论：25GA370 空载 620 RPM × 20% 占空比 = 124 RPM
```

**116 对 124，约 94%**。偏差来自有刷电机的摩擦和内阻（空载转速本身就带负载），
量级完全合理。

这一条同时验证了三件事：

1. 占空比映射确实是 `CCR = duty`（若差 10 倍，转速会差 10 倍，一眼看出）
2. `bsp_encoder` 的 408 边沿/圈是对的
3. 20 kHz 的 PSC/ARR 配置是自洽的（若频率错了 100 倍，电机根本转不起来）

### 5.6 没验证到什么

**PWM 频率 20 kHz 本身没有实测**（本机没有示波器/逻辑分析仪）。依据是：

- 生成代码里 `PSC=35 / ARR=99` 正确
- 实测转速与理论相符（若频率错到影响电机，转速会明显偏离）

要坐实需要**测 PA0 引脚**（示波器看周期，或逻辑分析仪）。这是本模块唯一
"靠推导而非实测"的结论，写在这里以免日后被当成已验证的事实。

---

## 六、设计取舍一览

| 决定 | 换来什么 | 代价 |
| --- | --- | --- |
| 整个模块放 BSP，不拆器件层 | 接口简洁、不用为 3 个信号造抽象 | 换驱动芯片（如 DRV8833）时要改这个模块而不是新增一个 |
| ARR = 99，duty 直接当 CCR | 占空比数值 = 百分数，无换算、无取整误差 | ARR 被绑死在 `.ioc` 与宏之间，改一处要改两处 |
| 20 kHz | 无啸叫、低速转矩平滑 | 比 1 kHz 稍多的开关损耗（对本应用无所谓） |
| 越界夹紧不报错 | 控制环瞬态越界时电机不停摆 | 调用者拿不到"参数错了"的信号，必须自己保证范围 |
| 区分 coast / brake | 两种停机语义各自明确 | 多一个接口，使用者要理解区别 |
| `init()` 最后才 `Start` | 上电绝不带非零占空比 | 多两行、且顺序不能调换（要写注释说明） |
| 方向脚交给 CubeMX 管 | 引脚冲突在配置阶段就被发现 | 改方向脚要重新生成代码，不能在代码里改 |

整个模块约 170 行，其中真正的"逻辑"只有 `set_duty` 里那十行分支，
其余都是**为安全付出的成本**——初始状态、夹紧、两种停机、`s_ready` 守卫。

---

## 附：这次踩过的坑

按"踩到的顺序"记录，都是实测：

| 坑 | 现象 | 根因 |
| --- | --- | --- |
| **把 PB0 当成电机 PWM 脚** | 方案里写"PB0 留给 TIM3_CH3 做 PWM" | 查设备 XML 才发现 PB0 在这个套件上接的是角度传感器（SENSOR1）。**引脚角色看原理图，不看芯片能力表** |
| **roundtrip 报白名单缺口** | `[TIM2] 缺: Channel-PWM\ Generation1\ CH1` | 检查脚本拿转义过的键名比未转义的白名单 → **误报**。用生成的代码证伪 |
| **引脚名写 `PA0`** | `set pin` 静默返回 KO | F1 上规范名是 `PA0-WKUP`。同类还有 `PC13-TAMPER-RTC`、`PB2-BOOT1` |
| **解析 CLI 输出抓到序列号** | 读出 14335（**超过 12 位 ADC 上限 4095**） | `sed` 取"第一个十六进制串"命中了 `ST-LINK SN: 37FF...`。要锚定地址前缀 |
| **忘了 `HAL_TIM_PWM_Start`** | 编译烧录全成功，电机不动 | `MX_TIM2_Init()` 只写寄存器，不使能输出比较和计数器 |
| **以为 `duty = 0` 就是停** | 电机两端不是悬空而是被 0 V 驱动 | TB6612 的四种状态里，"正转 0%" ≠ "高阻"。要有独立的 `coast()` |

**倒数第二条最值得记**：这套工具链最典型的失败模式就是
**"所有命令返回成功、硬件毫无反应"**。`HAL_TIM_PWM_Start` 漏掉时，
CubeMX 配置、roundtrip、编译、烧录**全部通过**——只有电机不动这一条能暴露它。
