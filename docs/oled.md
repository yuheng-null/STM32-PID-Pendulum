# OLED（SSD1306）驱动实现详解

本文逐层拆解本仓库 0.96 寸 OLED 模块的实现，包括设计取舍、协议原理和每一段代码在做什么。

**面向读者**：嵌入式初学者。假设你会 C（结构体、位运算、函数指针），但不要求懂 I2C 协议、不要求懂 STM32 的 GPIO 细节。

**配套代码**：

| 文件 | 层 | 职责 |
| --- | --- | --- |
| [driver/common/error.h](../driver/common/error.h) | 公共 | 全工程统一的 `error_t` |
| [driver/bus/i2c/i2c_if.h](../driver/bus/i2c/i2c_if.h) | 接口 | 抽象 I2C 总线契约 |
| [driver/bus/i2c/i2c_soft.h](../driver/bus/i2c/i2c_soft.h) · [.c](../driver/bus/i2c/i2c_soft.c) | 总线驱动 | 位翻转 I2C 实现 |
| [driver/device/ssd1306/](../driver/device/ssd1306/) | 器件驱动 | SSD1306 命令、显存、文本与图形 |
| [bsp/oled/](../bsp/oled/) | BSP | 本板接线与组装 |
| [Core/Src/main.c](../Core/Src/main.c) | 应用 | 测试代码 |

---

## 目录

- [一、全局图景](#一全局图景)
- [二、I2C 协议基础](#二i2c-协议基础)
- [三、总线层：i2c_soft](#三总线层i2c_soft)
- [四、器件层：ssd1306](#四器件层ssd1306)
- [五、BSP 层：把零件装起来](#五bsp-层把零件装起来)
- [六、测试代码](#六测试代码)
- [七、如何验证](#七如何验证)
- [八、容易踩的坑](#八容易踩的坑)

---

## 一、全局图景

### 1.1 这块屏是什么

0.96 寸 OLED 模块不是"一块屏幕"，而是三层东西叠在一起：

```
┌─────────────────────────────────────────┐
│  玻璃面板 + 驱动电路                      │  ← 看到的黑色那块
├─────────────────────────────────────────┤
│  SSD1306 控制器芯片                       │  ← 真正要驱动的东西
│   · 128×64 像素的显存（GDDRAM）           │
│   · 一堆命令寄存器                        │
│   · I2C / SPI 接口                        │
├─────────────────────────────────────────┤
│  引出 4 个针脚：VCC GND SCL SDA           │
└─────────────────────────────────────────┘
```

关键点：**代码不是在"点亮像素"，而是在通过 I2C 给 SSD1306 发命令和显存数据**。SSD1306 收到数据后自己负责扫描、点亮。这是所有"智能外设"的通用套路——MCU 只负责通信，具体活儿交给外设芯片。

### 1.2 一次刷新，数据是怎么走的

从 `main()` 到玻璃上亮起的像素，要穿过整整四层：

```
main.c
  oled_update_counter(159)
    │  调用（不知道底下是什么屏）
    ↓
ssd1306.c        器件驱动
  ssd1306_write_string(&s_oled, "loop: 159", &Font_6x8, WHITE)
    │  把 "loop: 159" 这 9 个字符按字模展开成像素，
    │  写进 s_oled.buffer[] 这块 1024 字节的本地显存
    │
  ssd1306_update_screen(&s_oled)
    │  把 buffer 分成 8 页，每页拼一个 I2C 帧
    │  调用（只认 i2c_if_t 这个接口，不认识 GPIO）
    ↓
i2c_soft.c       总线驱动
  dev->bus->write(ctx, 0x3C, frame, 129)
    │  真正的协议动作：起始条件 → 地址 → 数据 → 应答 → 停止条件
    │  每个 bit 靠翻转 PB8/PB9 电平实现
    ↓
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, SET/RESET)
    │  写寄存器 BSRR/BRR，改变引脚电压
    ↓
PB8 / PB9 两根线 ──I2C 电平信号──► SSD1306 ──► 像素亮/灭
```

### 1.3 为什么要分层

因为每层关心的东西完全不同，混在一起会互相拖累：

| 层 | 关心什么 | 不关心什么 |
|---|---|---|
| main.c | 我要显示什么内容 | 屏是什么型号、接在哪 |
| ssd1306.c | SSD1306 的命令、显存布局、字模怎么画 | 是软件 I2C 还是硬件 I2C、是 STM32 还是 GD32 |
| i2c_soft.c | 起止条件、时序、应答位 | 对面挂的是 OLED 还是温湿度传感器 |
| HAL | 寄存器怎么配 | 上面任何事 |

最实际的好处：

- **换 MCU** 只需重写 `i2c_soft.c`（换成另一家的 GPIO 操作），`ssd1306.c` 一行不用改
- **要在 PC 上做单元测试**，写个假的 `i2c_if_t` 把数据收进数组里即可，不需要硬件

### 1.4 为什么用软件模拟 I2C

| 理由 | 说明 |
| --- | --- |
| 引脚 | OLED 接在 PB8/PB9，而 STM32F103 的 I2C1 默认在 PB6/PB7，用硬件外设还得额外开 AFIO 重映射 |
| 可靠性 | STM32F103 的 I2C1 外设有已知 errata，总线异常时容易卡死，常常要整芯片复位才能恢复 |
| 学习价值 | 位翻转把起止条件、应答位、时钟同步摆在明面上，而不是藏在外设寄存器里 |

总线速度约 300 kHz，刷新一屏（1024 字节）约 30 ms，对显示用途完全够用。

软件 I2C 直接接管 PB8/PB9 的 GPIO 配置（开漏 + 上拉），**没有**在 CubeMX 里把这两个引脚配成 `GPIO_Output` —— 位翻转总线的引脚时序属于总线实现的一部分，由总线驱动自己配置更内聚，也避免 CubeMX 重新生成代码时把模式改回推挽。

---

## 二、I2C 协议基础

### 2.1 物理层：为什么必须是"开漏 + 上拉"

I2C 只用两根线：

- **SCL**（Serial Clock）—— 时钟，主机产生
- **SDA**（Serial Data）—— 数据，双向

这两根线上可能挂着多个器件。**这带来一个问题**：假设用普通推挽输出，器件 A 想输出高（接到 VCC），器件 B 想输出低（接到 GND）：

```
     VCC
      │
      ├──── A 输出高 ─┐
      │               │  ← 直接短路！电流从 VCC 灌到 GND
      └──── B 输出低 ─┘        轻则发热，重则烧引脚
      │
     GND
```

**所以 I2C 规定：任何器件都只能"拉低"，不能主动"拉高"。** 这就是**开漏输出**（Open-Drain）。高电平靠**上拉电阻**提供：

```
        VCC (3.3V)
          │
        ┌─┴─┐  上拉电阻
        │   │
        └─┬─┘
          │
          ├────────── 到 SSD1306
          │
        ┌─┴─┐
        │   │  器件内部：开漏 MOS 管
        └─┬─┘
          │
         GND
```

- 器件**不导通** → 线被上拉电阻拉高 → 读到 **1**
- 器件**导通**（拉低）→ 线被拽到 GND → 读到 **0**

这就形成了 **"线与"** 特性：**只要有一个器件拉低，整条线就是低；所有人都放手，线才是高。**

这个特性带来两个好处，正好是 I2C 的核心机制：

1. **应答（ACK）**：主机发完 8 个 bit 后松手，从机拉低 SDA 表示"我收到了"。这是"线与"天然实现的，不需要任何额外的线。
2. **时钟同步 / 仲裁**：多个主机同时说话时，谁拉低谁说了算，自动分出胜负。

代码对应 [i2c_soft.c](../driver/bus/i2c/i2c_soft.c) 的 `i2c_soft_gpio_config()`：

```c
gpio.Mode  = GPIO_MODE_OUTPUT_OD;   /* 开漏输出 */
gpio.Pull  = GPIO_PULLUP;           /* 使能内部上拉 */
```

> **实操细节**：STM32 内部上拉很弱（约 30–50 kΩ），对 400 kHz 来说上升沿会偏慢。不过 0.96 寸 OLED 模块板上一般自带 4.7 kΩ 外部上拉，并联后完全够用。如果高速下偶发不稳定，优先加外部上拉电阻，而不是降速。

### 2.2 协议层：五种动作

#### ① 起始条件（START）

**SCL 保持高电平时，SDA 由高变低。**

```
SCL  ────────┐
             └────────
SDA  ──┐
       └──────────────
       ↑ 这里
```

为什么这么定义？正常情况下 **SCL 高电平期间 SDA 必须保持稳定**（那是数据位）。所以"SCL 高时 SDA 变化"就成了一个绝不会出现在数据里的特殊信号，用来标记"一帧开始"。

#### ② 停止条件（STOP）

**SCL 保持高电平时，SDA 由低变高。**

```
SCL  ────────┐
             └────────
SDA  ────────┐
             └────────
             ↑ 这里
```

#### ③ 发送一个数据位

**SCL 低电平期间改变 SDA，SCL 高电平期间 SDA 必须稳定**（从机就是在 SCL 高电平这一刻采样）。

```
      ┌───┐   ┌───┐   ┌───┐
SCL   │   │   │   │   │   │
    ──┘   └───┘   └───┘   └──
        ↑       ↑
     在这里改  从机在这里采样
      SDA       （SDA 必须稳定）
```

#### ④ 应答（ACK / NACK）

每发送完 **8 个 bit**，必须跟 1 个应答位，凑成 9 个时钟：

- **主机发数据给从机** → 第 9 个时钟**主机松手**，从机拉低 SDA = **ACK**，不拉 = **NACK**
- **主机从从机读数据** → 第 9 个时钟**从机松手**，主机拉低 SDA = **ACK**（"我还要"），不拉 = **NACK**（"够了"）

#### ⑤ 一帧完整的样子

```
 START  地址(7bit) + R/W   ACK   数据(8bit)   ACK   ...   数据(8bit)   ACK   STOP
   │         │              │        │        │            │          │      │
   ▼         ▼              ▼        ▼        ▼            ▼          ▼      ▼
 ┌───┐ ┌─────────────┐ ┌───┐ ┌──────────┐ ┌───┐      ┌──────────┐ ┌───┐ ┌───┐
 │   │ │             │ │   │ │          │ │   │      │          │ │   │ │   │
└┘   └─┘             └─┘   └─┘          └─┘   └──────┘          └─┘   └─┘   └─
```

**地址字节的最后一位是读写位**：0 = 写，1 = 读。0.96 寸模块的 7 位地址是 `0x3C`，实际发出去的是：

- 写：`0x3C << 1 | 0` = **`0x78`**
- 读：`0x3C << 1 | 1` = **`0x79`**

> ⚠️ **新手最容易搞混的地方**：数据手册写 0x3C，但示波器上抓到的第一个字节是 0x78。

代码在 `i2c_soft_send_addr()`：

```c
const uint8_t frame = (uint8_t)((addr << 1) | (read ? 0x01U : 0x00U));
```

---

## 三、总线层：i2c_soft

### 3.1 数据结构

```c
typedef struct {
    GPIO_TypeDef *scl_port;   /* SCL 在哪个端口，这里是 GPIOB */
    uint16_t      scl_pin;    /* SCL 是哪个引脚，这里是 GPIO_PIN_8 */
    GPIO_TypeDef *sda_port;
    uint16_t      sda_pin;
    uint32_t      freq_hz;    /* 目标 SCL 频率 */
} i2c_soft_cfg_t;
```

端口和引脚**分开存**，因为 SCL 和 SDA 万一分在不同端口，代码不用改。

```c
typedef struct {
    i2c_soft_cfg_t cfg;       /* 引脚与频率 */
    i2c_if_t       iface;     /* 对外的抽象接口 */
    uint32_t       delay_us;  /* 半周期延时，由 freq_hz 算出 */
    bool           ready;     /* 初始化完成标志 */
} i2c_soft_t;
```

**为什么状态放在结构体里而不是全局变量？** 这样同一份代码可以支持多条总线。写成全局变量就不可能同时有两条软件 I2C。

### 3.2 延时：DWT 周期计数器

位翻转必须精确控制时间，否则时钟频率就是"看编译器心情"。这里用了 Cortex-M3 内核自带的 **DWT 周期计数器**——一个每过一个 CPU 周期加 1 的寄存器，精度 13.9 ns（72 MHz 下一个周期）。

```c
static void i2c_soft_timing_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  /* ① 打开调试/跟踪模块总开关 */
    DWT->CYCCNT = 0U;                                 /* ② 计数器清零 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              /* ③ 让计数器开始数 */

    /* ④ 自检：确认它真的在走 */
    const uint32_t start = DWT->CYCCNT;
    for (volatile uint32_t i = 0U; i < 64U; i++) { }
    s_dwt_available = (DWT->CYCCNT != start);
}
```

第 ④ 步的自检是必需的。DWT 在极少数情况下（比如某个调试器把 core debug 关了）**不会计数**。没有这个自检，延时函数会死等一个永远不动的计数器——**直接死锁**。这种"在某些电脑上能跑、在别人电脑上卡死"的 bug 最难查，宁可多写 5 行。

主延时函数：

```c
static void i2c_soft_delay(const i2c_soft_t *bus)
{
    if (s_dwt_available) {
        const uint32_t cycles_per_us = SystemCoreClock / I2C_SOFT_US_PER_SEC;
        const uint32_t wait_cycles    = bus->delay_us * cycles_per_us;
        const uint32_t start          = DWT->CYCCNT;

        while ((DWT->CYCCNT - start) < wait_cycles) { }
    } else {
        /* 退化路径：粗略空循环 */
        ...
    }
}
```

具体数字：`SystemCoreClock = 72 000 000` → `cycles_per_us = 72`；`delay_us = 1000000 / (400000 × 2) = 1`。所以每次等 **72 个周期 = 1 微秒**。

> **细节**：这里用 `DWT->CYCCNT - start` 的**减法**写法，而不是 `DWT->CYCCNT < start + wait`。原因是 CYCCNT 是 32 位，72 MHz 下约 60 秒就溢出回绕。减法在无符号运算里天然处理回绕（就像时钟从 23:59 到 00:01，差值仍是 2 分钟），而加法写法在回绕瞬间会死等。这是嵌入式延时的标准写法。

### 3.3 五个引脚原语

```c
static void i2c_soft_scl_high(const i2c_soft_t *bus) { HAL_GPIO_WritePin(bus->cfg.scl_port, bus->cfg.scl_pin, GPIO_PIN_SET); }
static void i2c_soft_scl_low (const i2c_soft_t *bus) { HAL_GPIO_WritePin(bus->cfg.scl_port, bus->cfg.scl_pin, GPIO_PIN_RESET); }
static void i2c_soft_sda_high(const i2c_soft_t *bus) { ...SET... }
static void i2c_soft_sda_low (const i2c_soft_t *bus) { ...RESET... }

static bool i2c_soft_sda_is_high(const i2c_soft_t *bus)
{
    return HAL_GPIO_ReadPin(bus->cfg.sda_port, bus->cfg.sda_pin) == GPIO_PIN_SET;
}
```

**这里藏着 STM32F1 的一个关键特性，也是整个软件 I2C 能这么简洁的原因**：

`i2c_soft_sda_is_high()` 读的是 `IDR`（输入数据寄存器），而这个引脚当时**配置的是输出模式**。在大多数 MCU 上，输出模式下读输入寄存器没有意义，得先切成输入模式。但 **STM32F1 的 GPIO 在输出模式下，输入通道依然保持有效**（RM0008 参考手册的 GPIO 章节明确写了）。

所以在 F1 上：**开漏输出 + 写 1 释放 + 直接读 IDR**，就能读出线路的真实电平，不需要来回切换模式。

> **移植注意**：换成 STM32F4 或其他厂商的芯片时，这里可能要改成"读之前切输入、读完切回输出"。

### 3.4 起始 / 停止

对照 [2.2 节](#22-协议层五种动作)的时序图看：

```c
static void i2c_soft_start(const i2c_soft_t *bus)
{
    i2c_soft_sda_high(bus);     /* ① 先确保 SDA 是高 */
    i2c_soft_scl_high(bus);     /* ② SCL 也拉高 */
    i2c_soft_delay(bus);

    i2c_soft_sda_low(bus);      /* ③ SCL 高时 SDA 由高变低 ← 起始条件 */
    i2c_soft_delay(bus);

    i2c_soft_scl_low(bus);      /* ④ SCL 拉低，准备发第一位 */
    i2c_soft_delay(bus);
}

static void i2c_soft_stop(const i2c_soft_t *bus)
{
    i2c_soft_sda_low(bus);      /* ① 先把 SDA 拉低 */
    i2c_soft_scl_high(bus);     /* ② SCL 拉高 */
    i2c_soft_delay(bus);

    i2c_soft_sda_high(bus);     /* ③ SCL 高时 SDA 由低变高 ← 停止条件 */
    i2c_soft_delay(bus);
}
```

**顺序不能反。** 如果先拉高 SCL 再改 SDA，那改 SDA 的那一刻 SCL 已经是高，会误产生起始/停止条件。所以必须先在 SCL 还是低的时候把 SDA 摆到位。

### 3.5 发送一个字节 + 读应答

这是全文件最核心的函数：

```c
static bool i2c_soft_write_byte(const i2c_soft_t *bus, uint8_t byte)
{
    /* ── 前 8 个时钟：逐位发送 ── */
    for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1) {
        if ((byte & mask) != 0U) {
            i2c_soft_sda_high(bus);
        } else {
            i2c_soft_sda_low(bus);
        }
        i2c_soft_delay(bus);

        i2c_soft_scl_high(bus);     /* SCL 拉高，从机此刻采样 */
        i2c_soft_delay(bus);

        i2c_soft_scl_low(bus);
        i2c_soft_delay(bus);
    }

    /* ── 第 9 个时钟：读从机应答 ── */
    i2c_soft_sda_high(bus);         /* 主机"松手"，SDA 交给从机 */
    i2c_soft_delay(bus);

    i2c_soft_scl_high(bus);
    i2c_soft_delay(bus);

    const bool acked = !i2c_soft_sda_is_high(bus);   /* 从机拉低 = ACK */

    i2c_soft_scl_low(bus);
    i2c_soft_delay(bus);

    return acked;
}
```

**逐位拆解 `mask` 循环**（假设 `byte = 0xA5 = 1010 0101`）：

| 轮次 | mask | `byte & mask` | SDA 输出 | 说明 |
|---|---|---|---|---|
| 1 | `1000 0000` | 非0 | 高 | 最高位 **1** |
| 2 | `0100 0000` | 0 | 低 | 第2位 **0** |
| 3 | `0010 0000` | 非0 | 高 | 第3位 **1** |
| 4 | `0001 0000` | 0 | 低 | 第4位 **0** |
| 5 | `0000 1000` | 0 | 低 | 第5位 **0** |
| 6 | `0000 0100` | 非0 | 高 | 第6位 **1** |
| 7 | `0000 0010` | 0 | 低 | 第7位 **0** |
| 8 | `0000 0001` | 非0 | 高 | 最低位 **1** |

**I2C 是 MSB 优先**（高位先发），所以 `mask` 从 `0x80` 开始右移。

**第 9 个时钟的应答读取，是理解"开漏 + 线与"的最佳例子**：

- 主机执行 `i2c_soft_sda_high()` —— 在开漏模式下，这不是"输出高电平"，而是**"松手"**，让上拉电阻把线拉高
- 此时如果从机想应答，它就**拉低 SDA**
- 主机在 SCL 高电平时刻读 IDR：
  - 读到**低** → 从机拉低了 → `acked = true`（**应答**）
  - 读到**高** → 没人动 → `acked = false`（**无应答**）

所以 `acked = !i2c_soft_sda_is_high(bus)` 这个取反，含义就是"读到低才算应答"。

**这个返回值非常有用**——`i2c_soft_if_write()` 就是靠它判断器件在不在：

```c
if (!i2c_soft_send_addr(bus, addr, false)) {
    i2c_soft_stop(bus);
    return ERR_COMMUNICATION;      /* 没人应答，直接返回错误 */
}
```

### 3.6 接收一个字节

```c
static uint8_t i2c_soft_read_byte(const i2c_soft_t *bus, bool ack)
{
    uint8_t byte = 0U;

    i2c_soft_sda_high(bus);      /* 松手，SDA 由从机驱动 */

    for (uint8_t i = 0U; i < 8U; i++) {
        byte <<= 1;                          /* 左移腾出最低位 */

        i2c_soft_scl_high(bus);
        i2c_soft_delay(bus);

        if (i2c_soft_sda_is_high(bus)) {     /* SCL 高电平期间采样 */
            byte |= 0x01U;
        }

        i2c_soft_scl_low(bus);
        i2c_soft_delay(bus);
    }

    /* 第 9 个时钟：主机回应答 */
    if (ack) {
        i2c_soft_sda_low(bus);    /* ACK：拉低 = "我还要" */
    } else {
        i2c_soft_sda_high(bus);   /* NACK：松手 = "够了" */
    }
    ...
    return byte;
}
```

注意移位方向相反：**发送是右移取掩码，接收是左移拼接**。发的时候从高位开始取；收的时候高位先到，需要不断往左推。

调用方在最后一个字节回 NACK：

```c
for (uint32_t i = 0U; i < len; i++) {
    /* 最后一个字节回 NACK，告诉从机「我不要了」 */
    data[i] = i2c_soft_read_byte(bus, (i + 1U) < len);
}
```

**为什么最后一个字节要 NACK？** 从机收到 ACK 后会继续准备下一个字节。主机回 NACK 是标准的"我读完了"信号，之后主机才能合法地发停止条件。

### 3.7 包装成抽象接口

协议动作写完后，包装成 [i2c_if.h](../driver/bus/i2c/i2c_if.h) 定义的三个函数：

```c
typedef struct {
    error_t (*write)(void *ctx, uint8_t addr, const uint8_t *data, uint32_t len);
    error_t (*read)(void *ctx, uint8_t addr, uint8_t *data, uint32_t len);
    error_t (*write_read)(void *ctx, uint8_t addr,
                          const uint8_t *cmd, uint32_t cmd_len,
                          uint8_t *rx, uint32_t rx_len);
    void *ctx;   /* 具体总线实例 */
} i2c_if_t;
```

**为什么需要 `void *ctx`？** 函数指针本身是"无状态"的——`i2c_soft_if_write` 只有一份，如果系统里有两条软件 I2C 总线，它怎么知道现在操作的是哪条？答案就是把总线实例指针一起存进接口结构体，调用时传进来。

这样**同一份代码能同时服务多条总线**，而设备驱动完全不需要知道 `i2c_soft_t` 这个类型的存在。

`ctx` 用 `void *`（无类型指针）是 C 里实现"抽象接口"的标准手法：接口层只承诺"我给你一个指针，你原样还给我的回调"，不承诺它指向什么。实现里再转回来：

```c
i2c_soft_t *bus = (i2c_soft_t *)ctx;
```

`write_read` 用到了**重复起始条件**——写完命令后**不发停止条件**，直接再来一个起始条件切到读方向。这是读寄存器类器件的标准时序（比如 MPU6050 读陀螺仪数据就要这样）。SSD1306 用不上读方向，但实现了，后面加其他 I2C 器件时直接能用。

### 3.8 总线恢复

```c
error_t i2c_soft_bus_recover(i2c_soft_t *bus)
{
    i2c_soft_sda_high(bus);      /* 先松手，让从机有机会放手 */

    for (uint8_t i = 0U; i < I2C_SOFT_RECOVER_CLOCKS; i++) {   /* 9 个时钟 */
        i2c_soft_scl_low(bus);
        i2c_soft_delay(bus);
        i2c_soft_scl_high(bus);
        i2c_soft_delay(bus);
    }

    i2c_soft_stop(bus);

    return i2c_soft_sda_is_high(bus) ? ERR_OK : ERR_BUSY;
}
```

**这是在解决什么问题？** 假设主机正在读数据，读到一半被打断了（按了复位键、或电压抖动）。从机此时可能正拉到第 4 个 bit，它**以为还有数据要发，就一直拽着 SDA 不放**。这时主机想发起始条件，但 SDA 本来就是低的，"高变低"根本产生不了——**整条总线锁死**。

恢复办法：主机在 SCL 上**空打 9 个时钟**。从机会跟着数，数到 8 个 bit + 1 个应答位之后，就知道"这一帧结束了"，自动放手。最后补一个停止条件，总线复位。返回值告诉你 SDA 是否回到高——回到了才算真的恢复成功。

这是 I2C 的老毛病，面试也常问。STM32F103 的**硬件** I2C1 遇到这种情况常常需要整芯片复位才能恢复，这也是选软件 I2C 的原因之一。

### 3.9 初始化

```c
error_t i2c_soft_init(i2c_soft_t *bus, const i2c_soft_cfg_t *cfg)
{
    if ((bus == NULL) || (cfg == NULL)) {
        return ERR_INVALID_PARAM;              /* ① 先检查指针 */
    }
    if ((cfg->scl_port == NULL) || (cfg->sda_port == NULL)) {
        return ERR_INVALID_PARAM;
    }

    bus->cfg   = *cfg;                         /* ② 拷贝配置（不是存指针！） */
    bus->ready = false;

    /* ③ 由频率算出半周期延时，至少 1 微秒 */
    const uint32_t freq_hz = (cfg->freq_hz != 0U) ? cfg->freq_hz : I2C_SOFT_DEFAULT_FREQ_HZ;
    uint32_t       half_us = I2C_SOFT_US_PER_SEC / (freq_hz * 2U);
    if (half_us == 0U) { half_us = 1U; }
    bus->delay_us = half_us;

    /* ④ 填好接口函数表 */
    bus->iface.write      = i2c_soft_if_write;
    bus->iface.read       = i2c_soft_if_read;
    bus->iface.write_read = i2c_soft_if_write_read;
    bus->iface.ctx        = bus;               /* ← 关键：把自己传回去 */

    i2c_soft_timing_init();
    i2c_soft_gpio_config(cfg->scl_port, cfg->scl_pin);
    i2c_soft_gpio_config(cfg->sda_port, cfg->sda_pin);

    bus->ready = true;                         /* ⑤ 最后才置位 */

    return ERR_OK;
}
```

几个值得注意的地方：

**② 为什么拷贝而不是存指针？** 如果只存指针，调用者传进来的若是局部变量，函数返回后指针就悬空了——这类 bug 极难查。`bus->cfg = *cfg;` 把整个结构体复制一份，之后调用者怎么改都无关。

**⑤ 为什么 `ready = true` 放在最后？** `ready` 是其他函数的前置检查。如果提前置位，而后面某步失败或被打断，就会留下一个"看起来能用、实际没配好"的状态。**先干活，全干完了再挂牌营业。**

**关于第 ③ 步的数字**：`half_us = 1000000 / (400000 × 2) = 1`，即目标 400 kHz、半周期 1 µs。但**实际频率会低于 400 kHz**，因为每次翻转都要经过 `HAL_GPIO_WritePin` 的函数调用开销。实测约 300 kHz 上下。这完全够用（SSD1306 手册上限 400 kHz），而且更慢意味着更稳。想调速度改 `freq_hz` 即可。

---

## 四、器件层：ssd1306

### 4.1 核心概念：显存是怎么组织的

**这一节是理解整个驱动的地基。**

SSD1306 内部有一块 128×64 像素 = **8192 个点**的显存。但它**不是**按"一个像素一个字节"存的（那样要 8 KB），而是压得很紧——**一个字节存 8 个纵向像素**：

```
SSD1306 显存（GDDRAM）的内存布局：

        列 0    列 1    列 2   ...            列 127
      ┌───────┬───────┬───────┬─────────────┬───────┐
页 0  │ byte  │ byte  │ byte  │    ...      │ byte  │  ← 覆盖 y = 0..7
      ├───────┼───────┼───────┼─────────────┼───────┤
页 1  │ byte  │ byte  │ byte  │    ...      │ byte  │  ← 覆盖 y = 8..15
      ├───────┼───────┼───────┼─────────────┼───────┤
页 2  │ byte  │ byte  │ byte  │    ...      │ byte  │  ← 覆盖 y = 16..23
      ├───────┼───────┼───────┼─────────────┼───────┤
 ...  │  ...  │  ...  │  ...  │    ...      │  ...  │
      ├───────┼───────┼───────┼─────────────┼───────┤
页 7  │ byte  │ byte  │ byte  │    ...      │ byte  │  ← 覆盖 y = 56..63
      └───────┴───────┴───────┴─────────────┴───────┘
       共 8 页 × 128 列 = 1024 字节
```

**每一个字节内部，8 个 bit 从下往上对应 8 行像素**：

```
   一个字节 = 0b 1010 0110
                      ││││││││
   bit7 ──────────────┘││││││└── bit0
                       │││││└──── ...
   bit0 = 这一列最上面那一行的像素
   bit7 = 这一列最下面那一行的像素
```

**具体例子**：点亮坐标 `(x=5, y=10)` 的像素。

1. `y = 10` 落在哪一页？`10 / 8 = 1` → **页 1**（页 1 覆盖 y = 8..15）
2. 在页内的第几行？`10 % 8 = 2` → **该字节的 bit 2**
3. 这个字节在显存里的下标？`x + 页号 × 128 = 5 + 1 × 128 =` **133**
4. 所以要置位 `buffer[133]` 的 bit 2，即 `buffer[133] |= 0x04`

这就是 `ssd1306_draw_pixel()`：

```c
const uint32_t index = (uint32_t)x + ((uint32_t)(y / SSD1306_PIXELS_PER_PAGE) * SSD1306_WIDTH);
const uint8_t  bit   = (uint8_t)(1U << (y % SSD1306_PIXELS_PER_PAGE));

if (color == SSD1306_WHITE) {
    dev->buffer[index] |= bit;                      /* 置 1 = 点亮 */
} else {
    dev->buffer[index] &= (uint8_t)(~bit);          /* 清 0 = 熄灭 */
}
```

`1U << (y % 8)` 是"造一个只有第 n 位是 1 的掩码"。`~bit` 取反后是"只有第 n 位是 0 的掩码"，用它做**与**运算就能单独把这一位清掉，其他 7 位不受影响。

> ⚠️ **常见错误**：写成 `buffer[index] = bit`（直接赋值）。这会把那一列**其他 7 个像素全部抹掉**。必须用 `|=` 和 `&=` 这种"只动目标位"的写法。这叫**读-改-写**。

### 4.2 为什么必须本地开一块显存

```c
uint8_t buffer[SSD1306_BUFFER_SIZE];    /* 1024 字节，屏幕在 RAM 里的镜像 */
```

**为什么不直接往屏幕上写？** 两个原因：

1. **SSD1306 的显存读不回来**。它只支持写，你发不了"把第 133 号字节念给我听"这种命令。
2. **改一个像素需要知道邻居**。上一节说了，改一个 bit 要先读出这个字节的另外 7 个 bit，否则会误伤。而既然读不回来，就没法在屏幕那边做"读-改-写"。

所以标准做法是：**在 RAM 里开一块 1:1 的镜像**，所有绘图都在 RAM 里改，改完一次性整块推给屏幕。这就是 `ssd1306_update_screen()` 干的事。

代价是 1024 字节 RAM。F103C8T6 有 20 KB，目前总共用了 2.7 KB，完全负担得起。

### 4.3 命令表

SSD1306 靠"命令"控制。所有命令集中在 [ssd1306_reg.h](../driver/device/ssd1306/ssd1306_reg.h)：

```c
#define SSD1306_CMD_DISPLAY_ON           (0xAFU)  /**< 开启显示 */
#define SSD1306_CMD_SET_CONTRAST         (0x81U)  /**< 对比度，后跟 1 字节 */
#define SSD1306_CMD_SET_MULTIPLEX        (0xA8U)  /**< 多路复用比，后跟 1 字节 */
...
```

**为什么不直接在代码里写 `0xAF`？** 因为过两周再回来看，没人知道 `0xD9` 后面跟的 `0x22` 是什么意思。集中定义后，代码变成：

```c
ssd1306_write_cmd_arg(dev, SSD1306_CMD_SET_PRECHARGE, SSD1306_PRECHARGE_DEFAULT);
```

一看就知道在设预充电周期。这是嵌入式代码可维护性的基本功——**不许有魔数**。

### 4.4 SSD1306 的 I2C 帧格式

这点和"普通 I2C 器件"不一样。给 SSD1306 发数据时，**地址字节之后还要跟一个"控制字节"**，告诉它"后面这坨是命令还是显存数据"：

```
[0x78]  [控制字节]  [数据...]
   │        │
   │        ├─ 0x00 → 后面的都是【命令】
   │        └─ 0x40 → 后面的都是【显存数据】
   └─ 从机地址 0x3C 左移一位 + 写位
```

代码：

```c
static error_t ssd1306_write_cmd(ssd1306_t *dev, uint8_t cmd)
{
    const uint8_t frame[2] = { SSD1306_CTRL_BYTE_COMMAND, cmd };   /* {0x00, 命令} */
    return dev->bus->write(dev->bus->ctx, dev->addr, frame, 2);
}

static error_t ssd1306_write_cmd_arg(ssd1306_t *dev, uint8_t cmd, uint8_t arg)
{
    const uint8_t frame[3] = { SSD1306_CTRL_BYTE_COMMAND, cmd, arg };
    return dev->bus->write(dev->bus->ctx, dev->addr, frame, 3);
}
```

> ⚠️ **坑**：控制字节和后面的数据**必须在同一个 I2C 帧里**（同一个起始条件和停止条件之间）。不能分两次 `write` 调用——那样会变成两个独立的帧，中间夹了个停止条件，SSD1306 会把它当成两次独立传输，控制字节就失效了。

所以 `ssd1306_write_data()` 必须先拷贝一份再拼帧：

```c
static error_t ssd1306_write_data(ssd1306_t *dev, const uint8_t *data, uint32_t len)
{
    uint8_t frame[SSD1306_DATA_FRAME_MAX];        /* 129 字节的栈缓冲 */

    frame[0] = SSD1306_CTRL_BYTE_DATA;            /* 先塞控制字节 0x40 */
    for (uint32_t i = 0U; i < len; i++) {
        frame[i + 1U] = data[i];                  /* 再把 128 字节数据搬进来 */
    }

    return dev->bus->write(dev->bus->ctx, dev->addr, frame, len + 1U);
}
```

这 129 字节的栈缓冲是必要的开销。本项目栈是 1 KB，调用链很浅（`main → oled_update_counter → ssd1306_update_screen → ssd1306_write_data → i2c_soft_if_write`），占 129 字节安全。

### 4.5 初始化序列

上电后要按顺序发二十多条命令把屏配好。这个序列**沿用了上游 [afiskon/stm32-ssd1306](https://github.com/afiskon/stm32-ssd1306) 在同类廉价模块上的实测配置**，而不是照抄数据手册的复位值——便宜模块的面板参数经常和手册有出入，实测值更可靠。

摘几条关键的：

```c
/* 屏幕内部上电复位期间不接受命令，必须等 */
ssd1306_delay(dev, SSD1306_POWER_ON_DELAY_MS);        /* 100 ms */

if (ssd1306_write_cmd(dev, SSD1306_CMD_DISPLAY_OFF) != ERR_OK) {
    return ERR_COMMUNICATION;                          /* 先关显示，避免花屏 */
}

/* 寻址模式设为「水平」，适合整屏连续刷新 */
ssd1306_write_cmd_arg(dev, SSD1306_CMD_SET_MEM_ADDR_MODE, SSD1306_MEM_ADDR_MODE_HORIZONTAL);

/* 面板参数 */
ssd1306_write_cmd_arg(dev, SSD1306_CMD_SET_MULTIPLEX, (uint8_t)(SSD1306_HEIGHT - 1U));  /* 64-1=63 */
ssd1306_write_cmd_arg(dev, SSD1306_CMD_SET_CLOCK_DIV,  SSD1306_CLOCK_DIV_DEFAULT);
ssd1306_write_cmd_arg(dev, SSD1306_CMD_SET_PRECHARGE,  SSD1306_PRECHARGE_DEFAULT);
ssd1306_write_cmd_arg(dev, SSD1306_CMD_SET_COM_PINS,   SSD1306_COM_PINS_ALTERNATIVE);
ssd1306_write_cmd_arg(dev, SSD1306_CMD_SET_VCOMH,      SSD1306_VCOMH_077);

/* 这类模块没有外部 VCC，必须开内部电荷泵才有显示 */
ssd1306_write_cmd_arg(dev, SSD1306_CMD_CHARGE_PUMP, SSD1306_CHARGE_PUMP_ENABLE);

/* 清屏并上屏，避免残留随机内容 */
ssd1306_fill(dev, SSD1306_BLACK);
ssd1306_update_screen(dev);

/* 最后才开显示 */
ssd1306_write_cmd(dev, SSD1306_CMD_DISPLAY_ON);
```

设计要点：

**① 每一条命令都检查返回值。** `write` 失败（从机不应答）就立刻 `return ERR_COMMUNICATION`。如果忽视返回值，屏幕没接好时会得到"程序在跑但屏幕全黑"的现象，完全无从查起。

**② 先 `DISPLAY_OFF`，配完了再 `DISPLAY_ON`。** 这样初始化过程中不会出现"配了一半"的乱码画面。

**③ 100 ms 延时是必须的。** SSD1306 内部有上电复位电路，这期间不接收命令。这个延时通过 `dev->delay_ms` 回调实现：

```c
static void ssd1306_delay(ssd1306_t *dev, uint32_t ms)
{
    if (dev->delay_ms != NULL) {
        dev->delay_ms(ms);
    }
}
```

**注意这个函数自己**不调用 `HAL_Delay`。因为设备驱动一旦调用 HAL，就绑死在 STM32 上了。所以延时函数由上层**注入**进来（`ssd1306_delay_ms_fn`）。这是**依赖注入**，好处是 PC 上做单元测试时注入一个空函数，测试瞬间跑完，不用真等 100 ms。

**④ `SSD1306_CMD_CHARGE_PUMP`（电荷泵）是最容易漏的一条。** 这类模块只有 VCC/GND 两根电源线，没有额外的面板高压输入。SSD1306 内部要靠电荷泵把 3.3 V 升压到约 7.5 V 才能驱动 OLED 面板。不开这条命令，屏永远不会亮——**而且不会有任何报错**，因为通信是正常的。遇到"通信全对但屏幕全黑"，先查这条。

### 4.6 刷新屏幕

```c
error_t ssd1306_update_screen(ssd1306_t *dev)
{
    for (uint8_t page = 0U; page < SSD1306_PAGE_COUNT; page++) {     /* 8 页 */
        /* 告诉屏幕：我要写第 page 页，从第 0 列开始 */
        ssd1306_write_cmd(dev, (uint8_t)(SSD1306_CMD_SET_PAGE_START | page));  /* 0xB0|page */
        ssd1306_write_cmd(dev, SSD1306_CMD_SET_LOW_COLUMN  | 0x00U);           /* 0x00|0 */
        ssd1306_write_cmd(dev, SSD1306_CMD_SET_HIGH_COLUMN | 0x00U);           /* 0x10|0 */

        /* 把这一页的 128 字节推过去 */
        const uint8_t *page_data = &dev->buffer[(uint32_t)page * SSD1306_WIDTH];
        ssd1306_write_data(dev, page_data, SSD1306_WIDTH);
    }
    return ERR_OK;
}
```

**`0xB0 | page` 这个写法**：命令 `0xB0` 的低 3 位就是页号。所以 `0xB0 | 0` = 第 0 页，`0xB0 | 3` = 第 3 页。这种"命令 + 数据在同一字节里"的设计在 SSD1306 里很常见（列地址高低位也是）。

**`&dev->buffer[page * 128]`**：因为 buffer 里也是按页顺序连续排的（第 0~127 字节是页 0，第 128~255 字节是页 1……），直接按页号偏移就能拿到对应那一页的数据。

**为什么每次都要重发页地址和列地址？** 虽然初始化时设了水平寻址模式，每次只写 128 字节就到页尾了，重新发一次更稳妥、意图也更明确。代价是每页多 3 次小传输，可忽略。

**性能估算**：一屏 1024 字节，加控制字节和地址开销约 1.1 KB，按 300 kHz 算大概 **30 ms**。

### 4.7 写字：字模是怎么被读出来的

字库里的字模是这么存的（[ssd1306_fonts.c](../driver/device/ssd1306/ssd1306_fonts.c)）：

```c
static const uint16_t Font6x8 [] = {
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,  // sp  (空格)
0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x0000, 0x1000,  // !   (感叹号)
...
```

**每个字符占 8 个 `uint16_t`**（因为 6×8 字体高 8 行），**每一行一个 `uint16_t`，有效位在最高位那一侧**。

看感叹号 `0x1000`：二进制 `0001 0000 0000 0000`。6 像素宽的字，取最高的 6 位 `000100`：

```
行0: 0x1000 → 0001 00  →  ..#...     ← 竖线的顶端
行1: 0x1000 → 0001 00  →  ..#...
行2: 0x1000 → 0001 00  →  ..#...
行3: 0x1000 → 0001 00  →  ..#...
行4: 0x1000 → 0001 00  →  ..#...
行5: 0x1000 → 0001 00  →  ..#...
行6: 0x0000 → 0000 00  →  ......     ← 空行（间隔）
行7: 0x1000 → 0001 00  →  ..#...     ← 下面的点
```

就是"一根竖线 + 底下一个点"，正是感叹号的样子。

**为什么数据放在高位而不是低位？** 因为字宽不固定（有 6×8、7×10、11×18）。统一高位对齐后，只要往左移位就能取出有效的那几位，不用管字宽是多少。

取位的方式：

```c
for (uint8_t row = 0U; row < font->height; row++) {
    const uint16_t row_bits = font->data[((uint32_t)index * font->height) + row];

    for (uint8_t col = 0U; col < char_width; col++) {
        /* 字模高位对齐，左移 col 位后看最高位即可 */
        const bool lit = ((uint16_t)(row_bits << col) & SSD1306_FONT_MSB_MASK) != 0U;
        const ssd1306_color_t pixel = lit ? color : (ssd1306_color_t)(!color);

        ssd1306_draw_pixel(dev, dev->cursor_x + col, dev->cursor_y + row, pixel);
    }
}
```

**`(row_bits << col) & 0x8000` 是什么意思？** 把"从左边数第 col 位"挪到最高位，然后检查它是不是 1。举例 `row_bits = 0xFC00`（`1111 1100 0000 0000`）：

| col | `row_bits << col` | `& 0x8000` | 结果 |
|---|---|---|---|
| 0 | `0xFC00` | `0x8000` | **亮** |
| 1 | `0xF800` | `0x8000` | **亮** |
| 2 | `0xF000` | `0x8000` | **亮** |
| 3 | `0xE000` | `0x8000` | **亮** |
| 4 | `0xC000` | `0x8000` | **亮** |
| 5 | `0x8000` | `0x8000` | **亮** |
| 6 | `0x0000` | `0x0000` | 灭 |

6 个像素亮、后面灭，正好画出一条 6 像素宽的横线。注意 `(uint16_t)` 那个强制转换——`row_bits` 会被提升成 `int`，移位后会超过 16 位，转回 `uint16_t` 相当于自动截断，正是要的效果（只关心低 16 位）。

**`index` 的算法**：

```c
const uint8_t index = (uint8_t)(char_code - SSD1306_FONT_FIRST_CHAR);   /* ch - 32 */
const uint16_t row_bits = font->data[((uint32_t)index * font->height) + row];
```

字库里从 ASCII 码 **32（空格）** 开始存，所以取第 32 个字符要先减 32 得到索引。然后 `索引 × 字高 + 行号` 就是那一行数据在数组里的位置。

**字库只存了 32~126 共 95 个字符，没有中文。** 所以 `ssd1306_write_char` 开头就检查：

```c
if ((char_code < SSD1306_FONT_FIRST_CHAR) || (char_code > SSD1306_FONT_LAST_CHAR)) {
    return 0;   /* 超出范围，不显示 */
}
```

点阵中文需要的字模是英文的几百倍，得用取模软件按需生成。本工程按需求不做中文显示。

**比例字体的处理**：

```c
const uint8_t char_width = (font->char_width != NULL) ? font->char_width[index] : font->width;
```

`Font_11x18` 是等宽字体，`char_width` 为 `NULL`，直接用 `font->width`。但从 Google Roboto 转来的 `Font_16x15` 是**比例字体**，每个字符宽度不同（`i` 窄、`W` 宽），它带一张 `char_width[]` 宽度表。代码两种都支持，所以 `SSD1306_Font_t` 里那个字段不能删。

**光标机制**：

```c
dev->cursor_x += char_width;     /* 写完一个字符，光标右移它的宽度 */
```

`ssd1306_write_string` 就是循环调用 `write_char` 直到遇到 `'\0'` 或者写不下为止：

```c
while (*str != '\0') {
    if (ssd1306_write_char(dev, *str, font, color) == 0) {
        break;      /* 放不下了，停止 */
    }
    written++;
    str++;
}
```

**放不下时为什么不换行？** 这是设计取舍。`write_char` 返回 0 表示"本行放不下"，`write_string` 选择停止。要自动换行的话，可以在这里加：把 `cursor_x` 归零、`cursor_y` 加上字高。

### 4.8 画线：Bresenham 算法

屏幕上没有"直线"这种硬件，只能一个点一个点地画。最笨的办法是算斜率 `y = y1 + (x-x1) * (y2-y1)/(x2-x1)`，但**嵌入式里要做浮点除法**，慢且有精度问题。

**Bresenham 算法**只用整数加减：

```c
int32_t err = dx + dy;

for (;;) {
    ssd1306_draw_pixel(dev, (uint16_t)x, (uint16_t)y, color);

    if ((x == (int32_t)x2) && (y == (int32_t)y2)) {
        break;                        /* 到达终点 */
    }

    const int32_t err2 = 2 * err;
    if (err2 >= dy) {
        err += dy;
        x += step_x;                  /* 水平方向走一步 */
    }
    if (err2 <= dx) {
        err += dx;
        y += step_y;                  /* 垂直方向走一步 */
    }
}
```

**核心思想**：`err` 是个"累积误差"。每走一步误差就偏一点，当误差超过阈值时，在另一个方向上也走一步。这样画出来的永远是**最接近理想直线的整数像素**，全程只有整数运算。

> `dy` 初始化时**故意取了负数**（`dy = -|y2-y1|`），这样 `err` 的初值 `dx + dy` 一开始就落在合理区间，省掉一次偏移调整。这是 Bresenham 的标准写法，第一次看会觉得别扭。

`draw_rectangle` 就是画四条线，`fill_rectangle` 就是双重循环打点。

> ⚠️ **隐患**：`fill_rectangle` 里 `for (uint16_t y = y1; y <= y2; y++)`，如果 `y2` 是 `65535`，`y++` 会溢出回 0，**死循环**。目前调用方传的是常量，不会出问题；但若将来这个函数由外部数据驱动，需要加保护。**这就是为什么嵌入式代码里到处都要做参数检查。**

### 4.9 关于"用别人的字库"

字库文件从 [afiskon/stm32-ssd1306](https://github.com/afiskon/stm32-ssd1306)（MIT 协议）**逐字节原样**引入，没有改一个字。判断依据：**字模数据是纯资产，不是逻辑**，80 KB 的手工点阵数据没有"重复造"的价值，自己生成反而容易出错。

但有个约束传到了驱动头文件上：

```c
typedef struct {
    const uint8_t         width;
    const uint8_t         height;
    const uint16_t *const data;
    const uint8_t *const  char_width;
} SSD1306_Font_t;
```

**字段顺序必须和上游严格一致**——上游的 `ssd1306_fonts.c` 里是用**位置初始化**填充的：

```c
const SSD1306_Font_t Font_6x8 = {6, 8, Font6x8, NULL};
                                 ↑  ↑  ↑        ↑
                              宽  高  数据   比例表
```

把 `width` 和 `height` 调换顺序，字模就会整个错位，屏上显示乱七八糟的东西。

**另一个细节**：上游的 `ssd1306_fonts.c` 靠 `<_ansi.h>`（newlib 内部头文件）间接拿到 `NULL` 的定义。本工程的 `ssd1306.h` 里没有这个，第一次编译报 `'NULL' undeclared`。解决办法是在 `ssd1306.h` 里显式 `#include <stddef.h>`——这样**字库文件仍然一个字都不用改**。凡是要改上游文件才能用的情况，都优先在**自己的**文件里想办法。

---

## 五、BSP 层：把零件装起来

[bsp_oled.c](../bsp/oled/bsp_oled.c) 很短，但它决定了"这块板子上，屏接在哪"。

```c
#define BSP_OLED_SCL_PORT   (GPIOB)
#define BSP_OLED_SCL_PIN    (GPIO_PIN_8)      /* SCL 接 PB8 */
#define BSP_OLED_SDA_PORT   (GPIOB)
#define BSP_OLED_SDA_PIN    (GPIO_PIN_9)      /* SDA 接 PB9 */

error_t bsp_oled_init(ssd1306_t *dev)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();     /* ← 注意这一步！ */

    const i2c_soft_cfg_t bus_cfg = {
        .scl_port = BSP_OLED_SCL_PORT,
        .scl_pin  = BSP_OLED_SCL_PIN,
        .sda_port = BSP_OLED_SDA_PORT,
        .sda_pin  = BSP_OLED_SDA_PIN,
        .freq_hz  = I2C_SOFT_DEFAULT_FREQ_HZ,
    };

    error_t err = i2c_soft_init(&s_oled_bus, &bus_cfg);
    if (err != ERR_OK) {
        return err;
    }

    dev->bus      = i2c_soft_iface(&s_oled_bus);   /* 注入总线 */
    dev->addr     = BSP_OLED_I2C_ADDR;             /* 注入地址 */
    dev->delay_ms = bsp_oled_delay_ms;             /* 注入延时 */

    return ssd1306_init(dev);
}
```

**`__HAL_RCC_GPIOB_CLK_ENABLE()` 这行非常关键，也最容易漏。** STM32 的每个外设（包括每个 GPIO 端口）都有独立的时钟开关，**不开时钟，写它的寄存器完全无效，而且是静默的**——不报错、不报警，就是没反应。

本工程只用了 PC13（LED）和 PA13/PA14（SWD），所以 CubeMX 生成的 `MX_GPIO_Init()` 里只有 GPIOA/C/D，**没有 GPIOB**。因此 BSP 必须自己开。这是新手做第二、第三个外设时最常踩的坑——代码逻辑全对，就是没反应。

**关于 `dev->bus / addr / delay_ms` 这三个"注入"**：设备驱动不认识具体总线、不认识具体地址、不认识延时怎么实现，全由上层填好再调 `init`。好处是同一份 `ssd1306.c` 可以用在软件 I2C、硬件 I2C、SPI 转 I2C 上，也可以在 PC 上用假总线跑。

### 总线扫描

```c
for (uint8_t addr = BSP_OLED_SCAN_ADDR_FIRST; addr <= BSP_OLED_SCAN_ADDR_LAST; addr++) {
    uint8_t dummy = 0U;

    /* 向该地址发起一次单字节读，只看「有没有人应答」 */
    if (s_oled_bus.iface.read(s_oled_bus.iface.ctx, addr, &dummy, 1U) == ERR_OK) {
        found[count] = addr;
        count++;
    }
}
```

**原理**：对每个地址发一次读操作。器件存在就会在自己的地址字节后回 ACK，于是 `read` 返回 `ERR_OK`；不存在则没人应答，返回 `ERR_COMMUNICATION`。读到的内容丢掉不看，**只要"有没有人应答"这 1 bit 信息**。

这就是 I2C "有应答"这个机制最实用的地方——它让总线扫描成为可能，插上任何器件都能扫出来确认接线对不对。

扫描范围取 `0x08 ~ 0x77`，因为 I2C 规范里 `0x00~0x07` 和 `0x78~0x7F` 是保留地址（后者包含 10 位地址前缀和广播地址），扫它们没意义。

**这个函数在实战中很值钱**：以后接 MPU6050、接温湿度传感器，第一件事都是扫描一遍确认地址。

---

## 六、测试代码

[main.c](../Core/Src/main.c) 里分了四块。

### 6.1 布局常量

```c
#define OLED_LINE_TITLE_Y   (0U)    /* 标题，Font_7x10，占 0~9   */
#define OLED_LINE_SCAN_Y    (11U)   /* 扫描结果，Font_6x8，占 11~18 */
#define OLED_LINE_RULE_Y    (20U)   /* 分隔线 */
#define OLED_LINE_MAIN_Y    (23U)   /* 主信息，Font_11x18，占 23~40 */
#define OLED_LINE_PIN_Y     (43U)   /* 接线说明，Font_6x8，占 43~50 */
#define OLED_LINE_COUNT_Y   (53U)   /* 循环计数，Font_6x8，占 53~60 */
```

**为什么把坐标提出来做宏？** 屏幕有 64 行，字号又有 8/10/18 三种高度，一行一行手算很费劲。集中定义后，改版式只改这几个数。算一下就知道有没有重叠：`0~9`、`11~18`、线在 `20`、`23~40`、`43~50`、`53~60`。**全部塞进 0~63，没有重叠**。

### 6.2 数字转字符串

```c
static uint8_t oled_u32_to_dec(uint32_t value, char *out)
{
    char    tmp[10];
    uint8_t n = 0U;

    if (value == 0U) {
        out[0] = '0';  out[1] = '\0';  return 1U;
    }

    while ((value > 0U) && (n < sizeof(tmp))) {
        tmp[n] = (char)('0' + (value % 10U));    /* 取个位，变成 ASCII */
        value /= 10U;                            /* 去掉个位 */
        n++;
    }

    for (uint8_t i = 0U; i < n; i++) {
        out[i] = tmp[n - 1U - i];                /* 逆序翻回来 */
    }
    out[n] = '\0';

    return n;
}
```

**为什么不用 `snprintf`？** 因为 `("%d", n)` 会把 newlib 里**整套 printf 格式化引擎**链接进来，Flash 上要多花 2~4 KB，就为了显示一个整数。F103C8T6 只有 64 KB Flash，为这点事不划算。自己写 15 行搞定。

**`'0' + (value % 10)` 这个写法值得记一下**：ASCII 表里 `'0'` 是 48，`'1'` 是 49……数字字符是连续的。所以 `'0' + 0` 是 `'0'`，`'0' + 7` 是 `'7'`。把 0~9 的数值变成字符就是这么一句。

**为什么要用 `tmp[]` 中转再逆序？** 取余数天然是从低位到高位（先拿到个位），而显示要从高位到低位。所以先逆序存进临时数组，再翻回来。

### 6.3 静态画面（只画一次）

```c
static void oled_draw_static_screen(void)
{
    uint8_t       found[BSP_OLED_SCAN_MAX];
    const uint8_t count = bsp_oled_bus_scan(found, BSP_OLED_SCAN_MAX);

    ssd1306_fill(&s_oled, SSD1306_BLACK);            /* ① 清屏（改的是 RAM） */

    ssd1306_set_cursor(&s_oled, 0U, OLED_LINE_TITLE_Y);
    ssd1306_write_string(&s_oled, "PID Pendulum", &Font_7x10, SSD1306_WHITE);

    /* ② 打印扫描结果 —— 这一段是自检 */
    ssd1306_set_cursor(&s_oled, 0U, OLED_LINE_SCAN_Y);
    if ((count == 1U) && (found[0] == BSP_OLED_I2C_ADDR)) {
        ssd1306_write_string(&s_oled, "scan: 0x3C ok", &Font_6x8, SSD1306_WHITE);
    } else {
        ...  /* 显示 "scan: N dev!" */
    }

    ssd1306_draw_line(&s_oled, 0U, OLED_LINE_RULE_Y, 127U, OLED_LINE_RULE_Y, SSD1306_WHITE);
    ...
    ssd1306_update_screen(&s_oled);                  /* ③ 一次性推给屏幕 */
}
```

**② 的这段自检是测试代码里最有价值的部分。** 它把"接线对不对"这个原本要靠示波器才能确认的事情，变成屏幕上的一行字。

**③ `update_screen` 是所有绘图动作的"提交点"。** 前面所有 `write_string`、`draw_line` 都只是在改 RAM 里的 buffer。只有调用了 `update_screen`，内容才会真的到屏幕上。

**这个"改 buffer → 提交"的模式是图形编程的基础。** 好处是：一屏内容改 100 次也只通信一次；改到一半的内容不会被看到（不会闪烁撕裂）。

### 6.4 动态计数

```c
static void oled_update_counter(uint32_t count)
{
    char msg[12];

    /* 先把整行清掉，否则位数变少时会留下残影 */
    ssd1306_fill_rectangle(&s_oled, 0U, OLED_LINE_COUNT_Y,
                           127U, OLED_LINE_COUNT_Y + 7U, SSD1306_BLACK);

    ssd1306_set_cursor(&s_oled, 0U, OLED_LINE_COUNT_Y);
    ssd1306_write_string(&s_oled, "loop: ", &Font_6x8, SSD1306_WHITE);
    oled_u32_to_dec(count, msg);
    ssd1306_write_string(&s_oled, msg, &Font_6x8, SSD1306_WHITE);

    ssd1306_update_screen(&s_oled);
}
```

**"先清行再写"是必须的，而且是很容易漏的经典 bug。** 想想位数减少的情况：从 `loop: 100` 变成 `loop: 99`。新内容只写到第 8 个字符，**第 9 个字符位置还留着上一帧的 `0`**，屏幕上就会显示 `loop: 990`——看起来像数字跳错了。

用 `fill_rectangle` 把整行涂黑再写，就彻底避免了。**这是所有"更新局部区域"的显示代码都要注意的事。**

### 6.5 主循环

```c
if (bsp_oled_init(&s_oled) != ERR_OK) {
    /* 屏幕没应答：LED 以 10 Hz 快闪作为错误指示 */
    while (1) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        HAL_Delay(50);
    }
}

oled_draw_static_screen();

while (1) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    loop_count++;
    oled_update_counter(loop_count);
    HAL_Delay(500);
}
```

**初始化失败时为什么要快闪？** 如果只是 `while(1);` 死循环，看到的现象是"屏幕全黑 + LED 不亮"，这跟"程序没烧进去"、"板子没供电"、"屏坏了"的现象**完全一样**，没法区分。

用 LED 闪烁频率编码错误状态，是嵌入式调试的经典手法。**10 Hz 快闪 = 屏没通，1 Hz 慢闪 = 一切正常**，一眼就能分辨。

**为什么循环里既闪 LED 又刷屏？** 这是刻意设计的**自检机制**：两者节奏绑定（都是 500 ms）。如果哪天主循环里某处阻塞了，LED 闪烁和屏幕计数会**同时**变慢，立刻能看出来。只做其中一个就没这个参照了。

---

## 七、如何验证

手上没有摄像头看不到屏时，"能编译"和"烧进去了"都**不等于代码跑对了**。用下面三个层次验证。

### 第 1 层：编译零警告

`-Wall` 下零警告。嵌入式里警告往往就是真 bug 的前兆（比如有符号/无符号比较、隐式截断）。

### 第 2 层：通过调试口读回运行时变量

用 ST-Link 的调试口直接读 RAM，**不打断程序运行**（`mode=hotplug` 不复位芯片）：

```
0x2000043C : 00000101
             ││
             │└─ display_on  = 1  ← 显示已开
             └── initialized = 1  ← ssd1306_init() 成功返回
```

**`initialized = 1` 是最关键的证据。** 它的含义是：SSD1306 **应答了 0x3C 地址，并且 25 条初始化命令全部被接受**。而"能应答"要求 PB8/PB9 接线正确、上拉有效、位翻转时序正确、地址没写错——**一次性把整条硬件链路都验证了**。

再看 `loop_count` 是否在增长，可确认主循环在跑、`update_screen` 那趟 30 ms 的 I2C 传输没有卡住。

符号地址可以这样取（不同构建会变）：

```bash
arm-none-eabi-nm --print-size build/Debug/PID_Pendulum.elf | grep loop_count
```

### 第 3 层：把显存读出来还原成图像

既然显存在 RAM 里，就可以把它读出来、按 [4.1 节](#41-核心概念显存是怎么组织的)的规则反解成像素，直接看到"屏幕上现在是什么"：

```
|.####....###...###...........####.....................#.........###......|
|.#...#....#....#..#..........#...#....................#...........#......|
   P    I    D              P    e    n    d    u    l    u    m
```

**并且做了自洽校验**：那一刻 `loop_count = 159`，屏幕上画出来的数字就是 `159`（1、5、9 三个字形逐一可辨），光标停在 x=54 = 9 字符 × 6 像素——**数字、字符数、光标位置三者严格对上**。这证明字模渲染、字符宽度计算、光标推进全对。

> 这招很实用：以后每加一个显示功能，都能在"没有摄像头"的情况下确认屏上画对了。

---

## 八、容易踩的坑

| 坑 | 现象 | 原因 |
|---|---|---|
| 忘记开 GPIO 时钟 | 代码全对，引脚毫无反应，且无任何报错 | `__HAL_RCC_GPIOB_CLK_ENABLE()` 必须显式调用 |
| 忘记开电荷泵 | 通信完全正常，但屏永远不亮 | `CHARGE_PUMP` 命令是这类模块的必需品，无外部 VCC |
| 控制字节和数据分两次发 | 屏上花屏或完全不动 | SSD1306 的控制字节必须和数据在同一 I2C 帧内 |
| 画点用 `=` 而不是 `\|=` | 一列像素被整列抹掉 | 要读-改-写，只动目标位 |
| 更新局部时不清旧内容 | 数字位数变少时残留旧字符，显示 `990` 而非 `99` | 先 `fill_rectangle` 清行 |
| 地址写成 0x3C 直接发 | 从机不应答 | 发出去的应该是 `0x3C << 1 = 0x78` |
| SDA 用推挽输出 | 偶发短路、发热，多器件时更明显 | I2C 必须开漏 + 上拉 |
| 延时用 `CYCCNT < start + wait` | 约 60 秒后随机卡死一次 | 32 位计数器回绕，要用减法 |
| 模块是 SH1106 而非 SSD1306 | 显示正常但整体水平偏移 2 像素 | SH1106 是 132 列的控制器，需要设 `X_OFFSET` |

---

## 附：如何复用这套结构

以后加编码器、电机、陀螺仪，照这个模式复制即可：

1. `driver/bus/` 里加总线（SPI、UART……），实现对应的抽象接口
2. `driver/device/` 里加器件驱动，只依赖抽象接口，不碰 HAL
3. `bsp/` 里接线（哪个引脚、哪个地址、延时怎么实现）
4. `main.c` 里调用

分层的好处到第二个、第三个模块才真正体现出来。
