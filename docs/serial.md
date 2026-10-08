# 串口（BSP Serial）实现详解

本文逐层拆解本仓库串口模块的实现：串口的时间预算、CubeMX 配了什么、
环形缓冲为什么可以不加锁、以及怎么用两条独立通道把「它真的通了」证出来。

**面向读者**：嵌入式初学者。假设你会 C（结构体、指针、位运算），
但不要求懂 USART 的寄存器细节。

**配套代码**：

| 文件 | 层 | 职责 |
| --- | --- | --- |
| [PID_Pendulum.ioc](../PID_Pendulum.ioc) | CubeMX | USART1 的声明（异步、PA9/PA10、115200-8-N-1） |
| [Core/Src/main.c](../Core/Src/main.c) 生成部分 | CubeMX 生成 | `MX_USART1_UART_Init()`、NVIC 调用 |
| [Core/Src/stm32f1xx_hal_msp.c](../Core/Src/stm32f1xx_hal_msp.c) | CubeMX 生成 | USART1 时钟使能、PA9/PA10 复用配置、`HAL_NVIC_EnableIRQ` |
| [bsp/serial/bsp_serial.h](../bsp/serial/bsp_serial.h) · [.c](../bsp/serial/bsp_serial.c) | BSP | 双向环形缓冲、中断回调、`printf` 重定向 |
| [Core/Src/main.c](../Core/Src/main.c) | 应用 | 临时自检代码（回显 + 周期上报） |

---

## 目录

- [一、要解决什么问题](#一要解决什么问题)
- [二、串口基础：三个必须先搞清的概念](#二串口基础三个必须先搞清的概念)
- [三、CubeMX 配了什么](#三cubemx-配了什么)
- [四、手写的部分](#四手写的部分)
- [五、怎么测试](#五怎么测试)
- [六、设计取舍一览](#六设计取舍一览)
- [附：这次踩过的坑](#附这次踩过的坑)

---

## 一、要解决什么问题

### 1.1 需求与约束

用 USART1 打通 MCU 与 PC 的串口通信，供后续调试（打印 PID 参数、把采样数据
发给上位机画波形）用。

拿到需求先把**手上的资源和限制**列清楚：

| 问题 | 答案 |
| --- | --- |
| 用哪个 USART？ | USART1（APB2，72 MHz） |
| 用哪两个脚？ | PA9 = TX，PA10 = RX |
| 这两个脚被占了吗？ | 没有。PA9 同时是 TIM1_CH2、PA10 是 TIM1_CH3，但 TIM1 只用了更新中断做节拍，没占用通道引脚 |
| 对端是什么？ | PC 上的 USB-TTL 模块（CH340），枚举成 COM7 |
| 波特率？ | 115200（CubeMX 默认值） |

### 1.2 该放哪一层

本仓库的分层是 `应用层 → bsp → driver/device → driver/bus → HAL`。

串口和已有的模块对比：

- 它**不是器件驱动**（`driver/device/`）——USART 是 MCU 内部外设，没有独立器件；
- 它**比 `bsp_key` / `bsp_pot` 复杂**——但复杂度来自「缓冲与并发」，
  而不是来自「这块板子上什么接在什么脚上」；
- 它**不需要 `driver/bus` 那样的抽象**——HAL 本身就是总线层，
  再包一层接口是过度设计（等真有第二个串口/另一颗芯片时再说）。

而且它直接调 `HAL_UART_*`，和 `bsp_key` 直接调 `HAL_GPIO_ReadPin` 是同一种做法。

→ 放 `bsp/serial/`。

---

## 二、串口基础：三个必须先搞清的概念

### 2.1 时间预算：87 µs 从哪来

这一节决定后面所有设计，必须先算清楚。

```
波特率 115200 bps = 每秒 115200 个「位」
一帧 = 起始位(1) + 数据位(8) + 停止位(1) = 10 位
所以：115200 ÷ 10 = 11520 字节/秒
     → 每个字节 1/11520 ≈ 86.8 µs
```

**87 µs 就是本模块的时间预算。** 记住这个数，下面两个结论都从它来。

### 2.2 为什么接收必须中断，发送可以不

**接收**：如果靠主循环轮询 `HAL_UART_Receive()`，那么从「一个字节到达」到
「主循环来取」之间不能超过 87 µs。而本工程的主循环里挂着 OLED 整屏刷新
（软件 I2C 写 1024 字节显存），耗时是**毫秒**量级——差了两个数量级。

> 结论：**接收必须中断驱动，没有商量余地。**

**发送**：反过来，如果发送用轮询（`HAL_UART_Transmit`），
发 32 字节要 `32 × 87 µs ≈ 2.8 ms`。这段时间主循环完全停住，
按键不响应、电位器不采样。

> 结论：发送**可以**轮询，但会阻塞 2.8 ms。做成中断驱动是为了
> **不阻塞主循环**——这正是本仓库其他模块都在遵循的同一原则。

### 2.3 HAL 的中断收发是怎么工作的

理解这一点，才能看懂驱动为什么要那样写。

**发送**（`HAL_UART_Transmit_IT`）：

它**不搬运数据**。它只做三件事——记住你给的指针和长度、把状态标成「忙」、
使能 TXE（发送数据寄存器空）中断，然后立刻返回。

之后每来一次 TXE 中断，HAL 从缓冲区取**一个字节**塞进 DR。
发完最后一个字节，还要等 TC（发送完成）标志——它表示最后一位已经从移位寄存器
移出去了——然后才调用 `HAL_UART_TxCpltCallback()`。

> 这解释了两件事：
> ① 为什么 `bsp_serial_write()` 返回时数据可能一个字节都还没发出去；
> ② 为什么需要一个 `s_tx_busy` 标志——同一时刻只能有一笔传输在跑。

**接收**（`HAL_UART_Receive_IT`）：

它指定一个缓冲区和长度（我们给 1）。每收满这么多个字节，
就调用一次 `HAL_UART_RxCpltCallback()`，**并且自动关掉接收中断**。

> 这解释了那条最容易漏的规则：**回调里必须重新调用 `HAL_UART_Receive_IT()`**。
> 不重新武装，串口只能收到**一个字节**就再无下文，而且不报任何错。

---

## 三、CubeMX 配了什么

**原则**：`bsp_serial.c` 里一行寄存器配置都没有。波特率、字长、校验、
引脚模式、NVIC 全在 `.ioc` 里。

### 3.1 命令行配方

分两步走，和 [docs/pot.md](pot.md) 一样**一次只加一点**，出错时好定位。

**第一步：只激活外设 + 分配引脚**

```text
set mode USART1 "Asynchronous"
set pin PA9 USART1_TX
set pin PA10 USART1_RX
```

模式名 `Asynchronous` 和信号名 `USART1_TX` 都是用
`get modes USART1` / `get functions PA9` **查出来的，不是猜的**。

**第二步：加参数**

```text
set ip parameters USART1 WordLength WORDLENGTH_8B
set ip parameters USART1 Parity     PARITY_NONE
set ip parameters USART1 StopBits   STOPBITS_1
```

⚠️ **这三个取值是 F1 专用的短名**，写错后果严重，见 3.4 节。

**波特率没有写进配方**，因为 115200 就是 CubeMX 的默认值，
而 **CubeMX 不写等于默认值的键**。这不是失败——去生成的代码里核对即可。

**第三步：使能中断**（CLI 做不到，要 `inject`）

```bash
python cubemx_cfg.py inject <工程> \
  --key 'NVIC.USART1_IRQn=true\:2\:0\:false\:false\:true\:true\:true\:true'
```

那串值的格式是 `true\:抢占优先级\:子优先级\:false\:false\:true\:true\:true\:true`。

**优先级为什么取 2**：先查现状——

```
NVIC.PriorityGroup = NVIC_PRIORITYGROUP_4     ← 4 位抢占优先级，取值 0~15
NVIC.TIM1_UP_IRQn  = true\:1\:0\:...          ← 系统节拍占 1
```

给 USART1 取 **2（比节拍低）**。理由：节拍的抖动影响按键消抖计时；
而 115200 下一字节有 87 µs 余量，串口中断晚几微秒无所谓。

### 3.2 生成出来的代码

**`Core/Src/main.c`**：

```c
huart1.Instance          = USART1;
huart1.Init.BaudRate     = 115200;
huart1.Init.WordLength   = UART_WORDLENGTH_8B;
huart1.Init.StopBits     = UART_STOPBITS_1;
huart1.Init.Parity       = UART_PARITY_NONE;
huart1.Init.Mode         = UART_MODE_TX_RX;
huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
huart1.Init.OverSampling = UART_OVERSAMPLING_16;
if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
```

**`Core/Src/stm32f1xx_hal_msp.c`**：

```c
__HAL_RCC_USART1_CLK_ENABLE();
HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
HAL_NVIC_EnableIRQ(USART1_IRQn);
/* PA9 复用推挽、PA10 浮空输入，由 HAL_UART_MspInit() 完成 */
```

**`Core/Src/stm32f1xx_it.c`**：

```c
void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);      /* 统一分发到各个回调 */
}
```

**验收标准**：上面这些必须**逐项在文件里找到**。「参数被接受」≠「代码被生成」。

### 3.3 一处 CubeMX 不会做的：CMake 源文件清单

CubeMX 会把 `stm32f1xx_hal_uart.c` 拷进 `Drivers/`，也会自动打开
`stm32f1xx_hal_conf.h` 里的 `HAL_UART_MODULE_ENABLED`，**但不会更新
`cmake/stm32cubemx/CMakeLists.txt` 的源文件列表**。

漏了的症状是链接期：

```
undefined reference to 'HAL_UART_Init'
```

要手工补进 `STM32_Drivers_Src`：

```cmake
${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
```

这是安全的——那个文件和顶层 `CMakeLists.txt` 一样**只在首次生成时产出**，
之后重新生成不会冲掉手工加的行。

### 3.4 踩过的坑：F1 的参数取值格式和别的系列不一样

**这是本模块最值钱的一条。**

一开始按常见的写法写了：

```text
set ip parameters USART1 WordLength UART_WORDLENGTH_8B     ← 错
set ip parameters USART1 Parity     UART_PARITY_NONE       ← 错
set ip parameters USART1 StopBits   UART_STOPBITS_1        ← 错
```

**四层检查全部通过**：

| 检查 | 结果 |
| --- | --- |
| 命令返回 | ✅ OK |
| 键写进 `.ioc` | ✅ 在 |
| `roundtrip` 往返比对 | ✅ 「原样保留」 |
| 生成代码、编译 | ✅ 通过 |

只有 CubeMX 的日志里有一句：

```
IP (USART1) : Parameter (WordLength) has invalid value (UART_WORDLENGTH_8B)
IP not ready for code generation: USART1
```

**注意措辞**：对比 RCC 的 `Invalid parameter (ADCCLKDivider)`（**参数名**不认），
这里报的是 `Parameter (WordLength) has invalid **value**`——**名字对、取值错**。

**后果**：CubeMX 把 USART1 标成 not-ready，仍然生成了 `MX_USART1_UART_Init()`，
但**只写了一半字段**——`WordLength` / `StopBits` / `Parity` 三行直接消失。

而 `huart1` 是全局对象，C 标准保证静态存储期对象零初始化，所以这三个字段是 **0**。
再查 HAL 的宏：

```c
#define UART_WORDLENGTH_8B    0x00000000U
#define UART_STOPBITS_1       0x00000000U
#define UART_PARITY_NONE      0x00000000U
```

**三个全是 0**——零初始化**碰巧就是 8 数据位 / 1 停止位 / 无校验**，正是要的配置。
所以**串口真的能通，错误被硬件行为完美掩盖**。

**正确取值从哪来**：CubeMX 自己的定义文件。用 `VM_ASYNC` 当特征串定位：

```bash
grep -rl "VM_ASYNC" db/ | grep F1xx
# → db/mcu/config/llConfig/USART-STM32F1xx_DefMapping.xml
```

里面写着：

```xml
<Item Name="WordLength" Value="WORDLENGTH_8B"/>
<Item Name="Parity"     Value="PARITY_NONE"/>
<Item Name="StopBits"   Value="STOPBITS_1"/>
<Item Name="HwFlowCtl"  Value="UART_HWCONTROL_NONE"/>
```

**F1 用的是不带 `UART_` 前缀的短名。** 而且注意最后一行——
`HwFlowCtl` **带**前缀。**同一个文件里前后不一致**，
所以只能查表，**绝不能按别的系列类推**。

> 这条教训已经写进全局 `CLAUDE.md`：
> **「键写进 .ioc」「roundtrip 通过」「能编译」都不等于「取值合法」。**

---

## 四、手写的部分

### 4.1 先定接口

```c
error_t  bsp_serial_init(void);                                  /* 武装接收中断 */
uint32_t bsp_serial_write(const uint8_t *data, uint32_t len);    /* 非阻塞 */
uint32_t bsp_serial_read(uint8_t *out, uint32_t max);            /* 非阻塞 */
uint32_t bsp_serial_rx_available(void);
uint32_t bsp_serial_tx_free(void);
uint32_t bsp_serial_rx_dropped(void);                            /* 诊断量 */
error_t  bsp_serial_flush(uint32_t timeout_ms);                  /* 唯一的阻塞函数 */
```

几个刻意的选择：

- **收发的返回类型不同**：`write` 返回实际写进去的字节数（可能小于 `len`），
  `read` 返回实际取到的字节数。两者都是「有多少算多少」，**绝不等待**。
- **不提供「读一行」之类的接口**。串口是**字节流**：一次 `read` 可能只拿到
  半个报文，也可能拿到两条半。上层要自己处理这个事实，
  而不是让驱动假装「一次调用 = 一条消息」。
- **`flush` 是唯一的阻塞函数**，且文档里明确写了「只应在收尾/调试场合使用」。

### 4.2 环形缓冲：为什么可以不加锁

这是整个驱动最关键的设计。先看数据流向：

```
发送：  主循环 ──写──> s_tx_head
        中断   ──写──> s_tx_tail

接收：  中断   ──写──> s_rx_head
        主循环 ──写──> s_rx_tail
```

**每个索引都只有一个写者**——这就是 SPSC（单生产者单消费者）模型。
两个写者写的是**不同的变量**，不存在「同时改一个变量」的竞态。

而 Cortex-M3 对 16 位对齐变量的读写是**单条指令**，不会读到「改了一半」的值。
所以**绝大部分代码不需要关中断**——这正是它能做到非阻塞的基础。

**唯一的顺序要求**：必须**先把数据写进缓冲、再把 head 推上去**。
否则中断可能读到 head 已更新、内容还是旧的位置。代码里用 `__DMB()` 明确这个顺序。

**两个必须遵守的约定**：

| 约定 | 原因 |
| --- | --- |
| 缓冲区大小必须是 **2 的幂** | 索引回绕用 `& (SIZE-1)`（一条指令），不是 `% SIZE`（除法） |
| **必须空出一格** | 不留的话 `head == tail` 同时意味着「空」和「满」，无法区分 |

两条都有编译期保护：

```c
#if (BSP_SERIAL_TX_BUF_SIZE & (BSP_SERIAL_TX_BUF_SIZE - 1U)) != 0U
#error "BSP_SERIAL_TX_BUF_SIZE 必须是 2 的幂"
#endif
```

### 4.3 状态变量

```c
static uint8_t  s_tx_buf[256];
static volatile uint16_t s_tx_head;    /* 主循环写 */
static volatile uint16_t s_tx_tail;    /* 中断写 */
static volatile uint16_t s_tx_len;
static volatile bool     s_tx_busy;

static uint8_t  s_rx_buf[256];
static volatile uint16_t s_rx_head;    /* 中断写 */
static volatile uint16_t s_rx_tail;    /* 主循环写 */
static uint8_t  s_rx_byte;             /* HAL 单字节接收的落点 */
static volatile uint32_t s_rx_dropped;
static bool     s_ready;
```

**`s_tx_len` 为什么需要**？`HAL_UART_Transmit_IT()` 只拿走一个指针，
完成回调里**不告诉我们是哪一笔**——得自己记住长度，才能推进 `tail`。

### 4.4 唯一需要关中断的地方

```c
static void serial_tx_kick(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (!s_tx_busy) {
        const uint16_t head = s_tx_head, tail = s_tx_tail;
        if (head != tail) {
            const uint16_t n = (head > tail) ? (uint16_t)(head - tail)
                                             : (uint16_t)(BSP_SERIAL_TX_BUF_SIZE - tail);
            s_tx_len = n;  s_tx_busy = true;
            if (HAL_UART_Transmit_IT(&huart1, &s_tx_buf[tail], n) != HAL_OK) {
                s_tx_busy = false;  s_tx_len = 0;      /* 启动失败要退回去，否则发不出去 */
            }
        }
    }

    __set_PRIMASK(primask);
}
```

**这里是 check-then-act**，而 `s_tx_busy` 同时被中断改。不加保护的致命交错：

1. 主循环读到 `s_tx_busy == false`，判断「空闲」，准备启动；
2. **就在这一瞬间**，上一笔的 TxCplt 中断到达，回调把 `s_tx_busy` 清 0，
   并自己启动了一笔新的发送；
3. 主循环恢复执行，**也去启动一笔** → 两次 `HAL_UART_Transmit_IT` 并发，
   内部的发送指针互相覆盖，发出去的数据就乱了。

窗口只有几条指令，**极难复现**。关中断的时长就是这几条语句 + 一次 HAL 调用
（它不搬数据，只记指针、使能 TXE 中断），量级是微秒，对 1 ms 节拍毫无影响。

**恢复用 `__set_PRIMASK(primask)` 而不是无脑 `__enable_irq()`**：
这样即使本函数被从已经关中断的上下文里调用，也不会意外开中断。

### 4.5 发送路径

```c
uint32_t bsp_serial_write(const uint8_t *data, uint32_t len)
{
    uint16_t head = s_tx_head;         /* 只有本函数写 head，先读到本地 */
    const uint16_t tail = s_tx_tail;   /* 中断写，只读一次保证整段一致 */
    uint32_t written = 0;

    while (written < len) {
        const uint16_t next = (uint16_t)((head + 1U) & SERIAL_TX_MASK);
        if (next == tail) break;       /* 满：能放多少放多少 */
        s_tx_buf[head] = data[written];
        head = next;  written++;
    }
    if (written > 0) { __DMB(); s_tx_head = head; serial_tx_kick(); }
    return written;
}
```

完成回调推进 `tail` 并接着发下一段：

```c
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;
    s_tx_tail = (uint16_t)((s_tx_tail + s_tx_len) & SERIAL_TX_MASK);
    s_tx_len = 0;  s_tx_busy = false;
    serial_tx_kick();                  /* 缓冲里可能还有下一段 */
}
```

**`tx_kick` 里那段三元表达式**：

```c
const uint16_t n = (head > tail) ? (head - tail) : (BSP_SERIAL_TX_BUF_SIZE - tail);
```

它算的是「从 tail 到 head，或到缓冲区末尾」这一段**连续**数据的长度。
**只发连续的一段，不跨越回绕点**——因为 HAL 拿到的是个普通指针，
不支持环形。跨回绕的部分留给下一次回调再发。

### 4.6 接收路径与两个必须的回调

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    const uint16_t next = (uint16_t)((s_rx_head + 1U) & SERIAL_RX_MASK);
    if (next == s_rx_tail) {
        s_rx_dropped++;                /* 满：丢新来的，保留先到的 */
    } else {
        s_rx_buf[s_rx_head] = s_rx_byte;
        __DMB();
        s_rx_head = next;
    }

    (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);   /* ← 必须 */
}
```

**缓冲满时丢「新来的」而不是丢「旧的」**：这样上层至少还能看到一条完整报文的
前半段，比丢掉中间要好。

**结尾那句重新武装是绝对必须的**（原因见 2.3 节）。漏了的现象是
「刚上电那一瞬间有数据，之后永远没有」，而且不报任何错。

**第二个回调更容易漏**：

```c
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;
    __HAL_UART_CLEAR_PEFLAG(&huart1);                    /* 清残留标志 */
    (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);   /* 重新武装 */
}
```

为什么不能省，要去 `stm32f1xx_hal_uart.c` 的 `HAL_UART_IRQHandler()` 里看。
它把错误分成**两类**，处理方式完全不同：

| 类别 | 触发条件 | HAL 的行为 |
| --- | --- | --- |
| **阻塞类** | `ORE`（接收溢出），或启用了接收 DMA | 源码注释 `Blocking error : transfer is aborted`，调用 `UART_EndRxTransfer()` **中止接收、关掉接收中断** |
| **非阻塞类** | 只有 `PE` / `NE` / `FE` | 源码注释 `Non Blocking error : transfer could go on.`，**不中止**，只通知 |

**阻塞类不重新武装 = 永久只能发不能收。** 本回调对两类都是
「清标志 + 重新武装」：前者必须，后者冗余但无害（HAL 若发现已在接收中会返回
`HAL_BUSY`，忽略即可）。

实际最容易撞上的是 `ORE`：主循环太久没来取数据、或对端热插拔时线上一串脉冲，
接收移位寄存器里的数据还没被读走就被新数据覆盖。

### 4.7 printf 重定向

```c
int _write(int file, char *ptr, int len)
{
    (void)file;
    if ((ptr == NULL) || (len <= 0)) return 0;
    return (int)bsp_serial_write((const uint8_t *)ptr, (uint32_t)len);
}
```

newlib 的 `printf` 不直接碰硬件，而是把格式化好的字节交给 `_write()`。
工具链默认提供一个**什么都不做**的 `_write`（来自 `--specs=nosys.specs`），
我们给出强定义把它覆盖掉。

**三条约束**（都写在头文件里了）：

1. **缓冲满时 printf 会截断输出**（返回短计数），不会阻塞等待。这是刻意的。
2. **`%f` 默认不可用**——链接选项没开 `-u _printf_float`，开了会多占约 6 KB Flash。
   要打印浮点可以先自行放大成整数再打。
3. **绝对不要在中断里调 printf**：它不重入，而且格式化耗时可观。

---

## 五、怎么测试

### 5.1 为什么「编译通过 + 烧录成功」不算验证

因为这套工具链最典型的失败模式就是**全绿但硬件没反应**。
本模块就真实发生过一次：配置写错了四层检查全绿，串口**还真的能通**（见 3.4 节）。

所以验证要有**独立的第二条通道**：串口说的事，用 SWD 去核对。

### 5.2 第一层：先确认「验证手段」存在

在写代码之前就做——先确认 PC 上能看到那个串口：

```bash
python -c "import serial.tools.list_ports as lp; [print(p.device,'|',p.description) for p in lp.comports()]"
```

```
COM5/COM3/COM6/COM4 | 蓝牙链接上的标准串行      ← 无关
COM7                | USB-SERIAL CH340          ← 就是它
```

**有 COM7 意味着验证能做端到端**，而不是「编译通过」这种假验证。

### 5.3 第二层：端到端收发

烧录后开 COM7，同时验证三件事：

```python
# 1. 只读，看 MCU 主动发的东西（验证 TX）
buf = read_for(3.0 seconds)

# 2. 发一个标记，看能不能原样收回来（验证 RX + 回显）
write(b"PING-9f3c\r\n")
back = read_for(2.0 seconds)
assert b"PING-9f3c" in back
```

结果：

```
  < PID_Pendulum tick=133 rp1=1947 rp2=2279 rp3=1100 rp4=1743
  < PID_Pendulum tick=134 rp1=1956 rp2=2280 rp3=1098 rp4=1750
  ...
  < PING-9f3c
TX (MCU -> PC) : PASS
RX (PC -> MCU) : PASS
```

**顺带的一条证据**：`rp1~rp4` 的值在轻微跳动。这说明是**真实的 ADC 噪声**，
而不是某个死值——如果读出来四个恒定不变的数，反而要怀疑。

那一行 `tick=...` 是用 `printf` 拼的，所以**同时验证了 printf 重定向**。

### 5.4 第三层：SWD 独立验证计时

第一次跑完发现数字不对：**3 秒收到 70 行**，而按 500 ms 一行只该有 6 行。差了 12 倍。

**没有立刻去改代码**——先要确定是固件的问题还是测量的问题。
换一条**完全不依赖串口**的通道：直接读 RAM 里 HAL 的毫秒计数 `uwTick`。

```bash
arm-none-eabi-nm build/Debug/PID_Pendulum.elf | grep uwTick
# 20000084 B uwTick

STM32_Programmer_CLI -c port=SWD mode=hotplug -r32 0x20000084 4
```

`mode=hotplug` 是**不复位**连接，所以读到的是正在运行的程序的实时值。
隔 20 秒读两次：

```
uwTick 增量 = 20293 ms
墙钟   增量 = 20240 ms
比值 = 1.003
```

**SysTick 完全正常，`HAL_GetTick()` 没问题。** 所以 500 ms 的判断逻辑是对的，
问题在测量侧。

### 5.5 一次差点误判：缓冲区交付时机

回头核对数字，发现其实是**自洽的**：

```
4096 字节 ÷ 56 字节/行 = 73 行
按 2 行/秒要 36.5 秒攒满 —— 正好等于脚本里那个异常的 t=37.9s
```

**原因**：pyserial 的 `read(4096)` 会**阻塞攒满一整块**才返回，
我的墙钟时间戳全打在了缓冲区被一次性交付的那一瞬间，全是同一个值。

换成 `readline()` 逐行读：

```
收 12 行 / 6.0 s
  t=0.319  PID_Pendulum tick=479 ...
  t=0.857  PID_Pendulum tick=480 ...
平均间隔 = 0.553 s
```

**12 行 / 6.0 秒 = 每行正好 500 ms。固件没问题，是我的测量脚本错了。**

> 这一条的价值在于：**「验证手段」本身也要被验证**。
> 如果当时直接去改 500 ms 的逻辑，就会把一段正确的代码改坏。

### 5.6 第四层：读运行时诊断量

接收缓冲有没有溢出？驱动里留了个计数器 `s_rx_dropped`，
用 SWD 直接读它（同样不依赖串口）：

```bash
arm-none-eabi-nm build/Debug/PID_Pendulum.elf | grep s_rx_dropped
# 200007f0 b s_rx_dropped

STM32_Programmer_CLI -c port=SWD mode=hotplug -r32 0x200007f0 4
# 0x200007F0 : 00000000        ← 零丢弃
```

这个量长期不为 0 就说明**主循环取数据太慢**，
或者上层协议的单条报文超过了 `BSP_SERIAL_RX_BUF_SIZE`。

### 5.7 没验证到什么

诚实起见：

- **没有测过错误回调的实际触发**。`HAL_UART_ErrorCallback` 那条路径靠的是
  读 HAL 源码确认其必要性，没有真去制造一次 `ORE`（比如故意让主循环忙很久）。
- **没有测过 `bsp_serial_flush()` 的超时分支**。
- **没有测过 RX 缓冲满时的丢弃行为**（`s_rx_dropped` 一直是 0）。
- **没有测过 115200 以外的波特率**，也没上示波器量实际位宽。

前三条要制造出来都不难（在应用层加个「忙等 100 ms」或「连发 10 KB」即可），
留作后续如果需要时的回归项。

---

## 六、设计取舍一览

| 决定 | 换来什么 | 代价 |
| --- | --- | --- |
| **收发都中断 + 环形缓冲** | 主循环一次都不用等 | 代码约 300 行；`write` 返回时数据可能还没发出去 |
| **SPSC 无锁** | 绝大部分代码不关中断 | 缓冲区大小必须是 2 的幂，且必须空出一格 |
| `tx_kick` 关中断 | 消除 check-then-act 竞态 | 几微秒的关中断（对 1 ms 节拍可忽略） |
| **不提供「读一行」接口** | 不欺骗上层，字节流就是字节流 | 上层要自己处理半包/粘包 |
| **缓冲区满时丢新数据** | 保留先到数据的完整性 | 突发流量下会丢（但 `s_rx_dropped` 可见） |
| **printf 走缓冲** | printf 不阻塞 | 输出可能被截断；+8 KB Flash |
| 不开 `-u _printf_float` | 省约 6 KB Flash | `%f` 不能用 |
| USART1 优先级取 2 | 1 ms 节拍可以打断串口 ISR | 串口字节延迟略增（有 87 µs 余量） |
| **`flush()` 是唯一阻塞函数** | 收尾时能确保日志发出去 | 有被误用的风险（文档里标了） |
| 放 `bsp/` 而非 `driver/bus/` | 不做无用抽象，与 key/pot 一致 | 直接依赖 HAL（换芯片要改） |

整个模块约 300 行，其中真正的「逻辑」不到 100 行，其余都是**为正确性付出的成本**——
环形缓冲的空格约定、发行顺序的 `__DMB()`、`tx_kick` 的临界区、两个回调的重新武装、
错误回调的分类处理。

---

## 附：这次踩过的坑

按"踩到的顺序"记录，都是实测：

| 坑 | 现象 | 根因 |
| --- | --- | --- |
| **F1 的参数取值格式和别的系列不同** | 四层检查全绿、串口还真能通，只有日志里一句 `has invalid value` | F1 用短名 `WORDLENGTH_8B`，不是 `UART_WORDLENGTH_8B`；且同文件里 `HwFlowCtl` 却带前缀 |
| **IP not-ready 时生成「半成品」初始化函数** | `MX_USART1_UART_Init()` 少三个字段 | 参数非法让 CubeMX 退化成部分模板；零值**碰巧**等于 8N1 默认，错误被掩盖 |
| **`roundtrip` 通过 ≠ 取值合法** | 往返比对报「原样保留」 | 它只比「键在不在、有没有被改写」，不比「值合不合法」 |
| **`BaudRate 115200` 不落盘** | 键没出现在 `.ioc`，以为写错了 | 115200 就是默认值，**CubeMX 不写等于默认值的键**（改成 9600 就会落盘） |
| **CMake 源文件清单不更新** | `undefined reference to 'HAL_UART_Init'` | CubeMX 不更新 `cmake/stm32cubemx/CMakeLists.txt`，要手工补 |
| **`apply` 的非法键检查看不见本次改动** | 写错了但 apply 不报 | 它读的是「写入**之前**那次加载」的日志，CubeMX 还没见过新键 |
| **pyserial `read(4096)` 的交付时机** | 3 秒「收到」70 行，差点去改正确的代码 | 它阻塞攒满一整块才返回，墙钟时间戳全落在同一瞬间 |
| **误以为「出错就中止接收」** | 注释写宽了 | 只有 `ORE`（或 DMA）才 `UART_EndRxTransfer()`；单独的 `PE`/`NE`/`FE` 走 `Non Blocking error` 分支 |

**第一条和第二条尤其值得记**：一个错误的值，因为「零初始化恰好等于默认值」，
产出了**完全正确**的硬件行为。这让它躲过了：
命令返回值、文件内容、往返比对、编译链接、乃至**实际通信**五道检查。

唯一能发现它的是 **CubeMX 的日志原话**，以及**去生成的代码里数一数字段够不够**。
