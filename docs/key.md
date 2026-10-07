# 按键（BSP Key）实现详解

本文逐层拆解本仓库 4 路按键模块的实现：电气原理、CubeMX 配置、消抖算法、跨上下文的事件机制，以及怎么在没有摄像头的情况下验证它。

**面向读者**：嵌入式初学者。假设你会 C（结构体、指针、中断的概念），但不要求懂 GPIO 寄存器细节。

**配套代码**：

| 文件 | 层 | 职责 |
| --- | --- | --- |
| [PID_Pendulum.ioc](../PID_Pendulum.ioc) | CubeMX | 4 个引脚的声明（上拉输入 + 标签） |
| [Core/Inc/main.h](../Core/Inc/main.h) | CubeMX 生成 | `KEY1_Pin` / `KEY1_GPIO_Port` 宏 |
| [Core/Src/main.c](../Core/Src/main.c) | CubeMX 生成 | `MX_GPIO_Init()` 里真正的引脚配置 |
| [bsp/key/bsp_key.h](../bsp/key/bsp_key.h) · [.c](../bsp/key/bsp_key.c) | BSP | 消抖状态机、事件上报 |
| [bsp/tick/](../bsp/tick/) | BSP | TIM1 的 1 ms 节拍，按键挂在它上面 |
| [Core/Src/main.c](../Core/Src/main.c) | 应用 | 事件轮询 + 屏幕显示 |

---

## 目录

- [一、要解决什么问题](#一要解决什么问题)
- [二、电气层：为什么必须上拉](#二电气层为什么必须上拉)
- [三、CubeMX 配置了什么](#三cubemx-配置了什么)
- [四、手写的部分](#四手写的部分)
- [五、怎么测试](#五怎么测试)
- [六、设计取舍一览](#六设计取舍一览)

---

## 一、要解决什么问题

按键看起来最简单，实际上要处理三件事，**每一件都有陷阱**：

| 需求 | 陷阱 |
| --- | --- |
| ① 读出「是否按下」 | 引脚不按时是**悬空**的，读到的电平不确定 |
| ② 把弹跳过滤掉 | 机械触点闭合瞬间会**抖动**，一次按下会被读成十几次 |
| ③ 让应用知道「发生了什么」 | 只给「当前是否按着」，应用会漏掉快速按放 |

三个需求对应三段设计，下文按这个顺序展开。

---

## 二、电气层：为什么必须上拉

按键本质是一个开关。本仓库的接线：

```
       3.3V
         │
       ┌─┴─┐  引脚内部上拉电阻（约 30~50 kΩ）
       │   │
       └─┬─┘
         │
    PB10 ├──────────────┐
         │              │
   （引脚输入）       ┌──┴──┐  按键
                    │     │
                    └──┬──┘
                       │
                      GND
```

**不按时**：开关断开，引脚**什么都不接**——这叫**悬空**（floating）。悬空引脚的电平是不确定的，会跟着周围的电磁干扰乱跳（CMOS 输入阻抗极高，一段导线就相当于天线）。所以必须在引脚上拴一个上拉电阻，让它在空闲时有确定的高电平。

**按下时**：开关把引脚直接接到 GND，电流从上拉电阻流过（3.3 V / 47 kΩ ≈ 0.07 mA），引脚被强制拉低。

结论：**不按 = 高，按下 = 低**，即「低电平有效」。

> **一个容易搞混的地方**：STM32 在**输入模式**下，「上拉还是下拉」是由 **ODR 寄存器**决定的，不是 CRH。
> `CRH` 里配 `MODE=00, CNF=10` 只说明「这是带上下拉的输入」，具体朝哪边拉，看 `ODR` 对应位是 1（上拉）还是 0（下拉）。
> 这就是为什么后文验证时要**同时**看 `CRH`（`0x8`）和 `IDR`（读到高）才能确认上拉真的生效。

这个结论直接决定了两处代码：

```c
/* bsp_key.c */
#define BSP_KEY_ACTIVE_LEVEL    (GPIO_PIN_RESET)   /* 按下读到低 */

/* 把「引脚电平」翻译成「是否按下」 */
return HAL_GPIO_ReadPin(hw->port, hw->pin) == BSP_KEY_ACTIVE_LEVEL;
```

**所有上层代码只谈「按下/松开」，不谈「高/低」。** 如果哪天接线反了（按下接 3.3 V），只需改这一行常量 + 把 `.ioc` 里改成下拉，上层一行都不用动。

---

## 三、CubeMX 配置了什么

### 3.1 `.ioc` 里的四个引脚声明

```ini
PB10.Signal=GPIO_Input
PB10.GPIO_Label=KEY1
PB10.GPIO_PuPd=GPIO_PULLUP
PB10.Locked=true
PB10.GPIOParameters=GPIO_PuPd,GPIO_Label
```

PA11 / PA12 / PB11 是同样的五条（标签依次 KEY2 / KEY3 / KEY4）。

| 键 | 含义 |
| --- | --- |
| `.Signal=GPIO_Input` | 该引脚当普通 GPIO 输入用（不是复用功能） |
| `.GPIO_Label=KEY1` | 起名，会生成 `KEY1_Pin` / `KEY1_GPIO_Port` 宏 |
| `.GPIO_PuPd=GPIO_PULLUP` | 上拉——即第二节那条电气要求 |
| `.Locked=true` | 锁定分配，防止以后误改 |
| `.GPIOParameters=...` | **白名单**，见下 |

> ⚠️ **白名单是 CubeMX 最阴险的机制**：它用逗号分隔的名单决定哪些参数才算数。
> 写了 `GPIO_PuPd=GPIO_PULLUP` 却没把 `GPIO_PuPd` 列进 `GPIOParameters`，
> CubeMX 会**完全无视**这个参数——不报错、不警告，生成的代码里就是没有上拉。

### 3.2 生成了什么

**① `Core/Inc/main.h` —— 引脚宏**

```c
#define KEY1_Pin GPIO_PIN_10
#define KEY1_GPIO_Port GPIOB
#define KEY2_Pin GPIO_PIN_11
#define KEY2_GPIO_Port GPIOB
#define KEY3_Pin GPIO_PIN_11
#define KEY3_GPIO_Port GPIOA
#define KEY4_Pin GPIO_PIN_12
#define KEY4_GPIO_Port GPIOA
```

**② `Core/Src/main.c` 的 `MX_GPIO_Init()` —— 真正的寄存器配置**

```c
/*Configure GPIO pins : KEY1_Pin KEY2_Pin */
GPIO_InitStruct.Pin = KEY1_Pin|KEY2_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_PULLUP;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/*Configure GPIO pins : KEY3_Pin KEY4_Pin */
GPIO_InitStruct.Pin = KEY3_Pin|KEY4_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_PULLUP;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
```

CubeMX 把同一端口的两个脚**合并成一次 `HAL_GPIO_Init`**（配置参数相同）。GPIO 端口时钟也在 `MX_GPIO_Init` 开头使能了。

**③ 于是 `bsp_key.c` 里没有任何硬件配置代码**，它只做逻辑。

### 3.3 为什么按键的引脚交给 CubeMX，而 OLED 的不是

本仓库两种做法并存，是有理由的：

| | 归谁 | 理由 |
| --- | --- | --- |
| **按键** | CubeMX | 「输入 + 上拉」只是**本板的接线选择**，没有协议含义 |
| **OLED 的软件 I2C** | 总线驱动自己配 | 「开漏 + 上拉」是 **I2C 协议的一部分**，总线驱动必须自己保证，也要能脱离 CubeMX 复用到别的引脚 |

「交给 CubeMX」还有一层实际收益：**防止将来抢引脚**。

PB10/PB11 同时也可能是 USART3 或 I2C2 的引脚，PA11 是 TIM1_CH4，PA12 是 TIM1_ETR。如果 `.ioc` 不知道它们已被按键占用，以后加 USART3 时会**静默抢走**这两个脚——`MX_GPIO_Init` 把它们配成复用推挽，随后 `bsp_key_init` 又改回输入，**串口就死了，而且不报错**。现在这四个脚在 `.ioc` 里已被占用，CubeMX 会在配置阶段就报冲突。

---

## 四、手写的部分

### 4.1 先定接口

按正常开发顺序，**先想清楚「别人怎么用我」**，再写实现：

```c
/* bsp_key.h */
typedef enum {
    KEY_ID_K1 = 0, KEY_ID_K2, KEY_ID_K3, KEY_ID_K4,
    KEY_ID_COUNT,               /* 按键总数，也当遍历上界 */
} key_id_t;

typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_PRESS,            /* 按下 */
    KEY_EVENT_RELEASE,          /* 松开 */
} key_event_t;

error_t     bsp_key_init(void);
key_event_t bsp_key_take_event(key_id_t id);   /* 取走事件（读取即清除） */
bool        bsp_key_is_pressed(key_id_t id);   /* 当前是否按着 */
void        bsp_key_scan_isr(void);            /* 内部用，由节拍调用 */
```

**关键设计：同时提供「电平」和「边沿」两种查询。** 这不是冗余，是两类问题的不同答案：

| 你想知道 | 用哪个 | 例子 |
| --- | --- | --- |
| 「现在按着没有」 | `is_pressed` | 长按加速、组合键 |
| 「刚刚发生了什么」 | `take_event` | 计次、翻页、触发动作 |

只给电平的话，应用得自己「记住上次状态」来推断边沿——每个应用都要重复写一遍，而且**会漏事件**（见 4.7）。

### 4.2 数据结构

```c
/** 引脚的物理位置，与 key_id_t 一一对应 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} key_hw_t;

/** 单个按键的消抖状态 */
typedef struct {
    bool stable;                    /* 消抖后的稳定状态（true = 按下） */
    bool last_raw;                  /* 上一次原始采样（true = 按下） */
    uint16_t same_count;            /* 连续相同采样的次数 */
    volatile bool press_pending;    /* 有未取走的「按下」事件 */
    volatile bool release_pending;  /* 有未取走的「松开」事件 */
} key_state_t;
```

两个文件级静态数组：

```c
static const key_hw_t s_key_hw[KEY_ID_COUNT] = {
    { KEY1_GPIO_Port, KEY1_Pin },      /* 用 CubeMX 的宏，不硬编码 GPIOB */
    { KEY2_GPIO_Port, KEY2_Pin },
    { KEY3_GPIO_Port, KEY3_Pin },
    { KEY4_GPIO_Port, KEY4_Pin },
};

static key_state_t s_key_state[KEY_ID_COUNT];
```

**为什么要有 `key_hw_t` 这张表？** 四个键的代码完全一样，只有「哪个端口、哪个位」不同。抽成表之后扫描就是一个循环：

```c
for (uint8_t i = 0; i < KEY_ID_COUNT; i++) {
    key_scan_one(&s_key_hw[i], &s_key_state[i]);
}
```

加第五个按键只需在 `.ioc` 加一行、表里加一行、枚举加一项。

**关于 `volatile`**：只有 `press_pending` / `release_pending` 加了——它们**由中断写、由主循环读**，必须防止编译器把值缓存进寄存器。`stable` / `last_raw` / `same_count` 只在中断上下文里碰，不需要。乱加 `volatile` 会拖慢代码，这是精准使用。

### 4.3 消抖算法

#### 为什么不能直接读

机械触点的金属片在闭合瞬间会**弹跳**几毫秒：

```
理想情况：   ────────┐
                     └────────      一次干净的下落

实际情况：   ──┐ ┌─┐ ┌──┐ ┌──────
               └─┘ └─┘  └─┘        十几个毛刺
             ← 弹跳 1~5 ms →
```

在这几毫秒内不断采样会读到一串乱七八糟的电平。**一次按下被当成十几次按下**——计数值乱跳、菜单翻好几页。

#### 消抖原理

标准做法：**定期采样 + 连续 N 次一致才认定电平真的变了**。

巧妙之处在于——**毛刺攒不够 N 次**。一旦电平又跳，计数就清零重来。只有真正稳定下来的电平，才可能连续 N 次保持一致。

#### 代码逐步推演

```c
static void key_scan_one(const key_hw_t *hw, key_state_t *st)
{
    const bool raw = key_read_pressed(hw);      /* ① 采一次 */

    if (raw != st->last_raw) {
        st->last_raw   = raw;                   /* ② 电平变了，重新计时 */
        st->same_count = 0U;
        return;
    }

    if (st->same_count < BSP_KEY_DEBOUNCE_TICKS) {
        st->same_count++;                       /* ③ 和上次一样，累加 */

        if ((st->same_count >= BSP_KEY_DEBOUNCE_TICKS) && (st->stable != raw)) {
            st->stable = raw;                   /* ④ 连续 20 次，承认翻转 */
            if (raw) { st->press_pending   = true; }
            else     { st->release_pending = true; }
        }
    }
}
```

`BSP_KEY_DEBOUNCE_TICKS = 20`（20 ms ÷ 1 ms 每拍）。**代入时间轴**看一遍，假设按键在 t=0 开始抖：

| 时刻 | `raw` | 和 `last_raw` | 动作 |
| --- | --- | --- | --- |
| t=0 | 1 | 变了 | `last_raw=1`, `same_count=0` |
| t=1 | 0 | 变了（弹回） | `last_raw=0`, `same_count=0` |
| t=2 | 1 | 变了 | `same_count=0` |
| t=3 | 1 | 一样 | `same_count=1` |
| t=4 | 0 | 变了（又弹） | `same_count=0` ← **功亏一篑** |
| t=5 | 1 | 变了 | `same_count=0` |
| … | 抖到 t≈5 ms 稳定为 1 | | |
| t=25 | 1 | 一样 | `same_count` 攒够 20 → **`stable=1`，产生按下事件** |

所以一次按下会在**抖动结束后 20 ms** 被确认。人对 20 ms 无感（要 100 ms 才开始觉得慢），对 5 ms 的抖动有充分余量。

**为什么是 20 而不是 5？** 余量。便宜按键的弹跳能到 5~10 ms，选 20 稳妥；这个值只影响确认延迟，不影响功能。

### 4.4 为什么采样必须放在 1 ms 中断里

这是整套设计里**最关键的决定**。

如果放在主循环里采样：

```c
while (1) {
    bsp_key_poll();          /* ← 假设放这里 */
    oled_refresh();          /* 刷一次屏 30 ms */
}
```

刷一次屏要 30 ms（1024 字节走 I2C）。这 30 ms 内循环一次都没跑，按键**一次都没被采样**。而人按一下键大约 50~200 ms——**如果这 50 ms 整段落在某次刷屏期间，这次按下就被完全漏掉**。现象是「有时候按键没反应」，随机、偶发，最难查。

放到 1 ms 定时中断里之后：

```
时间轴（每格 1 ms）：
主循环:   [====刷屏 30ms====][==刷屏==][按键处理]
节拍中断:  │││││││││││││││││││││││││││││││││   ← 每 1 ms 雷打不动
           ↓ 采样间隔恒定，与主循环在忙什么无关
```

**采样间隔恒定**才是消抖成立的前提。消抖算的是「连续 20 次」，如果采样间隔忽长忽短，「20 次」对应的物理时间就会从 20 ms 变成 300 ms。

### 4.5 挂到节拍上

按键模块自己不碰定时器，只是「订阅」节拍：

```c
error_t bsp_key_init(void)
{
    ...
    return bsp_tick_register(bsp_key_scan_isr);   /* ← 订阅 */
}
```

`bsp_tick` 那一层负责 TIM1 的 1 ms 中断，并逐个调用订阅者。

**为什么分两层？** 因为 1 ms 节拍是**系统基础设施**，不是按键的私产。以后的 PID 控制环、传感器采样都要挂上来。如果 TIM1 被按键独占，第二个需要节拍的模块就只能去抢别的定时器——不必要的返工。

### 4.6 初始化：为什么用「当前电平」做初值

```c
const bool raw = key_read_pressed(&s_key_hw[i]);   /* ← 读实际电平 */

s_key_state[i].stable     = raw;
s_key_state[i].last_raw   = raw;
s_key_state[i].same_count = BSP_KEY_DEBOUNCE_TICKS;   /* 已视为稳定 */
```

如果偷懒把 `stable` 和 `last_raw` 都初始化成 `false`（松开），而开机时用户正好**按着**某个键：

```
t=0    初始化: stable=false, last_raw=false
t=1    第一次采样: raw=true ≠ last_raw(false)  → last_raw=true, count=0
...
t=21   count 攒够 20，stable(false) != raw(true) → 产生「按下」事件
```

**开机就凭空多计一次数。** 用实际电平做初值就没这个问题。

`same_count` 直接置成满值（20）同理——表示「这个状态我已认为是稳定的」，避免开机瞬间重走一遍确认流程。

### 4.7 事件为什么用「边沿」，不用「电平」

假设只有 `is_pressed()`，应用想统计按了几次：

```c
bool last = false;
while (1) {
    bool now = bsp_key_is_pressed(KEY_ID_K1);
    if (now && !last) { count++; }     /* 检测上升沿 */
    last = now;
}
```

**这套写法会漏事件。** 如果用户在两次轮询之间完成了一次快速按放（按下和松开都在那段时间里），主循环看到的 `now` 前后都是 false，**这一次按放完全不可见**。

而中断每 1 ms 看一次，按放至少要 2 个消抖周期（40 ms）才产生一对事件，中断绝不会漏掉。

**结论：边沿必须由「看得足够勤」的那一层（中断）捕获并暂存，等主循环来取。**

这就是 `press_pending` / `release_pending` 的作用——**中断打标记，主循环取走**：

```
中断侧：  按下确认 → st->press_pending = true      （只置位，不做别的）
主循环侧：take_event() → 读到 true，清除，返回 PRESS
```

中断里**只置位，不做任何业务**，符合「快进快出」：

```c
void bsp_key_scan_isr(void)
{
    for (...) { key_scan_one(...); }   /* 4 次读引脚 + 几个比较自增，约 2~3 µs */
}
```

没有 `printf`、没有刷屏、没有阻塞。1 ms 里只占约 0.3%。

**两个标志分开，是为了不丢事件。** 如果只有一个「有事件」标志，「按下然后松开」就只剩一个标志，主循环只能看到一次。现在两个都在，`take_event` 先返回 PRESS，下一次调用再返回 RELEASE，一个都不丢。

### 4.8 临界区：一个「一天出现一次」的竞态

`take_event` 对两个标志做的是「**判断 + 清除**」两步操作，而这两个标志由**中断**写：

```c
const uint32_t primask = __get_PRIMASK();
__disable_irq();                        /* ← 关中断 */

if (st->press_pending) {
    st->press_pending = false;
    ev = KEY_EVENT_PRESS;
} else if (st->release_pending) { ... }

__set_PRIMASK(primask);                 /* ← 恢复之前的状态 */
```

**不加保护会怎样**：

```
主循环:  读 press_pending → 是 true
中断:                         ← 就在这一刻，用户又按了一下，中断把 press_pending 置回 true
主循环:  press_pending = false   ← 把刚才中断置的标记清掉了！
```

**那一次按下就丢了。** 这种 bug 的恶劣之处在于：时间窗口只有几条指令，一天可能只出现一次，**完全无法复现**。

关中断只持续 3 条指令（几十纳秒），对 1 ms 节拍毫无影响。

**注意是「恢复」而不是「无条件开中断」**：

```c
const uint32_t primask = __get_PRIMASK();   /* 记住进函数前中断是开是关 */
...
__set_PRIMASK(primask);                     /* 原样恢复 */
```

如果写成无脑的 `__enable_irq()`，当 `take_event` 被一个**已经关着中断**的上下文调用时，它会把中断意外打开——这是嵌套临界区里的经典错误。

**而 `is_pressed()` 不需要关中断**，因为 `bool` 是单字节：

```c
return s_key_state[id].stable;   /* Cortex-M3 对字节的读写是原子的 */
```

如果它是个 32 位变量、或要读多个字段组成一个结果，那就必须关中断了。

### 4.9 应用层怎么用

```c
static void app_poll_keys(void)
{
  bool dirty = false;

  for (uint8_t i = 0U; i < (uint8_t)KEY_ID_COUNT; i++) {
    const key_id_t id = (key_id_t)i;
    key_event_t    ev;

    while ((ev = bsp_key_take_event(id)) != KEY_EVENT_NONE) {   /* ← 取干净 */
      if (ev == KEY_EVENT_PRESS) {
        s_press_count[i]++;      /* 计数是「应用策略」，不属于驱动 */
        s_last_down = i;
      } else {
        s_last_up = i;
      }
      dirty = true;
    }
  }

  if (dirty) { oled_redraw(); }   /* 有变化才重画屏幕 */
}
```

两个要点：

**① 用 `while` 而不是 `if`。** 一个键可能同时攒着按下和松开两个事件（主循环正在刷屏的 30 ms 里用户完成了一次按放），要一次取干净。

**② 计数放在应用层，不放进驱动。** 驱动只回答「发生了什么」（边沿），不猜「你要怎么统计」。将来要做长按、双击是另一种统计口径——驱动不用改。

---

## 五、怎么测试

### 5.1 为什么「编译通过 + 烧录成功」不算验证

这是嵌入式最典型的认知陷阱。这套工具链的失败模式是：

> **所有命令都返回 0，没有一条报错，硬件毫无反应。**

编译通过只证明语法对；烧录成功只证明二进制写进 Flash 了。**这两件事和「按键能不能用」之间没有任何逻辑关系。**

### 5.2 第一层：静态验证——读寄存器

用 ST-Link 调试口直接读芯片寄存器，**不打断程序运行**（`mode=hotplug` 不复位）：

| 读什么 | 期望 | 实际 | 说明 |
| --- | --- | --- | --- |
| `GPIOB CRH` (0x40010C04) | PB10/PB11 = `0x8` | `0x4444**88**77` | 输入 + 上下拉 ✓ |
| `GPIOA CRH` (0x40010804) | PA11/PA12 = `0x8` | `0x8888**8**444` | ✓ |
| `GPIOB IDR` (0x40010C08) | 按键位为 1 | `0x00000FDA` | 未按下，上拉生效 ✓ |
| `GPIOA IDR` (0x40010808) | 按键位为 1 | `0x0000DF1C` | ✓ |

**为什么是 `0x8`？** STM32 每个引脚在 `CRH` 里占 4 位：`MODE[1:0]` + `CNF[1:0]`。

- `MODE=00` → 输入模式
- `CNF=10` → 带上下拉的输入
- 合起来 `1000b` = `0x8`

而「上拉还是下拉」由 `ODR` 决定（见第二节），所以要配合 `IDR` 读到高才能确认上拉真的生效。

**这一层证明了**：CubeMX 生成的配置真的写进寄存器了，引脚电气状态正确。
**没证明**：代码逻辑对不对。

### 5.3 第二层：动态验证——模拟一次按键

没有摄像头也没有机械手，所以用调试口**从外部改变引脚电平**，让固件的消抖逻辑真实跑一遍。

**做法**（两步，顺序有讲究）：

```bash
# ① PB10 从「输入」改成「推挽输出」
#    CRH 的第 2 个 nibble（bit 8~11）从 8 改成 2：0x44448877 → 0x44448277
STM32_Programmer_CLI -c port=SWD mode=hotplug -w32 0x40010C04 0x44448277

# ② 用 ODR 把 PB10 拉低 —— 固件由此「看到」按下
STM32_Programmer_CLI -c port=SWD mode=hotplug -w32 0x40010C0C 0x00000B10
```

`0x2` 是 `MODE=10`（输出 2 MHz）+ `CNF=00`（推挽）= `0010b`。
`0x00000B10` 是把原来的 `0x00000F10` 清掉 bit10（PB10 的 ODR 位）。

> **别用 BRR。** 实测用 `BRR` 写会报 `Failed to download data`——因为 **BRR 是只写寄存器，读回恒为 0**，
> 而 `STM32_Programmer_CLI` 写完会回读校验，读到全 0 就判定失败。换成可读写的 `ODR` 即可。

**这样安全吗？** 安全。PB10 上接的是「按键 → GND」，把它强行拉低，只是让那个本来由按键承担的接地路径改由芯片承担。电流受上拉电阻限制（3.3 V / 47 kΩ ≈ 0.07 mA）。**但如果引脚上接的是电源轨或别的有源输出，就不能这么干。**

**读回运行时变量：**

```
s_press_count[K1] (0x20000488) : 00000001 00000000 00000000 00000000
                                  ↑ K1 计数 +1，其余三个键仍为 0

s_key_state[0]    (0x200004E0) : 01 01 14 00 00 00
                                  │  │  │  └─┴─ 事件已被主循环取走
                                  │  │  └────── same_count = 0x14 = 20（消抖阈值）
                                  │  └───────── last_raw = 1
                                  └──────────── stable = 1（已确认按下）
```

**最后看屏幕**（读显存还原成图像，方法见 [oled.md](oled.md) 第七节）：

```
K1[X]      ← 方括号里出现了 X（原来是空的）
K1:1       ← 计数从 0 变成 1
down:K1    ← 「最近按下」从 - 变成 K1
```

**这证明了整条链路：**

```
引脚电平变化
  → TIM1 1 ms 中断触发
    → bsp_tick 分发
      → key_scan_one 采样到变化
        → 连续 20 次一致，判定翻转
          → 置 press_pending
            → 主循环 take_event 取走（含临界区保护）
              → 计数 +1
                → 重画屏幕 → update_screen → I2C → 屏幕上真的变了
```

这比「直接改内存变量」强得多——后者只验证了显示，这里是**真的让 ISR 跑了一遍**。

### 5.4 没验证到什么

没有物理按下过按键（没有摄像头，也没有机械执行器），所以：

| 已验证 | 未验证 |
| --- | --- |
| 采样节拍、计数、阈值判定 | **真实触点的弹跳行为** |
| 事件产生与消费 | 实际接线的机械特性 |
| 显示与整条 I2C 链路 | 长按、连按的手感 |

特别是**弹跳**：模拟出来的是**干净的电平跳变**，不是真实的抖动。所以「消抖算法能扛住真实弹跳」是靠算法结构推断的（任何偏离都会把计数清零，毛刺攒不够 20 次），**不是实测的**。

**这一条要动手确认**：按 K1 一次，屏幕上的 `K1` 计数应该**正好 +1**。

- 一次按下加了 2、3 甚至十几 → 消抖时间不够，把 `BSP_KEY_DEBOUNCE_MS` 从 20 加到 30
- 按下去要等一会儿才有反应 → 正常，那是 20 ms 确认延迟

---

## 六、设计取舍一览

| 决定 | 换来什么 | 代价 |
| --- | --- | --- |
| 采样放 1 ms 中断 | 采样间隔恒定，不丢短按 | 中断里多跑 2~3 µs |
| 消抖 20 ms | 抗抖动有余量 | 确认延迟 20 ms（无感） |
| 边沿 + 电平双接口 | 两类需求都能满足 | 多两个标志位 |
| press/release 两个标志分开 | 快速按放不丢事件 | 状态多一个字节 |
| `take_event` 关中断 3 条指令 | 消除「一天一次」的竞态 | 几十纳秒的中断延迟 |
| 引脚交给 CubeMX | 配置集中、防抢引脚 | `bsp_key_init` 必须在 `MX_GPIO_Init` 之后 |

整个模块约 260 行，其中真正的「算法」只有 `key_scan_one` 那 25 行，其余都是**为健壮性付出的成本**——初值处理、临界区、事件缓冲、参数校验。

这也基本是嵌入式驱动开发的常态比例：**逻辑本身往往很简单，难的是让它在各种边界条件下都不出错。**
