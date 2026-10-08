/**
 * @file    bsp_serial.c
 * @brief   板载串口的实现（USART1，双向中断 + 环形缓冲）。
 *
 * ── 环形缓冲的并发模型：单生产者 / 单消费者（SPSC）──────────────────
 *   这是本模块最需要看懂的地方。
 *
 *   发送缓冲：head 只由**主循环**写（bsp_serial_write），
 *             tail 只由**发送中断**写（HAL_UART_TxCpltCallback）。
 *   接收缓冲：head 只由**接收中断**写（HAL_UART_RxCpltCallback），
 *             tail 只由**主循环**写（bsp_serial_read）。
 *
 *   两个索引各自只有一个写者，所以**读写索引本身不需要临界区**——
 *   Cortex-M3 对 16 位对齐变量的访存是单条指令，不会读到「改了一半」的值。
 *   这就是这套缓冲不用关中断的原因，也是它能做到非阻塞的基础。
 *
 *   ⚠️ 但索引**发布**与数据**写入**之间要有顺序保证：必须先把数据写进缓冲、
 *      再把 head 推上去，否则中断可能读到 head 已更新、内容还是旧的位置。
 *      代码里用 __DMB() 明确这个顺序。
 *
 *   唯一的例外是 tx_kick()：见那里的说明。
 * ────────────────────────────────────────────────────────────────────
 */

#include "bsp_serial.h"

#include <stdbool.h>
#include <stddef.h>

#include "main.h"                   /* 借它引入 stm32f1xx_hal.h */

/*
 * USART1 的句柄由 CubeMX 生成在 main.c 里（`UART_HandleTypeDef huart1;`），
 * 但 main.h 没有为它生成 extern 声明，所以这里自己声明一份。
 * 与 bsp/tick/bsp_tick.c 声明 htim1、bsp/pot/bsp_pot.c 声明 hadc2 是同样的做法。
 */
extern UART_HandleTypeDef huart1;

/* 缓冲大小必须是 2 的幂，下面的掩码才成立。编译期挡住写错的改动。 */
#if (BSP_SERIAL_TX_BUF_SIZE & (BSP_SERIAL_TX_BUF_SIZE - 1U)) != 0U
#error "BSP_SERIAL_TX_BUF_SIZE 必须是 2 的幂"
#endif
#if (BSP_SERIAL_RX_BUF_SIZE & (BSP_SERIAL_RX_BUF_SIZE - 1U)) != 0U
#error "BSP_SERIAL_RX_BUF_SIZE 必须是 2 的幂"
#endif

#define SERIAL_TX_MASK          (BSP_SERIAL_TX_BUF_SIZE - 1U)
#define SERIAL_RX_MASK          (BSP_SERIAL_RX_BUF_SIZE - 1U)

/** 每次收到一个字节的中转落点（HAL 要求给它一个缓冲区地址） */
#define SERIAL_RX_CHUNK         (1U)

/* ------------------------------------------------------------- 发送状态 */

static uint8_t  s_tx_buf[BSP_SERIAL_TX_BUF_SIZE];
static volatile uint16_t s_tx_head;     /**< 生产者（主循环）写 */
static volatile uint16_t s_tx_tail;     /**< 消费者（发送中断）写 */

/**
 * 当前这一笔交给 HAL 的长度。
 *
 * HAL_UART_Transmit_IT() 只拿到一个指针，完成回调里不告诉我们是哪一笔，
 * 所以得自己记住，回调里靠它推进 tail。
 */
static volatile uint16_t s_tx_len;

/**
 * 是否有一笔正在硬件里发送。
 *
 * 用来保证同一时刻只有一个 HAL_UART_Transmit_IT() 在跑——
 * HAL 对重复启动会返回 HAL_BUSY，而且此时内部的缓冲区指针会被搞乱。
 */
static volatile bool s_tx_busy;

/* ------------------------------------------------------------- 接收状态 */

static uint8_t  s_rx_buf[BSP_SERIAL_RX_BUF_SIZE];
static volatile uint16_t s_rx_head;     /**< 生产者（接收中断）写 */
static volatile uint16_t s_rx_tail;     /**< 消费者（主循环）写 */

/** 接收中断的单字节落点 */
static uint8_t  s_rx_byte;

/** 接收缓冲满而丢弃的字节数（诊断用） */
static volatile uint32_t s_rx_dropped;

/** 是否已初始化 */
static bool s_ready = false;

/* --------------------------------------------------------------- 内部 */

/**
 * @brief 若当前空闲且有数据待发，就启动一笔发送。
 *
 * ── 为什么这里必须关中断 ────────────────────────────────────────────
 *   本函数做的事是「检查 s_tx_busy，若空闲则置位并启动发送」——
 *   一个典型的 check-then-act。而 s_tx_busy 同时被**发送中断**改
 *   （回调里清掉它，然后也调用本函数）。
 *
 *   若不加保护，会出现这样一条交错：
 *     1. 主循环读到 s_tx_busy == false，判断「空闲」，准备启动；
 *     2. 就在这一瞬间，上一笔的 TxCplt 中断到达，回调把 s_tx_busy 清 0，
 *        并自己启动了一笔新的发送；
 *     3. 主循环恢复执行，也去启动一笔 → 两次 HAL_UART_Transmit_IT 并发，
 *        内部的发送指针互相覆盖，发出去的东西就乱了。
 *
 *   这个窗口只有几条指令，不加保护时**极难复现**，所以必须在这里关中断。
 *   关中断的时长就是下面这几条语句 + 一次 HAL 调用（它不搬数据，
 *   只是记下指针并使能 TXE 中断），量级是微秒，对 1 ms 节拍毫无影响。
 * ────────────────────────────────────────────────────────────────────
 */
static void serial_tx_kick(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (!s_tx_busy) {
        const uint16_t head = s_tx_head;
        const uint16_t tail = s_tx_tail;

        if (head != tail) {
            /*
             * 只发「从 tail 到 head 或到缓冲区末尾」这一段连续数据，
             * 不跨越回绕点。跨回绕的部分留给下一次回调再发。
             */
            const uint16_t n = (head > tail) ? (uint16_t)(head - tail)
                                             : (uint16_t)(BSP_SERIAL_TX_BUF_SIZE - tail);

            s_tx_len  = n;
            s_tx_busy = true;

            if (HAL_UART_Transmit_IT(&huart1, &s_tx_buf[tail], n) != HAL_OK) {
                /* 启动失败就把状态退回去，否则发送会永久卡死 */
                s_tx_busy = false;
                s_tx_len  = 0U;
            }
        }
    }

    __set_PRIMASK(primask);
}

/* ------------------------------------------------------------- 对外实现 */

error_t bsp_serial_init(void)
{
    s_ready = false;

    s_tx_head = 0U;
    s_tx_tail = 0U;
    s_tx_len  = 0U;
    s_tx_busy = false;

    s_rx_head    = 0U;
    s_rx_tail    = 0U;
    s_rx_dropped = 0U;

    /*
     * 武装接收。这一步**必须**在数据到来之前完成——晚了就丢字节，
     * 而且不会报错。
     *
     * 注意这里不调用 HAL_UART_Init()：USART1 的寄存器配置（115200-8-N-1）
     * 由 CubeMX 生成的 MX_USART1_UART_Init() 负责，本函数只管中断这一层。
     * 因此本函数必须在 MX_USART1_UART_Init() 之后被调用。
     */
    if (HAL_UART_Receive_IT(&huart1, &s_rx_byte, SERIAL_RX_CHUNK) != HAL_OK) {
        return ERR_NOT_READY;
    }

    s_ready = true;

    return ERR_OK;
}

uint32_t bsp_serial_write(const uint8_t *data, uint32_t len)
{
    if ((data == NULL) || (!s_ready)) {
        return 0U;
    }

    uint16_t head     = s_tx_head;      /* 只有本函数写 head，先读到本地 */
    const uint16_t tail = s_tx_tail;    /* 由中断写，只读一次，保证整段一致 */
    uint32_t written  = 0U;

    while (written < len) {
        /* 留一个空位用于区分「满」和「空」——这是环形缓冲的常规做法：
           若不空出一格，head == tail 就同时意味着空和满，无法判断。 */
        const uint16_t next = (uint16_t)((head + 1U) & SERIAL_TX_MASK);

        if (next == tail) {
            break;                      /* 缓冲满，能放多少放多少 */
        }

        s_tx_buf[head] = data[written];
        head = next;
        written++;
    }

    if (written > 0U) {
        /* 先让数据对中断可见，再发布 head（见文件头的顺序说明） */
        __DMB();
        s_tx_head = head;

        serial_tx_kick();
    }

    return written;
}

uint32_t bsp_serial_read(uint8_t *out, uint32_t max)
{
    if ((out == NULL) || (!s_ready)) {
        return 0U;
    }

    const uint16_t head = s_rx_head;    /* 由中断写，只读一次 */
    uint16_t       tail = s_rx_tail;    /* 只有本函数写 tail */
    uint32_t       got  = 0U;

    while ((got < max) && (tail != head)) {
        out[got] = s_rx_buf[tail];
        tail = (uint16_t)((tail + 1U) & SERIAL_RX_MASK);
        got++;
    }

    if (got > 0U) {
        __DMB();
        s_rx_tail = tail;
    }

    return got;
}

uint32_t bsp_serial_rx_available(void)
{
    const uint16_t head = s_rx_head;
    const uint16_t tail = s_rx_tail;

    return (uint32_t)((head - tail) & SERIAL_RX_MASK);
}

uint32_t bsp_serial_tx_free(void)
{
    const uint16_t head = s_tx_head;
    const uint16_t tail = s_tx_tail;

    /* 空出一格，故减 1（见 bsp_serial_write 里的说明） */
    return (uint32_t)(((tail - head - 1U) & SERIAL_TX_MASK));
}

uint32_t bsp_serial_rx_dropped(void)
{
    return s_rx_dropped;
}

error_t bsp_serial_flush(uint32_t timeout_ms)
{
    if (!s_ready) {
        return ERR_NOT_INITIALIZED;
    }

    const uint32_t start = HAL_GetTick();

    /*
     * 「发完」有两个条件，缺一不可：
     *   1. 缓冲空了（head == tail）——没有数据还在排队；
     *   2. 硬件也空闲（!s_tx_busy）——最后一笔已经发完。
     *
     * 只看第 1 条会在「最后一笔正在发送中」时提前返回，
     * 此时最后一个字节可能还没完全移出移位寄存器。
     */
    while ((s_tx_head != s_tx_tail) || s_tx_busy) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            return ERR_TIMEOUT;
        }
    }

    return ERR_OK;
}

/* ------------------------------------------------- HAL 回调（中断上下文） */

/**
 * @brief 一笔发送完成。
 *
 * @note 由 CubeMX 生成的 USART1_IRQHandler → HAL_UART_IRQHandler 调用。
 *       这是 HAL 定义的 __weak 符号，整个工程只能有一份非弱定义。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) {
        return;                     /* 不是本模块的串口，不处理 */
    }

    /* 刚才那一笔是从旧 tail 开始、共 s_tx_len 字节，推进到它的末尾 */
    s_tx_tail = (uint16_t)((s_tx_tail + s_tx_len) & SERIAL_TX_MASK);
    s_tx_len  = 0U;
    s_tx_busy = false;

    /* 缓冲里可能还有下一段，接着发 */
    serial_tx_kick();
}

/**
 * @brief 收到一个字节。
 *
 * @note ⚠️ 结尾那句重新武装是**必须**的：HAL_UART_Receive_IT() 收满
 *       指定的 Size（这里是 1）就自动关闭接收中断并回调一次，
 *       不在回调里重新武装，串口只能收到**一个字节**就再无下文——
 *       而且不会报任何错，现象是「刚上电那一瞬间有数据，之后永远没有」。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) {
        return;
    }

    const uint16_t next = (uint16_t)((s_rx_head + 1U) & SERIAL_RX_MASK);

    if (next == s_rx_tail) {
        /* 缓冲满：丢弃。丢掉的是**新来的**字节，保留先到的，
           这样上层至少还能看到一条完整报文的前半段，比丢中间的强。 */
        s_rx_dropped++;
    } else {
        s_rx_buf[s_rx_head] = s_rx_byte;
        __DMB();
        s_rx_head = next;
    }

    (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, SERIAL_RX_CHUNK);
}

/**
 * @brief 接收出错回调（校验错 PE / 噪声 NE / 帧错 FE / 溢出 ORE）。
 *
 * @note 这个回调为什么不能省，要去 HAL 源码里看（stm32f1xx_hal_uart.c 的
 *       HAL_UART_IRQHandler）。它把错误分成**两类**，处理方式完全不同：
 *
 *         · **阻塞类**——ORE（接收溢出），或者启用了接收 DMA。
 *           源码里对这个分支的注释就是「Blocking error : transfer is aborted」，
 *           紧接着调用 UART_EndRxTransfer()：**中止接收、关掉接收中断**，
 *           然后把控制权交到本回调。此时若不重新武装，接收中断从此再不触发，
 *           串口就**永久性地只能发不能收**。
 *
 *         · **非阻塞类**——只有 PE / NE / FE 时。源码注释是
 *           「Non Blocking error : transfer could go on.」，
 *           **不会**中止接收，只是通知一声。
 *
 *       本回调对两种都是「清标志 + 重新武装」：前者必须，后者冗余但无害
 *       （HAL 若发现已在接收中会返回 HAL_BUSY，忽略即可）。
 *
 *       实际最容易撞上的是 ORE（溢出）：主循环太久没来取数据、或对端热插拔时
 *       线上出现一串脉冲，接收移位寄存器里的数据还没被读走就被新数据覆盖。
 *       现象正是「本来好好的，某一刻起再也收不到东西」。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) {
        return;
    }

    /* 清掉可能残留的 PE/FE/NE/ORE 标志，否则会立刻再触发一次 */
    __HAL_UART_CLEAR_PEFLAG(&huart1);

    /* 中止过接收，重新武装 */
    (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, SERIAL_RX_CHUNK);
}

/* ------------------------------------------------------- printf 重定向 */

/**
 * @brief newlib 的系统调用桩：printf 最终落到这里。
 *
 * newlib 的 printf 不直接碰硬件，而是把格式化好的字节交给 _write()。
 * 工具链默认提供一个「什么都不做」的 _write（来自 --specs=nosys.specs），
 * 这里给出强定义把它覆盖掉，printf 就接到了本模块的发送缓冲上。
 *
 * ⚠️ 语义：缓冲满时返回**短计数**，printf 输出的内容会被截断。
 *    这是刻意的（见文件头）。要保证不丢，调用前先用 bsp_serial_tx_free()
 *    确认空间，或改用 bsp_serial_write() + bsp_serial_flush()。
 */
int _write(int file, char *ptr, int len)
{
    (void)file;

    if ((ptr == NULL) || (len <= 0)) {
        return 0;
    }

    return (int)bsp_serial_write((const uint8_t *)ptr, (uint32_t)len);
}
