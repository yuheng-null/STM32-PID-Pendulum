/**
 * @file    bsp_serial.h
 * @brief   板载串口：USART1（PA9 = TX，PA10 = RX），收发双向中断 + 环形缓冲。
 *
 * ── 接线与参数 ──────────────────────────────────────────────────────
 *   MCU PA9  (USART1_TX) ── USB-TTL 模块的 RX
 *   MCU PA10 (USART1_RX) ── USB-TTL 模块的 TX
 *   MCU GND              ── USB-TTL 模块的 GND（**必须共地**）
 *
 *   ⚠️ TX 接对端的 RX、RX 接对端的 TX，交叉接线。
 *      TX 接 TX 的现象是「两边都收不到，且不报任何错」。
 *
 *   GPIO 与 USART1 本身的参数都在 CubeMX 里（见 PID_Pendulum.ioc），
 *   由生成的 MX_USART1_UART_Init() / HAL_UART_MspInit() 完成：
 *   115200-8-N-1，无硬件流控，PA9 复用推挽、PA10 浮空/上拉输入。
 *   本模块不碰 GPIO，也不重配波特率。
 * ────────────────────────────────────────────────────────────────────
 *
 * ── 为什么收发都要中断 + 环形缓冲 ────────────────────────────────────
 *   收：**必须**中断驱动。115200 下一个字节只隔约 87 µs，主循环里任何一次
 *       屏幕刷新（软件 I2C 写 1024 字节显存）都远超这个时间，靠轮询必然丢数据。
 *
 *   发：做成中断驱动是为了**不阻塞主循环**。同样是 115200，发送 32 字节
 *       要 2.8 ms——如果用 HAL_UART_Transmit() 轮询发送，主循环会被卡住
 *       整整 2.8 ms，这段时间里按键不响应、电位器不采样。
 *       本模块的发送只是把数据拷进缓冲就返回，真正往硬件塞字节由中断做。
 *
 *   代价：write() 返回时数据**可能还没发出去**。这是非阻塞的固有语义，
 *   不是 bug。需要「返回即已发完」的场合要用 bsp_serial_flush() 显式等待。
 * ────────────────────────────────────────────────────────────────────
 *
 * ── 缓冲区大小与「必须 2 的幂」 ──────────────────────────────────────
 *   大小必须是 2 的幂，因为索引回绕用的是一次按位与 `& (SIZE-1)`，
 *   而不是取模 `% SIZE`——前者在 Cortex-M3 上是一条指令，后者是除法。
 *   改大小时请保持 2 的幂，否则回绕会算错（而且不会报错，只会莫名其妙丢数据）。
 *
 *   两个缓冲各占 SIZE 字节的静态 RAM。当前 256 + 256 = 512 字节。
 * ────────────────────────────────────────────────────────────────────
 *
 * ── printf ──────────────────────────────────────────────────────────
 *   本模块实现了 newlib 的 _write() 系统调用，所以 printf 可以直接用，
 *   输出会走**发送环形缓冲**（非阻塞）。用法：
 *
 *     bsp_serial_init();
 *     printf("rp1=%u rp2=%u\r\n", bsp_pot_raw(POT_ID_RP1), bsp_pot_raw(POT_ID_RP2));
 *
 *   ⚠️ 三条约束：
 *     1. 缓冲满时 printf 会**截断输出**（返回的字节数小于请求值），
 *        不会阻塞等待。这是刻意的——调试输出不值得拖住控制循环。
 *        用 bsp_serial_rx_available() 式的思路看，可用 bsp_serial_tx_free()
 *        预判还有多少空间。
 *     2. 含浮点的格式化（%f）默认**不可用**：链接选项没开
 *        `-u _printf_float`，开了会多占约 6 KB Flash。要用得改顶层
 *        CMakeLists.txt。打印浮点可以先自行放大成整数再打。
 *     3. printf 会拉进约 10 KB 的 newlib 格式化代码。Flash 只有 64 KB，
 *        用量见每次编译后的内存占用表。
 *
 *   ⚠️ **绝对不要在中断里调 printf**：它不重入，而且格式化耗时可观。
 * ────────────────────────────────────────────────────────────────────
 */

#ifndef BSP_SERIAL_H
#define BSP_SERIAL_H

#include <stdint.h>

#include "error.h"

/**
 * 发送缓冲大小（字节）。必须是 2 的幂，理由见文件头说明。
 */
#define BSP_SERIAL_TX_BUF_SIZE      (256U)

/**
 * 接收缓冲大小（字节）。必须是 2 的幂。
 */
#define BSP_SERIAL_RX_BUF_SIZE      (256U)

/**
 * @brief 初始化串口模块：清空缓冲、武装接收中断。
 *
 * @return error_t ERR_OK 成功；ERR_NOT_READY 接收中断武装失败
 *
 * @note 必须在 CubeMX 生成的 MX_USART1_UART_Init() **之后**调用。
 *       本函数不调用 HAL_UART_Init()——USART1 的寄存器配置归 CubeMX 管。
 *
 * @note 本函数只是把「接收一个字节」的中断挂上去。HAL_UART_Receive_IT()
 *       每次只收 Size 个字节（这里给 1），收满就回调一次，回调里再重新武装。
 *       这是 HAL 的单字节接收惯用法。
 */
error_t bsp_serial_init(void);

/**
 * @brief 把数据放进发送缓冲，立即返回，不等待发完。
 *
 * @param data 待发送数据，不可为 NULL
 * @param len  字节数
 *
 * @return uint32_t 实际放进去的字节数；缓冲满时小于 len。
 *                  模块未初始化或 data 为 NULL 时返回 0。
 *
 * @note 非阻塞。返回后数据可能还在缓冲里排队，由发送中断逐步送出。
 *       想要「返回即已发完」，随后调用 bsp_serial_flush()。
 */
uint32_t bsp_serial_write(const uint8_t *data, uint32_t len);

/**
 * @brief 从接收缓冲取走数据。
 *
 * @param out 接收缓冲，不可为 NULL
 * @param max 最多取多少字节
 *
 * @return uint32_t 实际取到的字节数；缓冲为空时返回 0（不是错误）
 *
 * @note 非阻塞：有多少取多少，绝不等下一个字节。
 */
uint32_t bsp_serial_read(uint8_t *out, uint32_t max);

/**
 * @brief 接收缓冲里已积压多少字节（可用于判断是否攒够一整条命令）。
 */
uint32_t bsp_serial_rx_available(void);

/**
 * @brief 发送缓冲里还能再放多少字节。
 */
uint32_t bsp_serial_tx_free(void);

/**
 * @brief 因接收缓冲满而被丢弃的字节数。
 *
 * @note 这个计数长期不为 0 说明**主循环取数据太慢**，
 *       或者上层协议的单条报文超过了 BSP_SERIAL_RX_BUF_SIZE。
 *       它是个诊断量，正常情况下应该恒为 0。
 */
uint32_t bsp_serial_rx_dropped(void);

/**
 * @brief 阻塞等待发送缓冲里的数据全部发完（含最后一位移出）。
 *
 * @param timeout_ms 等待上限（毫秒）
 *
 * @return error_t ERR_OK 已发完；ERR_TIMEOUT 超时
 *
 * @note **这是本模块唯一的阻塞函数**，只应在收尾/调试场合使用
 *       （例如 main 返回前、或复位前确保日志已经发出去）。
 *       不要在周期性任务里调用它。
 */
error_t bsp_serial_flush(uint32_t timeout_ms);

#endif /* BSP_SERIAL_H */
