/**
 * @file    bsp_key.h
 * @brief   板载 4 个独立按键（K1~K4），带消抖。
 *
 * ── 接线 ────────────────────────────────────────────────────────────
 *   K1 ── PB10        K2 ── PB11        K3 ── PA11        K4 ── PA12
 *
 *   按键另一端接 GND，引脚配成上拉输入，因此**按下时读到低电平**。
 *   如果实际接法相反（按下读高），改 bsp_key.c 里的 BSP_KEY_ACTIVE_LEVEL
 *   为 GPIO_PIN_SET，并同步把 .ioc 里这四个引脚改成下拉。
 *
 *   这四个引脚的**声明和配置都在 CubeMX 里**（见 PID_Pendulum.ioc），
 *   本模块只使用生成的 KEY1_Pin / KEY1_GPIO_Port 宏。
 * ────────────────────────────────────────────────────────────────────
 *
 * ── 为什么消抖要放在 1ms 中断里，而不是主循环里 ──────────────────────
 *   机械触点在按下/松开的瞬间会弹跳几毫秒，直接读会得到一串
 *   忽高忽低的毛刺（一次按下被当成十几次）。
 *   消抖的标准做法是「定期采样 + 连续 N 次一致才认定电平真的变了」。
 *
 *   关键在**定期**这两个字：这一版的主循环里刷一次屏要 ~30 ms，
 *   如果靠在主循环里采样，采样间隔会随着有没有刷屏而剧烈变化——
 *   一次短按（约 50 ms）可能整段落在刷屏期间，直接被漏掉。
 *   挂到 1 ms 节拍上，采样间隔就是恒定的，与主循环在忙什么无关。
 *
 * ── 使用方式 ────────────────────────────────────────────────────────
 *   主循环里轮询事件；事件是「边沿」，读取即清除：
 *
 *     key_event_t ev = bsp_key_take_event(KEY_ID_K1);
 *     if (ev == KEY_EVENT_PRESS)      { ... 按下瞬间 ... }
 *     if (ev == KEY_EVENT_RELEASE)    { ... 松开瞬间 ... }
 *
 *   想知道「当前是不是按着」，用 bsp_key_is_pressed()（电平）。
 */

#ifndef BSP_KEY_H
#define BSP_KEY_H

#include <stdbool.h>
#include <stdint.h>

#include "error.h"

/** 按键编号。顺序即 K1~K4。 */
typedef enum {
    KEY_ID_K1 = 0,
    KEY_ID_K2,
    KEY_ID_K3,
    KEY_ID_K4,
    KEY_ID_COUNT,               /**< 按键总数，也用作遍历上界 */
} key_id_t;

/** 按键事件（边沿） */
typedef enum {
    KEY_EVENT_NONE = 0,         /**< 自上次读取以来没有事件 */
    KEY_EVENT_PRESS,            /**< 按下 */
    KEY_EVENT_RELEASE,          /**< 松开 */
} key_event_t;

/**
 * @brief 初始化 4 个按键。
 *
 * 用当前实际电平初始化消抖状态（避免开机瞬间凭空产生一个按下事件），
 * 然后把自己注册到 1 ms 节拍上。**不配置引脚**——那由 CubeMX 生成的
 * MX_GPIO_Init() 完成。
 *
 * @return error_t ERR_OK 成功；ERR_OVERFLOW 节拍任务槽位已满
 *
 * @note 调用顺序有要求，两个都要满足：
 *       ① 在 MX_GPIO_Init() **之后**（否则读到的引脚电平还是浮空值，
 *          记录的初始状态是错的）；
 *       ② 在 bsp_tick_init() **之后**（注册节拍任务的前提）。
 */
error_t bsp_key_init(void);

/**
 * @brief 1 ms 采样一次 4 个按键并推进消抖状态机。
 *
 * 由 bsp_tick 每毫秒调用一次，**运行在中断上下文**。
 * 应用不需要（也不应该）主动调用它。
 */
void bsp_key_scan_isr(void);

/**
 * @brief 取走该键自上次取走以来发生的事件（读取即清除）。
 *
 * @param id 按键编号
 *
 * @return key_event_t 发生的事件；没有则返回 KEY_EVENT_NONE
 *
 * @note 同一个键的按下与松开若都还没被取走，会先返回 PRESS，
 *       下一次调用再返回 RELEASE，不会丢事件。
 */
key_event_t bsp_key_take_event(key_id_t id);

/**
 * @brief 该键当前是否处于按下状态（已消抖）。
 *
 * @param id 按键编号
 *
 * @return bool true 表示稳定地处于按下状态
 */
bool bsp_key_is_pressed(key_id_t id);

#endif /* BSP_KEY_H */
