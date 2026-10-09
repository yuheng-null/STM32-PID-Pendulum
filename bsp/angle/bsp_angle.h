/**
 * @file    bsp_angle.h
 * @brief   倒立摆摆杆的角度传感器（模拟电位器，走 ADC1_IN8）。
 *
 * ── 接线 ────────────────────────────────────────────────────────────
 *   PB0 ── SENSOR1 (ADC1_IN8) ← 角度传感器模块的 OUT
 *
 *   本套件的角度传感器**不是** MPU6050/AS5600 那类数字器件，而是一只
 *   360° 旋转的电位器（型号 SV01A103AEA01R00，10K）：两个固定端接
 *   3V3 与 GND，抽头接 PB0。摆杆转动带动抽头，PB0 上就是一个随角度
 *   连续变化的电压，ADC 把它量化成 0~4095。
 *
 *   引脚的模拟输入模式由生成的 HAL_ADC_MspInit() 完成（PB0 配成
 *   GPIO_MODE_ANALOG），本模块不碰 GPIO。
 * ────────────────────────────────────────────────────────────────────
 *
 * ── ⚠️ 有效角度只有 333°，有约 27° 的盲区 ───────────────────────────
 *   电位器转到两端之间会有一小段「盲区」，跨过它时读数会突变。
 *   这不是代码问题，是器件特性。**本模块不做任何补偿**——
 *   正确的做法是应用层做范围检查（参考实现在角度超出中心区间时
 *   直接停机）。
 *
 *   另外手册强调角度传感器与摆杆的**安装方向**要求：摆杆自然下垂时，
 *   法兰联轴器的切面必须朝上。装反了倒立摆无法工作。
 * ────────────────────────────────────────────────────────────────────
 *
 * ── 为什么本模块不给「角度是多少度」 ────────────────────────────────
 *   「竖直」对应哪个读数，取决于传感器的机械安装相位——换一台设备就
 *   变了。把它写进驱动会让模块不可移植，也违反「驱动不含板外标定」
 *   的分工。所以这里只提供电压/原始值，**零点由应用层标定**：
 *
 *     把摆杆扶到竖直，读 bsp_angle_raw()，记下来当 CENTER_ANGLE；
 *     之后 PID 用 (raw - CENTER_ANGLE) 当误差即可。
 *
 *   参考实现就是这么做的（main.c 里一个 CENTER_ANGLE 宏，实测应在
 *   1900~2200 之间）。
 * ────────────────────────────────────────────────────────────────────
 *
 * ── 两套采样接口，用哪套取决于调用上下文 ────────────────────────────
 *
 *   ① **主循环 / 测试用 —— 阻塞式**
 *
 *        if (bsp_angle_sample() == ERR_OK) {
 *            uint16_t a = bsp_angle_raw();      // 0~4095
 *        }
 *
 *      触发一次、等到结果、返回，全程约 6 µs。顺带把上次的值刷新到
 *      `bsp_angle_raw()`。
 *
 *   ② **中断 / 控制环用 —— 非阻塞式（分开的两步）**
 *
 *        bsp_angle_trigger();                    // 启动转换，立即返回
 *        ...                                    // 干别的去
 *        if (bsp_angle_poll() == ERR_OK) {       // 回来看看转完没有
 *            uint16_t a = bsp_angle_raw();       // ERR_OK 才代表有新值
 *        }
 *
 *      **拆成两步是必须的，不是风格问题**：一次转换要 5.7 µs，而 1 ms
 *      控制节拍里有的是别的事可干；更关键的是 `bsp_angle_sample()` 内部
 *      走 `HAL_ADC_Start()`，那里面有 `HAL_GetTick()` 的超时判断，
 *      在 1 ms 中断里**会死等**（详见 `bsp_angle_sample()` 的说明）。
 *      `trigger`/`poll` 里没有任何依赖 `HAL_GetTick()` 的调用，
 *      整套是纯寄存器操作，**可以在任何优先级的中断里跑**。
 *
 *   ⚠️ 换句话说：**`bsp_angle_sample()` 不要挂到 1 ms 节拍上**
 *      （理由与 bsp_pot 相同）；节拍里请用 ②。
 *
 *   两套接口共用同一份状态和同一个 `s_angle_raw`，可以混用。
 */

#ifndef BSP_ANGLE_H
#define BSP_ANGLE_H

#include <stdint.h>

#include "error.h"

/** ADC 满量程（12 位） */
#define BSP_ANGLE_RAW_MAX       (4095U)

/** 参考电压（mV）。VDDA = 3.3 V，与板子供电一致。 */
#define BSP_ANGLE_VREF_MV       (3300U)

/**
 * @brief 初始化 ADC1 并做自校准。
 *
 * @return error_t ERR_OK 成功；ERR_NOT_READY 校准或启动失败
 *
 * @note 必须在 CubeMX 生成的 MX_ADC1_Init() **之后**调用。
 *
 * @note F1 的 ADC 必须先自校准再使用（与 ADC2 同理）：它测量芯片内部
 *       的电容失配并写入校准寄存器，不做的话读数会有几十个 LSB 的
 *       固定偏差，而且随温度漂移。漏了的现象是「能读、但数值不准」，
 *       **不会报任何错**。
 *
 * @note 校准内部用到 HAL_GetTick()，所以本函数只能在初始化阶段
 *       （主线程）调用。
 *
 * @note **本函数把 ADC 永久打开，之后再也不关**（详见 .c 里的说明）。
 *       这是刻意的：ADC 每次从关到开都要等一次 ADRDY 稳定时间，而
 *       那段等待在 HAL 里靠 HAL_GetTick()，中断里没法用。
 */
error_t bsp_angle_init(void);

/**
 * @brief 【非阻塞 · 第一步】启动一次转换，立即返回。
 *
 * @return error_t ERR_OK 已启动；ERR_NOT_INITIALIZED 未初始化
 *
 * @note 纯寄存器写（置 `CR2` 的 `SWSTART`），耗时几十纳秒，**中断安全**。
 *
 * @note 调用之后要过约 5.7 µs 结果才就绪，用 `bsp_angle_poll()` 取。
 *
 * @note 重复调用（上次结果还没取走）不会出错：上一次的转换结果会被
 *       丢弃，从这一次重新算起。控制环每个周期都会先取再触发，
 *       正常不会走到这条路径。
 */
error_t bsp_angle_trigger(void);

/**
 * @brief 【非阻塞 · 第二步】看看上次 trigger 的转换转完没有。
 *
 * @return error_t ERR_OK = 转完了，值已更新到 `bsp_angle_raw()`；
 *                 ERR_NOT_READY = 还没转完（或没 trigger 过）；
 *                 ERR_NOT_INITIALIZED = 未初始化
 *
 * @note 一次转换只报告一次 `ERR_OK`。取走后再调还是 `ERR_NOT_READY`，
 *       直到下一次 `bsp_angle_trigger()` —— 这样「ERR_OK」就等价于
 *       「拿到一个新的采样」，而不是「读到一个可能很旧的值」。
 *       控制环靠这个语义判断数据是否新鲜。
 *
 * @note **中断安全**：只读 `SR` 与 `DR` 两个寄存器，不碰 HAL_GetTick。
 */
error_t bsp_angle_poll(void);

/**
 * @brief 【阻塞】触发一次转换、等到结果、返回。`trigger` + `poll` 的包装。
 *
 * @return error_t ERR_OK 成功；ERR_NOT_INITIALIZED 未初始化；
 *                 ERR_TIMEOUT 等超时（有界自旋耗尽）
 *
 * @note 正常耗时约 6 µs（(55.5 + 12.5) 个 ADC 周期 @ 12 MHz）。
 *
 * @note ⚠️ **只能在主循环里调用，不要放进中断。** 不是因为等待方式
 *       （它是有界自旋，不依赖 HAL_GetTick），而是因为**它要花 6 µs
 *       干等**——放在 1 ms 节拍里等于白占 0.6% 的 CPU，还没给控制环
 *       留下「转换期间去干别的」的机会。中断里请用 trigger/poll 两步式。
 *
 * @note 超时不是死等：自旋次数有上限（`BSP_ANGLE_SPIN_LIMIT`），
 *       耗尽就返回 ERR_TIMEOUT。这比原实现更安全——原实现依赖
 *       HAL_GetTick()，而 TIM1 节拍的抢占优先级（1）高于 SysTick（15），
 *       在节拍里 uwTick 根本不涨，真正的故障会**永久卡死**。
 */
error_t bsp_angle_sample(void);

/**
 * @brief 取上一次采样得到的原始值。
 *
 * @return uint16_t 0~4095；未采样过时返回 0
 *
 * @note 读的是缓存，不触发任何转换，纯内存访问。
 */
uint16_t bsp_angle_raw(void);

/**
 * @brief 取上一次采样折算出的电压。
 *
 * @return uint16_t 0~3300（mV）
 *
 * @note 用于验证接线：把摆杆从一端缓慢转到另一端，读数应从接近 0
 *       连续变到接近 3300。恒为 0 或恒为满量程 → 接线问题（常见是
 *       只接了两端没接抽头，或者 3V3/GND 那端没接上）。
 */
uint16_t bsp_angle_millivolt(void);

#endif /* BSP_ANGLE_H */
