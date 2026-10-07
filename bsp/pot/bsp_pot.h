/**
 * @file    bsp_pot.h
 * @brief   板载 4 路电位器旋钮（RP1~RP4），通过 ADC2 读取。
 *
 * ── 接线 ────────────────────────────────────────────────────────────
 *   RP1 中间抽头 ── PA2 (ADC2_IN2)
 *   RP2 中间抽头 ── PA3 (ADC2_IN3)
 *   RP3 中间抽头 ── PA4 (ADC2_IN4)
 *   RP4 中间抽头 ── PA5 (ADC2_IN5)
 *
 *   每只电位器**三个脚都要接**：两端分别接 3V3 与 GND，中间抽头接 ADC。
 *   抽头对地电压随旋钮在 0~3.3 V 之间连续变化，ADC 把它量化成 0~4095。
 *   若两端接反，读数方向会反过来（右旋变小）；若只接两端不接抽头，
 *   读数恒为 0 或恒为满量程。
 *
 *   这四个引脚的**声明与电气配置都在 CubeMX 里**（见 PID_Pendulum.ioc，
 *   信号为 ADCx_IN2..IN5）。模拟输入模式的引脚配置由生成的
 *   HAL_ADC_MspInit() 完成，本模块不碰 GPIO。
 * ────────────────────────────────────────────────────────────────────
 *
 * ── 为什么是「先采样、再读缓存」，而不是「每次读都转换一次」 ──────────
 *   4 路共用 ADC2 的**一个**数据寄存器（DR），而且 ADC 是「一问一答」的：
 *   启动一次只得到一路的结果。若让 bsp_pot_raw() 自己触发转换，
 *   连续读 4 次就是 4 次硬件动作，而且 4 个值来自 4 个不同瞬间
 *   （旋钮在转时会出现「四个值来自四个瞬间」的撕裂）。
 *
 *   所以接口拆成两半：
 *     bsp_pot_sample()  循环 4 次、把 4 路各采一遍写进缓存（约 23 µs）；
 *     bsp_pot_raw()     只是读缓存，纯内存访问，随便调。
 *   一次采样得到一个**时间上彼此靠近**的快照，读的时候也不再产生副作用。
 *
 * ── 使用方式 ────────────────────────────────────────────────────────
 *   在主循环里按固定周期采样（电位器是人手拧的，20 ms 一次足够）：
 *
 *     if (bsp_pot_sample() == ERR_OK) {
 *         uint16_t v = bsp_pot_raw(POT_ID_RP1);      // 0~4095
 *     }
 *
 *   **不要把它挂到 1 ms 节拍上**：一次采样要阻塞约 23 µs，放进中断
 *   违反「快进快出」（见 bsp/tick/bsp_tick.h 的说明）。
 */

#ifndef BSP_POT_H
#define BSP_POT_H

#include <stdint.h>

#include "error.h"

/** 电位器编号。顺序即 RP1~RP4，与 ADC2 的 rank 1~4 一一对应。 */
typedef enum {
    POT_ID_RP1 = 0,
    POT_ID_RP2,
    POT_ID_RP3,
    POT_ID_RP4,
    POT_ID_COUNT,               /**< 电位器总数，也用作遍历上界 */
} pot_id_t;

/** ADC 满量程（12 位） */
#define BSP_POT_RAW_MAX         (4095U)

/** 参考电压（mV）。VDDA = 3.3 V，与板子供电一致。 */
#define BSP_POT_VREF_MV         (3300U)

/**
 * @brief 初始化 ADC2 并做自校准。
 *
 * @return error_t ERR_OK 成功；ERR_NOT_READY 校准或启动失败
 *
 * @note 必须在 CubeMX 生成的 MX_ADC2_Init() **之后**调用。
 *
 * @note 自校准（HAL_ADCEx_Calibration_Start）在 F1 上是**必须**的：
 *       它测量芯片内部的电容失配并写入校准寄存器，不做的话读数会有
 *       几十个 LSB 的固定偏差，而且随温度漂移。这一步容易漏，
 *       漏了的现象是「能读、但数值不准」——不会报任何错。
 */
error_t bsp_pot_init(void);

/**
 * @brief 触发一次 4 通道扫描，把结果写进缓存。
 *
 * @return error_t ERR_OK 成功；ERR_NOT_READY ADC 启动失败；
 *                 ERR_TIMEOUT 等待某一路转换结束超时
 *
 * @note 阻塞约 23 µs（4 路 × (55.5 + 12.5) 个 ADC 周期 @ 12 MHz）。
 *       在**主循环**里调用，不要放进中断。
 */
error_t bsp_pot_sample(void);

/**
 * @brief 取上一次采样得到的原始值。
 *
 * @param id 电位器编号
 *
 * @return uint16_t 0~4095；id 越界时返回 0
 *
 * @note 读的是 bsp_pot_sample() 留下的缓存，不触发任何转换。
 */
uint16_t bsp_pot_raw(pot_id_t id);

/**
 * @brief 取上一次采样折算出的电压。
 *
 * @param id 电位器编号
 *
 * @return uint16_t 0~3300（mV）；id 越界时返回 0
 *
 * @note 用于验证接线：旋到两端应分别接近 0 mV 和 3300 mV。
 *       中间值不线性通常不是代码问题，而是电位器两端的供电没有接好。
 */
uint16_t bsp_pot_millivolt(pot_id_t id);

#endif /* BSP_POT_H */
