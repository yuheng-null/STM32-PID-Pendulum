/**
 * @file    bsp_tick.h
 * @brief   板级系统节拍：TIM1 每 1 ms 产生一次中断。
 *
 * 为什么单独抽出一层，而不是把它塞进按键模块里：
 *   「1 ms 节拍」是整个系统的基础设施，不是某一个模块的私产。
 *   按键拿它做消抖采样，以后的 PID 控制环、传感器采样、软件定时器
 *   都要挂到同一个节拍上。如果 TIM1 被按键模块独占，第二个需要节拍的
 *   模块就只能去抢别的定时器，或者把按键模块拆开——都是不必要的返工。
 *
 * 职责划分：
 *   · TIM1 的**配置**（分频、重装值、中断使能）由 CubeMX 生成，
 *     见 Core/Src/main.c 的 MX_TIM1_Init() 与 stm32f1xx_hal_msp.c 的
 *     HAL_TIM_Base_MspInit()。参数在 PID_Pendulum.ioc 里：
 *     PSC=71 / ARR=999 → 72MHz/72/1000 = 1 kHz。
 *   · 本模块只负责**启动**它，以及在中断里把节拍分发给注册的任务。
 *
 * 用法：
 *   MX_TIM1_Init();                      // CubeMX 生成，通常在 main 里
 *   bsp_tick_init();                     // 启动计时
 *   bsp_tick_register(my_1ms_task);      // 挂自己的 1ms 任务
 *
 * 被调用的任务运行在**中断上下文**里，必须遵守「快进快出」：
 * 不许阻塞、不许长循环、不许调用 HAL_Delay / 刷屏 / 打印。
 */

#ifndef BSP_TICK_H
#define BSP_TICK_H

#include <stdint.h>

#include "error.h"

/** 最多允许挂多少个 1 ms 任务 */
#define BSP_TICK_MAX_HANDLERS       (4U)

/**
 * 节拍周期（毫秒）。
 * 必须与 .ioc 里 TIM1 的 PSC/ARR 保持一致——改定时器参数时同步改这里，
 * 否则所有按节拍数换算时间的模块（比如按键消抖）都会算错。
 */
#define BSP_TICK_PERIOD_MS          (1U)

/** 1 ms 周期任务的函数原型 */
typedef void (*bsp_tick_fn_t)(void);

/**
 * @brief 启动 1 ms 节拍。
 *
 * @return error_t ERR_OK 成功；ERR_NOT_READY HAL 启动失败
 *
 * @note 必须在 CubeMX 生成的 MX_TIM1_Init() 之后、bsp_tick_register() 之前调用。
 */
error_t bsp_tick_init(void);

/**
 * @brief 注册一个 1 ms 周期任务。
 *
 * @param fn 任务函数（运行在中断上下文，必须快进快出）
 *
 * @return error_t ERR_OK 成功；ERR_INVALID_PARAM fn 为空；
 *                 ERR_OVERFLOW 槽位已满（见 BSP_TICK_MAX_HANDLERS）
 */
error_t bsp_tick_register(bsp_tick_fn_t fn);

#endif /* BSP_TICK_H */
