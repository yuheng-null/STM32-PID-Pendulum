/**
 * @file    bsp_angle.c
 * @brief   角度传感器的实现（ADC1 单通道，PB0 = ADC1_IN8）。
 *
 * ── 为什么不用 HAL_ADC_Start / PollForConversion ────────────────────
 *   本模块要给 1 ms 中断里的控制环提供采样。而 HAL 的这两个函数都不能
 *   在那样高优先级的中断里跑：
 *
 *     HAL_ADC_Start()              内部是 ADC_Enable()，它等 ADRDY 时用
 *                                  HAL_GetTick() 做超时判断
 *     HAL_ADC_PollForConversion()  同上，等 EOC 也用 HAL_GetTick()
 *
 *   TIM1 节拍的抢占优先级（1）高于 SysTick（15），在节拍任务里 uwTick
 *   根本不涨。于是超时判断永远不成立 —— 一旦转换没有正常完成，
 *   等待就会**永久卡死**，整个系统从此失去节拍。
 *   （这与 bsp_pot 是同一条约束，但它俩的解法不同：bsp_pot 至今只在
 *    主循环里用，所以保持原样。本模块要进中断，必须换写法。）
 *
 *   所以这里绕开那两个函数，直接用寄存器：
 *
 *     启动：SET_BIT(CR2, SWSTART | EXTTRIG)
 *     就绪：__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC)
 *     取值：HAL_ADC_GetValue(&hadc1)        ← 它只是读 DR，中断安全
 *
 *   这三条都在 HAL 源码里核实过（见各处的引文）。
 * ────────────────────────────────────────────────────────────────────
 *
 * ── ⚠️ EXTTRIG 必须一起写，只写 SWSTART 会静默无效 ──────────────────
 *   stm32f1xx_hal_adc.c 的 HAL_ADC_Init() 里有这段注释（原文）：
 *
 *     "External trigger polarity (ADC_CR2_EXTTRIG) is set into
 *      HAL_ADC_Start_xxx functions because if set in this function,
 *      a conversion on injected group would start a conversion also
 *      on regular group after ADC enabling."
 *
 *   即：**EXTTRIG 不在 init 里置位**，HAL 特意留到 Start 时才置。
 *   而 F1 上「软件触发」的机制正是 EXTSEL=SWSTART(0b111) + EXTTRIG=1，
 *   然后置 SWSTART 位产生一次触发事件。少了 EXTTRIG，置 SWSTART
 *   不会启动任何转换 —— 不报错、不置标志、读数永远不变。
 *
 *   （EXTSEL 那一半是 init 配的：MX_ADC1_Init() 里
 *    `hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START`，
 *    它在 HAL_ADC_Init() 里被写进 CR2 的 EXTSEL 字段。）
 * ────────────────────────────────────────────────────────────────────
 *
 * ── ADC 常开，从不 Stop ─────────────────────────────────────────────
 *   原实现在每次采样的末尾调用 HAL_ADC_Stop()，下次再 Start。改成
 *   trigger/poll 之后不能这么做了：每次 Start 都要等一次 ADRDY，
 *   而那段等待在中断里不可用。
 *
 *   于是 init() 里**一次性打开 ADC 并始终保持**，之后只反复触发单次
 *   转换（ContinuousConvMode = DISABLE，所以置一次 SWSTART = 转一次）。
 *   代价是 ADC 一直耗电（约 1 mA 量级），对插电的倒立摆无所谓。
 * ────────────────────────────────────────────────────────────────────
 */

#include "bsp_angle.h"

#include <stdbool.h>

#include "main.h"                   /* 借它引入 stm32f1xx_hal.h */

/*
 * ADC1 的句柄由 CubeMX 生成在 main.c 里（`ADC_HandleTypeDef hadc1;`），
 * main.h 没有为它生成 extern 声明，所以这里自己声明一份。
 */
extern ADC_HandleTypeDef hadc1;

/* ----------------------------------------------------------------- 配置 */

/**
 * `bsp_angle_sample()` 的自旋上限（次）。
 *
 * 一次转换要 (55.5 采样 + 12.5 转换) = 68 个 ADC 周期 @ 12 MHz ≈ 5.7 µs，
 * 换算到 72 MHz 内核约 410 个时钟周期。下面这个循环每轮大约 10 个周期，
 * 给 2000 次 ≈ 20 000 周期 ≈ 280 µs，是正常耗时的 **约 50 倍**余量。
 *
 * 取「次数」而不是「毫秒」，正是为了不依赖 HAL_GetTick()。
 */
#define BSP_ANGLE_SPIN_LIMIT        (2000U)

/* ----------------------------------------------------------------- 状态 */

/** 上一次采样得到的原始值 */
static uint16_t s_angle_raw = 0U;

/** 是否已成功初始化，防止未校准就采样 */
static bool s_ready = false;

/**
 * 有没有一次「已触发、结果还没被取走」的转换。
 *
 * 作用是把 poll() 的语义收紧成「拿到一个新采样才回 ERR_OK」：
 * 没有它的话，同一个值会被反复当成新值报出去，控制环就无法区分
 * 「数据在正常更新」和「ADC 卡住了，一直在读旧值」。
 */
static bool s_armed = false;

/* ----------------------------------------------------------------- 实现 */

error_t bsp_angle_init(void)
{
    s_ready     = false;
    s_angle_raw = 0U;
    s_armed     = false;

    /*
     * F1 的 ADC 必须先自校准再使用，且校准要求 ADC 处于关闭状态
     * （ADON=0）——MX_ADC1_Init() 刚跑完时正好满足。
     * 这一步容易漏，漏了的现象是「能读、但数值有几十个 LSB 的固定偏差」。
     */
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
        return ERR_NOT_READY;
    }

    /*
     * 唯一一次走 HAL 启动：它内部会置 ADON、等 ADRDY 稳定、再置一次
     * SWSTART|EXTTRIG。这里在主线程，HAL_GetTick() 正常，所以能这么用。
     *
     * 之后再不调用 HAL_ADC_Stop()，ADC 就一直是开着的。
     *
     * 副作用：hadc1.State 会停在 HAL_ADC_STATE_REG_BUSY，因为 HAL 认为
     * 「正在转换、还没 Stop」。这是刻意的——本模块此后完全绕开 HAL 的
     * 转换接口，没有第二个调用者会去看这个状态。要恢复成 HAL 的用法，
     * 得先 HAL_ADC_Stop()，而那就把 ADC 关掉了。
     */
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        return ERR_NOT_READY;
    }

    /*
     * Start 顺带触发了第一次转换，把它等完读掉，让 init 返回时状态干净
     * （没有转换在飞、EOC 已清）。这里可以放心自旋：正在初始化，
     * 没别的事要做。
     */
    for (uint32_t i = 0U; i < BSP_ANGLE_SPIN_LIMIT; i++) {
        if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC)) {
            break;
        }
    }
    /* 读 DR 取值并让硬件自动清 EOC；这个值丢弃，只是要清干净状态 */
    (void)HAL_ADC_GetValue(&hadc1);

    s_ready = true;

    return ERR_OK;
}

error_t bsp_angle_trigger(void)
{
    if (!s_ready) {
        return ERR_NOT_INITIALIZED;
    }

    /*
     * 和 HAL_ADC_Start() 里最后那两行是同一件事、同两个位：
     *   EXTTRIG=1 让 EXTSEL 选中的触发源生效（见文件头：init 不置这一位）
     *   SWSTART=1 产生这次触发事件，硬件随即把它自己清零
     * 重复置位是幂等的，上次没取走的结果会被这次覆盖掉。
     */
    SET_BIT(hadc1.Instance->CR2, (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG));

    s_armed = true;

    return ERR_OK;
}

error_t bsp_angle_poll(void)
{
    if (!s_ready) {
        return ERR_NOT_INITIALIZED;
    }

    if (!s_armed) {
        return ERR_NOT_READY;       /* 没触发过，或上次结果已经取走了 */
    }

    /*
     * EOC 在「整组转换结束」时由硬件置位（本工程 NbrOfConversion=1，
     * 扫描关闭，所以一次转换 = 一组）。
     *
     * 注意这里**不能**用 HAL_ADC_PollForConversion()——它用 HAL_GetTick()
     * 做超时判断，而本函数会被 1 ms 中断调用，uwTick 在那种上下文里不涨。
     * 直接查标志位则没有这个问题。
     */
    if (!__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC)) {
        return ERR_NOT_READY;       /* 还没转完，下次再来问 */
    }

    /* 读 DR 取回结果。HAL 源码注释原文：
     *   "EOC flag is not cleared here by software because automatically
     *    cleared by hardware when reading register DR."
     * 所以读一次就够了，不需要再手工清标志。 */
    s_angle_raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
    s_armed     = false;            /* 这一份结果只能被取走一次 */

    return ERR_OK;
}

error_t bsp_angle_sample(void)
{
    if (!s_ready) {
        return ERR_NOT_INITIALIZED;
    }

    /* trigger + 有界自旋 poll。自旋而不是等 tick，理由见 BSP_ANGLE_SPIN_LIMIT */
    (void)bsp_angle_trigger();

    for (uint32_t i = 0U; i < BSP_ANGLE_SPIN_LIMIT; i++) {
        if (bsp_angle_poll() == ERR_OK) {
            return ERR_OK;
        }
    }

    return ERR_TIMEOUT;
}

uint16_t bsp_angle_raw(void)
{
    return s_angle_raw;
}

uint16_t bsp_angle_millivolt(void)
{
    /*
     * 满量程用 4095 而不是 4096：12 位 ADC 的输出范围是 0~4095，
     * 除 4096 会让最大读数只能得到 3299 mV，差 1 mV。
     * 先乘后除，中间结果最大 4095 × 3300 ≈ 1.35e7，uint32_t 装得下。
     */
    const uint32_t mv = ((uint32_t)s_angle_raw * BSP_ANGLE_VREF_MV) / BSP_ANGLE_RAW_MAX;

    return (uint16_t)mv;
}
