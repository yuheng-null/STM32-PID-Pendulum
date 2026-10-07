/**
 * @file    bsp_pot.c
 * @brief   板载 4 路电位器的实现（ADC2，一次转换只采一路）。
 *
 * ── 为什么用 ADC2 而不是 ADC1 ────────────────────────────────────────
 *   ADC2 的 IN2~IN5 与 ADC1 是**同一组引脚**（PA2~PA5），换外设不影响接线。
 *   把 ADC1 空出来留给后面的模块。
 *   注意两者的 ADC 预分频器是共用的（F103 只有一个 ADCPRE），
 *   所以这一改动不影响 ADC 时钟。
 * ────────────────────────────────────────────────────────────────────
 *
 * ── 为什么不用「扫描模式 + 一次读完 4 路」，而要一路一路来 ────────────
 *   这是本模块最值得记的一条，也是走了弯路才弄明白的。
 *
 *   F103 的 ADC 在**扫描模式**下，SR.EOC 不是每转换完一路就置位一次，
 *   而是**整组（整个序列）转换结束后才置位一次**：
 *     · RM0008 对 SR.EOC 的定义是「end of a **group** channel conversion」；
 *     · HAL 源码 stm32f1xx_hal_adc.c 里也写着
 *       "As flag EOC is not set after each conversion"。
 *   后果是：DR 里只有**最后一路**的结果，而 EOC 只能告诉你「整组转完了」。
 *   想在扫描模式下逐路拿到数据，**硬件上只有 DMA 一条路**——RM0008 明确写
 *   "When using scan mode, DMA bit must be set…"。
 *
 *   而 STM32F103 的 **ADC2 没有 DMA 请求**（RM0008 的 DMA1 请求表里
 *   DMA1 Channel 1 只挂了 ADC1；C8T6 又是中容量，没有 DMA2），
 *   所以 ADC2 上根本用不了扫描模式。
 *
 *   于是改成：**序列长度设为 1，每次转换只包含一路**。
 *   这样「一组」就只有一次转换，EOC 恢复成「这次转换结束就置位」的正常语义，
 *   逐路读 DR 才成立。读之前用 HAL_ADC_ConfigChannel() 把 rank 1 切到目标通道。
 *
 *   参数在 .ioc 里：12 位、**扫描模式关闭**、单次转换、序列长度 1、
 *   采样时间 55.5 周期。ADC 时钟 = PCLK2/6 = 12 MHz（F103 的 ADCCLK 上限 14 MHz），
 *   一路耗时 (55.5 + 12.5) / 12 MHz ≈ 5.67 µs，4 路约 23 µs。
 *
 * ── 为什么这里可以用 HAL_ADC_PollForConversion ──────────────────────
 *   HAL 的这个函数内部分两条路径：
 *       if (SCAN 清零 && SQR1.L 清零)  → 真正等 EOC 标志
 *       else                          → 只空转一段最坏情况时间，不查任何标志
 *   本模块的配置（扫描关闭 + 序列长度 1）正好命中第一条，所以它是可靠的。
 *   反过来说，**扫描模式下它是不可用的**——这正是上面那个坑的一半。
 */

#include "bsp_pot.h"

#include <stdbool.h>

#include "main.h"                   /* 借它引入 stm32f1xx_hal.h */

/*
 * ADC2 的句柄由 CubeMX 生成在 main.c 里（`ADC_HandleTypeDef hadc2;`），
 * 但 main.h 没有为它生成 extern 声明，所以这里自己声明一份。
 * 与 bsp/tick/bsp_tick.c 里声明 htim1 是同样的做法。
 */
extern ADC_HandleTypeDef hadc2;

/* ----------------------------------------------------------------- 配置 */

/*
 * 每个电位器接在 ADC2 的哪个通道上。
 *
 * CubeMX 对 ADC 通道**不生成引脚宏**（不像 GPIO 会生成 KEY1_Pin 那样），
 * 所以这张对应表必须写在 BSP 里——而这正是 BSP 层的职责：
 * 「这块板子上，什么接在什么脚上」。
 *
 * ⚠️ 改接线时要同步改这里，并同步改 PID_Pendulum.ioc 里 PA2~PA5 的信号。
 *    目前与 .ioc 的对应关系：
 *      PA2 → ADC2_IN2   PA3 → ADC2_IN3   PA4 → ADC2_IN4   PA5 → ADC2_IN5
 */
static const uint32_t s_pot_channel[POT_ID_COUNT] = {
    ADC_CHANNEL_2,      /* RP1 ── PA2 */
    ADC_CHANNEL_3,      /* RP2 ── PA3 */
    ADC_CHANNEL_4,      /* RP3 ── PA4 */
    ADC_CHANNEL_5,      /* RP4 ── PA5 */
};

/**
 * 单次转换的等待上限（毫秒）。
 *
 * 一路转换只要约 5.67 µs，正常情况下这个值永远不会触发。
 * 给 2 ms 是为了在「ADC 时钟被关掉」这类真故障时能返回错误，
 * 而不是死等——这也是用 HAL_ADC_PollForConversion 而不是自己写
 * 自旋计数循环的原因：它自带超时。
 */
#define BSP_POT_POLL_TIMEOUT_MS     (2U)

/* ----------------------------------------------------------------- 状态 */

/** 上一次采样的原始值（0~4095） */
static uint16_t s_pot_raw[POT_ID_COUNT];

/** 是否已成功初始化，防止未校准就采样 */
static bool s_pot_ready = false;

/* ----------------------------------------------------------------- 实现 */

error_t bsp_pot_init(void)
{
    s_pot_ready = false;

    for (uint8_t i = 0U; i < (uint8_t)POT_ID_COUNT; i++) {
        s_pot_raw[i] = 0U;
    }

    /*
     * F1 的 ADC 必须先自校准再使用。校准要求 ADC 处于关闭状态
     * （ADON=0），MX_ADC2_Init() 刚跑完时正好满足，所以这里第一件事
     * 就是它。
     *
     * 校准过程由硬件完成，内部约 83 个 ADC 周期，函数会等它结束。
     * 漏掉这一步的现象是「能读、但数值有几十个 LSB 的固定偏差」，
     * 而且随温度漂移——不会报任何错。
     */
    if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK) {
        return ERR_NOT_READY;
    }

    s_pot_ready = true;

    return ERR_OK;
}

error_t bsp_pot_sample(void)
{
    if (!s_pot_ready) {
        return ERR_NOT_INITIALIZED;
    }

    for (uint8_t i = 0U; i < (uint8_t)POT_ID_COUNT; i++) {
        /*
         * 把「序列里的第 1 格」换成目标通道。序列长度是 1，
         * 所以换掉 rank 1 就等于换掉整个序列。
         *
         * 必须在 ADC 关闭时配置——上一路的 HAL_ADC_Stop() 已经保证了这点。
         */
        ADC_ChannelConfTypeDef cfg = {0};
        cfg.Channel      = s_pot_channel[i];
        cfg.Rank         = ADC_REGULAR_RANK_1;
        cfg.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;

        if (HAL_ADC_ConfigChannel(&hadc2, &cfg) != HAL_OK) {
            return ERR_NOT_READY;
        }

        if (HAL_ADC_Start(&hadc2) != HAL_OK) {
            return ERR_NOT_READY;
        }

        if (HAL_ADC_PollForConversion(&hadc2, BSP_POT_POLL_TIMEOUT_MS) != HAL_OK) {
            (void)HAL_ADC_Stop(&hadc2);
            return ERR_TIMEOUT;
        }

        /* 读 DR 取回结果，同时自动清掉 EOC */
        s_pot_raw[i] = (uint16_t)HAL_ADC_GetValue(&hadc2);

        /* 关掉 ADC，下一次循环才能改通道配置 */
        (void)HAL_ADC_Stop(&hadc2);
    }

    return ERR_OK;
}

uint16_t bsp_pot_raw(pot_id_t id)
{
    if ((uint32_t)id >= (uint32_t)POT_ID_COUNT) {
        return 0U;
    }

    /* 16 位半字访问在 Cortex-M3 上是单条指令，不会被采样过程撕裂 */
    return s_pot_raw[id];
}

uint16_t bsp_pot_millivolt(pot_id_t id)
{
    if ((uint32_t)id >= (uint32_t)POT_ID_COUNT) {
        return 0U;
    }

    /*
     * 满量程用 4095 而不是 4096：12 位 ADC 的输出范围是 0~4095，
     * 除 4096 会让旋到最大时只能得到 3299 mV，差 1 mV。
     * 先乘后除，中间结果最大 4095 × 3300 ≈ 1.35e7，uint32_t 装得下。
     */
    const uint32_t mv = ((uint32_t)s_pot_raw[id] * BSP_POT_VREF_MV) / BSP_POT_RAW_MAX;

    return (uint16_t)mv;
}
