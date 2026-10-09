/**
 * @file    bsp_angle.c
 * @brief   角度传感器的实现（ADC1 单通道轮询，PB0 = ADC1_IN8）。
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
 * 转换的等待上限（毫秒）。
 *
 * 一次转换只要约 5.7 µs，正常情况下这个值永远不会触发。
 * 给 2 ms 是为了在「ADC 时钟被关掉」这类真故障时能返回错误而不是死等
 * （与 bsp_pot 同样的理由）。
 */
#define BSP_ANGLE_POLL_TIMEOUT_MS   (2U)

/* ----------------------------------------------------------------- 状态 */

/** 上一次采样得到的原始值 */
static uint16_t s_angle_raw = 0U;

/** 是否已成功初始化，防止未校准就采样 */
static bool s_ready = false;

/* ----------------------------------------------------------------- 实现 */

error_t bsp_angle_init(void)
{
    s_ready     = false;
    s_angle_raw = 0U;

    /*
     * F1 的 ADC 必须先自校准再使用，且校准要求 ADC 处于关闭状态
     * （ADON=0）——MX_ADC1_Init() 刚跑完时正好满足。
     * 这一步容易漏，漏了的现象是「能读、但数值有几十个 LSB 的固定偏差」。
     */
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
        return ERR_NOT_READY;
    }

    s_ready = true;

    return ERR_OK;
}

error_t bsp_angle_sample(void)
{
    if (!s_ready) {
        return ERR_NOT_INITIALIZED;
    }

    /*
     * 这里**不需要**像 bsp_pot 那样每次改通道：本模块只用一个固定通道
     * （IN8），CubeMX 生成的 MX_ADC1_Init() 已经把 rank 1 配好了，
     * 序列长度是 1、扫描模式关闭，所以「一次转换 = 一路」，EOC 在本次
     * 转换结束时就置位，可以直接轮询读 DR。
     *
     * （相比之下 bsp_pot 要在 4 个通道间切换，那是它每次都要
     *   HAL_ADC_ConfigChannel() 的原因。）
     */
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        return ERR_NOT_READY;
    }

    if (HAL_ADC_PollForConversion(&hadc1, BSP_ANGLE_POLL_TIMEOUT_MS) != HAL_OK) {
        (void)HAL_ADC_Stop(&hadc1);
        return ERR_TIMEOUT;
    }

    /* 读 DR 取回结果，同时自动清掉 EOC */
    s_angle_raw = (uint16_t)HAL_ADC_GetValue(&hadc1);

    /* 关掉 ADC，下次采样重新 Start */
    (void)HAL_ADC_Stop(&hadc1);

    return ERR_OK;
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
