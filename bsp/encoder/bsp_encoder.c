/**
 * @file    bsp_encoder.c
 * @brief   编码器接口的实现（TIM3 编码器模式，只读不清零 + 模 2^16 求增量）。
 */

#include "bsp_encoder.h"

#include "main.h"                   /* 借它引入 stm32f1xx_hal.h */

/*
 * TIM3 的句柄由 CubeMX 生成在 main.c 里（`TIM_HandleTypeDef htim3;`），
 * main.h 没有为它生成 extern，所以这里自己声明一份。
 */
extern TIM_HandleTypeDef htim3;

/* ----------------------------------------------------------------- 状态 */

/** 上一次读到的计数值（游标）。初始化为开机时的 CNT，见 init。 */
static uint16_t s_last = 0U;

/** 自上次 reset 以来的累计边沿数 */
static int32_t s_total = 0;

/** 最近一次 update 得到的增量 */
static int16_t s_delta = 0;

/** 是否把增量取反（A/B 相接反时的软件兜底） */
static bool s_invert = false;

/** 是否已初始化 */
static bool s_ready = false;

/* ----------------------------------------------------------------- 实现 */

error_t bsp_encoder_init(void)
{
    s_ready  = false;
    s_total  = 0;
    s_delta  = 0;
    s_invert = false;

    /*
     * ⚠️ 通道参数必须是 TIM_CHANNEL_ALL。
     *
     * HAL_TIM_Encoder_Start() 内部按 Channel 分支：传 TIM_CHANNEL_1 只
     * 使能 CC1E，传 TIM_CHANNEL_2 只使能 CC2E，**只有落到 default 分支
     * 才会两个都使能**。TI12 四倍频需要 CH1、CH2 两路输入都工作，
     * 传单个通道会静默地少计一半甚至完全不计数——编译和运行都不报错。
     */
    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
        return ERR_NOT_READY;
    }

    /*
     * 把游标对齐到当前 CNT。
     * 不清零 CNT——见头文件里「为什么不清零计数器」。
     */
    s_last = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

    s_ready = true;

    return ERR_OK;
}

error_t bsp_encoder_update(void)
{
    if (!s_ready) {
        return ERR_NOT_INITIALIZED;
    }

    const uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

    /*
     * 模 2^16 相减：先在 16 位无符号里环绕相减，再重新解释成有符号。
     * 这样 0xFFFF → 0x0000 的回绕会自然算成 +1，而不是 -65535。
     *
     * ⚠️ 不能写成 (int16_t)now - (int16_t)s_last：
     *    两个值各自转成有符号后再在 int 里相减，跨半区（32767/32768）
     *    时会得到完全错误的结果。
     */
    int32_t d = (int32_t)(int16_t)(uint16_t)(now - s_last);

    s_last = now;

    if (s_invert) {
        d = -d;
    }

    s_total += d;
    s_delta  = (int16_t)d;

    return ERR_OK;
}

int32_t bsp_encoder_total(void)
{
    return s_total;
}

int16_t bsp_encoder_delta(void)
{
    return s_delta;
}

error_t bsp_encoder_reset(void)
{
    if (!s_ready) {
        return ERR_NOT_INITIALIZED;
    }

    /*
     * 这里要重新对齐游标，而不是只把 s_total 置 0：
     * 若不清 s_last，下一次 update 会把「上次采样到这次清零之间」
     * 累积的边沿也算进来，位置就会凭空多出一截。
     */
    s_last  = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    s_total = 0;
    s_delta = 0;

    return ERR_OK;
}

error_t bsp_encoder_set_invert(bool invert)
{
    if (!s_ready) {
        return ERR_NOT_INITIALIZED;
    }

    s_invert = invert;

    return ERR_OK;
}
