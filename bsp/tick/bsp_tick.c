/**
 * @file    bsp_tick.c
 * @brief   板级系统节拍实现（TIM1，1 ms 更新中断）。
 *
 * ── 1 ms 是怎么来的 ──────────────────────────────────────────────────
 *   TIM1 挂在 APB2 上。时钟树里 APB2 预分频 = 1（RCC_HCLK_DIV1），
 *   而 STM32 的规则是：APB 预分频为 1 时定时器时钟 = PCLKx；
 *   预分频大于 1 时定时器时钟 = 2 × PCLKx。
 *   所以 TIM1 的时钟 = PCLK2 = 72 MHz。
 *
 *       72 000 000 Hz ÷ (PSC+1) ÷ (ARR+1) = 中断频率
 *       72 000 000     ÷   72    ÷  1000  = 1000 Hz = 1 kHz = 1 ms
 *
 *   这两个值配在 .ioc 里（Parameter Settings），由 CubeMX 生成到
 *   Core/Src/main.c 的 MX_TIM1_Init()：
 *       htim1.Init.Prescaler = 71;
 *       htim1.Init.Period    = 999;
 *   中断本身的使能（NVIC）在 Core/Src/stm32f1xx_hal_msp.c 的
 *   HAL_TIM_Base_MspInit() 里。
 *
 *   注意 ARR 是 1000-1：计数器从 0 数到 ARR，数 999 共 1000 个数。
 *   少减这个 1 是最常见的低级错误。
 * ─────────────────────────────────────────────────────────────────────
 */

#include "bsp_tick.h"

#include "main.h"                   /* 借它引入 stm32f1xx_hal.h */

/*
 * TIM1 的句柄由 CubeMX 生成在 main.c 里（`TIM_HandleTypeDef htim1;`），
 * 但 main.h 没有为它生成 extern 声明，所以这里自己声明一份。
 * 类型和符号名都由 CubeMX 决定，因此这行必须与 main.c 保持一致。
 */
extern TIM_HandleTypeDef htim1;

/* ----------------------------------------------------------------- 状态 */

static bsp_tick_fn_t s_handlers[BSP_TICK_MAX_HANDLERS];
static volatile uint8_t s_handler_count = 0U;

/* ----------------------------------------------------------------- 实现 */

error_t bsp_tick_init(void)
{
    /*
     * Start_IT 才会使能更新中断（UIE 位）。光有 MX_TIM1_Init() 只是
     * 把参数写进寄存器，定时器还没跑，也不会进中断。
     */
    if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK) {
        return ERR_NOT_READY;
    }

    return ERR_OK;
}

error_t bsp_tick_register(bsp_tick_fn_t fn)
{
    if (fn == NULL) {
        return ERR_INVALID_PARAM;
    }
    if (s_handler_count >= BSP_TICK_MAX_HANDLERS) {
        return ERR_OVERFLOW;
    }

    /*
     * 顺序很重要：先写函数指针，再递增计数。
     * 此时节拍中断可能已经在跑了，如果反过来（先加计数再写指针），
     * 中断会读到 count 已经 +1 但槽位还是 NULL 的中间状态。
     * 现在的顺序下，中断要么看到旧计数（不含这个新任务），
     * 要么看到新计数且指针已就绪，不存在读到空指针的时刻。
     */
    s_handlers[s_handler_count] = fn;
    __DMB();                        /* 保证上面的写在下面之前对中断可见 */
    s_handler_count++;
    return ERR_OK;
}

/**
 * @brief HAL 的定时器溢出回调。
 *
 * 调用链：CubeMX 生成的 TIM1_UP_IRQHandler()
 *           → HAL_TIM_IRQHandler(&htim1)
 *             → 本函数
 *
 * 这个函数是 HAL 定义的 __weak 符号，整个工程只能有一份非弱定义。
 * 以后如果再启用别的定时器并使用 HAL 的溢出中断，要在下面的
 * 定时器判断里补一个 else if，不能另写一个同名函数。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM1) {
        return;                     /* 不是本模块的定时器，不处理 */
    }

    const uint8_t count = s_handler_count;   /* 读一次，避免循环中它变化 */

    for (uint8_t i = 0U; i < count; i++) {
        if (s_handlers[i] != NULL) {
            s_handlers[i]();
        }
    }
}
