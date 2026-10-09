/**
 * @file    bsp_motor.c
 * @brief   板载直流电机的实现（TB6612FNG A 路 + TIM2_CH1 PWM）。
 */

#include "bsp_motor.h"

#include <stdbool.h>

#include "main.h"                   /* 借它引入 stm32f1xx_hal.h 与引脚宏 */
#include "stm32f1xx_hal.h"

/*
 * TIM2 的句柄由 CubeMX 生成在 main.c 里（`TIM_HandleTypeDef htim2;`），
 * 但 main.h 没有为它生成 extern 声明，所以这里自己声明一份。
 * 与 bsp_tick.c 声明 htim1、bsp_pot.c 声明 hadc2 是同样的做法。
 */
extern TIM_HandleTypeDef htim2;

/* ----------------------------------------------------------------- 配置 */

/**
 * 正转时两个方向脚的电平。
 *
 * 约定 duty > 0 为「正转」。TB6612 的 A 路：AIN1=0 / AIN2=1 时电机正转。
 * ⚠️ 若实测方向与此相反（正 duty 时转反了），把这两行对调即可，
 *    不必动接线，也不要改 PWM。
 */
#define BSP_MOTOR_FWD_AIN1      (GPIO_PIN_RESET)
#define BSP_MOTOR_FWD_AIN2      (GPIO_PIN_SET)

/* ----------------------------------------------------------------- 状态 */

/** 当前设定的 duty（已夹紧），供 bsp_motor_duty() 回读 */
static int16_t s_duty = 0;

/** 是否已成功初始化，防止未启动 PWM 就改 CCR */
static bool s_ready = false;

/* ----------------------------------------------------------------- 内部 */

/** 直接写方向脚，不做任何检查 */
static void motor_set_dir(GPIO_PinState ain1, GPIO_PinState ain2)
{
    HAL_GPIO_WritePin(MOTOR_AIN1_GPIO_Port, MOTOR_AIN1_Pin, ain1);
    HAL_GPIO_WritePin(MOTOR_AIN2_GPIO_Port, MOTOR_AIN2_Pin, ain2);
}

/** 直接写比较值（即占空比，量纲 0~100） */
static void motor_set_ccr(uint16_t ccr)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
}

/* ----------------------------------------------------------------- 实现 */

error_t bsp_motor_init(void)
{
    s_ready = false;
    s_duty  = 0;

    /*
     * 先把「停」的状态写下去，再启动 PWM —— 顺序不能反。
     * 反过来的话，PWM 启动的那一瞬间 CCR 里还是复位值 0，倒是同样安全；
     * 但若将来有人在 CubeMX 里把 Pulse 初值改成非 0，反序就会让电机
     * 在初始化中途窜一下。保持这个顺序，上电一定不转。
     */
    motor_set_dir(GPIO_PIN_RESET, GPIO_PIN_RESET);  /* AIN1=AIN2=0 → 滑行 */
    motor_set_ccr(0U);

    /*
     * Start_IT 之外必须显式 Start：MX_TIM2_Init() 只把参数写进寄存器，
     * 既没使能输出比较（CC1E）也没使能计数器（CEN），电机不会动。
     * 只调一次——重复调用会因通道状态已是 BUSY 而返回 HAL_ERROR。
     */
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) {
        return ERR_NOT_READY;
    }

    s_ready = true;

    return ERR_OK;
}

error_t bsp_motor_set_duty(int16_t duty)
{
    if (!s_ready) {
        return ERR_NOT_INITIALIZED;
    }

    /* 夹紧到 ±BSP_MOTOR_DUTY_MAX，理由见头文件的说明 */
    if (duty > (int16_t)BSP_MOTOR_DUTY_MAX) {
        duty = (int16_t)BSP_MOTOR_DUTY_MAX;
    } else if (duty < -(int16_t)BSP_MOTOR_DUTY_MAX) {
        duty = -(int16_t)BSP_MOTOR_DUTY_MAX;
    }

    /*
     * duty == 0 单独处理成「滑行」。若照搬「正负号决定方向」的写法，
     * 0 会落进正转分支（AIN1=0/AIN2=1），那不是干净的停止状态。
     */
    if (duty == 0) {
        motor_set_dir(GPIO_PIN_RESET, GPIO_PIN_RESET);
        motor_set_ccr(0U);
    } else if (duty > 0) {
        motor_set_dir(BSP_MOTOR_FWD_AIN1, BSP_MOTOR_FWD_AIN2);
        motor_set_ccr((uint16_t)duty);
    } else {
        motor_set_dir(BSP_MOTOR_FWD_AIN2, BSP_MOTOR_FWD_AIN1);   /* 对调 */
        motor_set_ccr((uint16_t)(-duty));
    }

    s_duty = duty;

    return ERR_OK;
}

error_t bsp_motor_coast(void)
{
    if (!s_ready) {
        return ERR_NOT_INITIALIZED;
    }

    motor_set_dir(GPIO_PIN_RESET, GPIO_PIN_RESET);
    motor_set_ccr(0U);
    s_duty = 0;

    return ERR_OK;
}

error_t bsp_motor_brake(void)
{
    if (!s_ready) {
        return ERR_NOT_INITIALIZED;
    }

    motor_set_dir(GPIO_PIN_SET, GPIO_PIN_SET);
    motor_set_ccr(0U);
    s_duty = 0;

    return ERR_OK;
}

int16_t bsp_motor_duty(void)
{
    return s_duty;
}
