/**
 * @file    pid.c
 * @brief   位置式 PID 的实现。设计理由全部写在 pid.h 里。
 */

#include "pid.h"

#include <stddef.h>     /* NULL。pid.h 只引 stdbool，NULL 得自己引 */

void pid_reset(pid_t *p)
{
    if (p == NULL) {
        return;
    }

    /* 只清运行时状态。kp/ki/kd 和四个限幅是「配置」，不是「状态」，
     * 清掉的话每次启动都要重新调参，显然不对。 */
    p->err0  = 0.0f;
    p->err1  = 0.0f;
    p->integ = 0.0f;
    p->out   = 0.0f;
}

void pid_update(pid_t *p)
{
    if (p == NULL) {
        return;
    }

    /* ① 误差。顺序不能反：先把 err0 挪到 err1，再用新值覆盖 err0。 */
    p->err1 = p->err0;
    p->err0 = p->target - p->actual;

    /* ② 积分。
     *
     * Ki==0 时清零而不是累加，理由见 pid.h 的说明（调试便利，
     * 避免调 Ki 时被之前积压的值踹一脚）。 */
    if (p->ki == 0.0f) {
        p->integ = 0.0f;
    } else {
        p->integ += p->err0;
    }

    /* ③ 夹积分（抗饱和）。
     *
     * ⚠️ 必须在算出 out **之前**夹，而且夹的是积分累加值本身、
     *    不是积分项。夹积分项（Ki·integ）看起来一样，其实不同：
     *    那样 Ki 一变限幅范围就跟着变，调参时行为会莫名其妙。
     *    夹累加值则与 Ki 无关，行为稳定。 */
    if (p->integ > p->integ_max) {
        p->integ = p->integ_max;
    }
    if (p->integ < p->integ_min) {
        p->integ = p->integ_min;
    }

    /* ④ 位置式 PID 公式。
     *
     * 微分项是 (e − e_prev) 的差分形式，不是 (e − e_prev)/Δt。
     * 见 pid.h：本实现不含时间维，调用周期变了 Kd 的实际强度也跟着变。 */
    p->out = (p->kp * p->err0)
           + (p->ki * p->integ)
           + (p->kd * (p->err0 - p->err1));

    /* ⑤ 输出限幅。注意 err1 上面已经更新过了，这里只动 out。 */
    if (p->out > p->out_max) {
        p->out = p->out_max;
    }
    if (p->out < p->out_min) {
        p->out = p->out_min;
    }
}
