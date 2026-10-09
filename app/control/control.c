/**
 * @file    control.c
 * @brief   双环 PID 控制器的实现。设计动机全部写在 control.h 里。
 */

#include "control.h"

#include <stddef.h>

#include "bsp_angle.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "pid.h"

/* ----------------------------------------------------------------- 配置 */

/**
 * 角度环（内环）周期，毫秒。
 *
 * 5 ms 来自官方工程。它决定了角度环的带宽，而角度环必须比位置环
 * 快一个数量级以上，见 control.h。
 *
 * ⚠️ 改这个值等于改 Ki/Kd 的实际强度（pid.c 里的公式不含 Δt），
 *    改了要重新整定。
 */
#define CONTROL_ANGLE_PERIOD_MS     (5U)

/**
 * 位置环（外环）周期，毫秒。
 *
 * 50 ms 来自官方工程。比例是 10:1。
 */
#define CONTROL_POS_PERIOD_MS       (50U)

/**
 * 位置环输出的上限（ADC 码），**不受 PWM 量程影响**。
 *
 * 这一项限的是「允许把内环的目标角度偏移多少」，单位是角度传感器的
 * 码，跟电机 PWM 的范围是两个独立的量。官方取 ±100 码 ≈ ±8.2°
 * （本套件 12.22 码/度）。含义是：最多允许摆杆偏离竖直 8° 来换取
 * 横杆的移动速度 —— 偏得再多就该优先保摆杆了。
 */
#define CONTROL_POS_OUT_LIMIT       (100.0f)

/**
 * 角度环的积分限幅（ADU·步，即「码 × 累加次数」）。
 *
 * 取 out_max / ki = 1800 / 0.162 ≈ 11111 向下圆整。含义是：
 * **积分项单独就能顶满输出，再攒下去没有意义**，只会拖慢退饱和。
 */
#define CONTROL_ANGLE_INTEG_LIMIT   (10000.0f)

/**
 * 位置环的积分限幅。取 out_max / ki = 100 / 0.01 = 10000。
 *
 * ⚠️ 这个数**必须跟着 Ki 走**：Ki 从 0.18 改回官方原值 0.01 之后，
 *    这个上限也得跟着放大 —— 否则 `ki × integ` 最多只有 0.01 × 500 = 5 码，
 *    等于**把积分项悄悄废掉了**，而且不报任何错，只是"积分好像没用"。
 *    （这类"改了 A 忘了跟着改 B、且静默失效"的坑，本工程踩过不止一次。）
 */
#define CONTROL_POS_INTEG_LIMIT     (10000.0f)

/*
 * ── 默认增益：只有**角度环**要 ×18，位置环照抄官方 ────────────────
 *
 * 官方 00-PID综合测试程序 V1.0/V1.1 的 Mode3（倒立摆模式）参数是：
 *
 *     角度环  Kp=0.25  Ki=0.009  Kd=0.41    OutMax=100  OutMin=-100
 *     位置环  Kp=0.52  Ki=0.01   Kd=4.56    OutMax=100  OutMin=-100
 *     OFFSET_PWM = 5
 *
 * ── 角度环为什么 ×18 ────────────────────────────────────────────
 * 官方的 PWM 时基是 ARR=99（`PWM.c` 里 `TIM_Period = 100 - 1`），
 * 占空比满量程对应 100 个计数；本工程 ARR=1799，满量程 1800 个计数。
 * 真正决定物理强度的是**占空比**而不是裸计数：
 *
 *     官方：duty = Kp_official × e / 100
 *     本机：duty = Kp_ours     × e / 1800
 *     两者 duty 相同 ⟹ Kp_ours = Kp_official × 18
 *
 * ── 位置环为什么**不** ×18 ──────────────────────────────────────
 * 位置环的输出**不喂 PWM**，它加在内环的**角度目标**上（见下面
 * `s_angle_pid.target = center_angle - s_pos_pid.out`），单位是 ADC 码。
 * 官方的位置环输出限幅也是 ±100 码，两边**单位相同、量程相同**，
 * 所以增益原样照抄。
 *
 * ⚠️ **2026-10-10 真机实测证实了这一点。** 最初把位置环也乘了 18
 *    （PKP=9.36），现象是：摆杆能立住，但**一直在满量程抖**——
 *    位置误差只要 11 码就把外环顶到 ±100 限幅，外环退化成 bang-bang。
 *    实测数据（同一段 12 s 轨迹，只改这一组增益）：
 *
 *        PKP=9.36（错误）   |ANGLE-2086| 中位 126 码 ≈10°；|PWM| 中位 412，且持续打满
 *        PKP=0.52（正确）   |ANGLE-2086| 中位  11 码 ≈0.9°；|PWM| 中位 150，打满 0%
 *
 *    差了整整一个数量级。**「哪些量要跟着量程换算」这条规则，必须逐个
 *    增益去判断它的输出喂给谁，不能整组一起乘。**
 *
 * ⚠️ 角度环那三个仍然只是**整定的起点**。
 * ────────────────────────────────────────────────────────────────
 */
#define CONTROL_DEFAULT_AKP         (4.50f)     /* 0.25  × 18 */
#define CONTROL_DEFAULT_AKI         (0.162f)    /* 0.009 × 18 */
/*
 * 2026-10-10 实测：AKD 从 7.38（= 0.41×18）提到 12.0，摆杆的抖动变小。
 * 同一次测量里交错重复两次（OFFSET 都固定在 30）：
 *
 *     AKD= 7.38 : dev 标准差 10.2 / 9.1   PWM 相邻跳变 63 / 59
 *     AKD=12.00 : dev 标准差  7.8 / 8.2   PWM 相邻跳变 91 / 91
 *
 * 两次都更小，所以是有效果的（幅度不大，约 15~20%）。
 * ⚠️ 代价：PWM 的相邻跳变大了约 50% —— 微分项在放大 ADC 噪声
 *    （±8 码的 ADC 噪声 × Kd 就是几十码的 PWM 抖动）。可以接受，
 *    但如果以后发现电机发热或异响，**第一个要回退的就是它**。
 * ⚠️ 反方向试过了：AKD 降到 3.0 会让抖动**明显变大**
 *    （dev 标准差 12.1 / 12.4，两次都更差）——Kd 的阻尼是在干实事的，
 *    不要因为"它在放大噪声"就把它调小。
 */
#define CONTROL_DEFAULT_AKD         (12.0f)     /* 实测标定的值，见上 */
#define CONTROL_DEFAULT_PKP         (0.52f)     /* 官方原值，**不乘** */
#define CONTROL_DEFAULT_PKI         (0.01f)     /* 官方原值，**不乘** */
#define CONTROL_DEFAULT_PKD         (4.56f)     /* 官方原值，**不乘** */

/**
 * 静摩擦补偿（PWM 码）。
 *
 * 电机有静摩擦，占空比太小根本转不起来（本套件实测约 3% 才起转）。
 * 控制环在平衡点附近算出的输出经常只有零点几个百分点，这段时间电机
 * 是「死」的，表现为横杆在低频上小幅抖动、响应迟钝。
 *
 * 做法：输出非 0 时，按**输出的符号**再叠一个最小起转量。官方取 5
 * （量程 ±100 → 5% 占空比），换算到本机 = 90。
 *
 * ⚠️ 这个值的调法很别扭，两个方向都会坏：
 *      太小 → 横杆低频抖动（死区没被跨过去）
 *      太大 → 摆杆高频抖动（补偿过头，一直在过冲）
 *
 * ── 2026-10-10 实测把它从 90 降到 30 ────────────────────────────
 * 90 就是上面那个换算值，但真机上它偏大：`apply_motor()` 是「输出为正就
 * +offset、为负就 −offset」，所以平衡点附近**输出一过零，占空比就从 +90
 * 跳到 −90**（5% 的台阶）—— 而平衡时输出本来就在零附近来回穿，
 * 等于每穿一次就给电机一脚。
 *
 * 在**已经立住**的状态下运行中改这个参数（改的是 32 位对齐的 float，
 * 1 ms 中断里读是原子的），每个取值测 8 s、**交错顺序**、同一取值重复两次：
 *
 *     OFFSET=90 : dev 中位 14.5 / 13.0   PWM 标准差 161.0 / 161.5
 *     OFFSET=30 : dev 中位  5.0 /  9.5   PWM 标准差  78.7 / 100.0
 *     OFFSET=20 : dev 中位  9.5 /  9.0   PWM 标准差  95.6 /  92.4
 *     OFFSET= 0 : dev 中位 10.0          （反而不如 20~30：死区跨不过去）
 *
 * 结论：**90 明显偏大；20~30 是一个平台**，取 30（平台中间）。
 * ⚠️ 低值之间的差异已经淹在噪声里了（30 的两次是 5.0 和 9.5），
 *    再往细里抠没有意义。
 * ⚠️ 方法学教训：第一轮是**按从大到小的顺序**扫的，看起来"越扫越好"，
 *    其实混进了时间趋势。改成交错顺序 + 重复之后形状就变了。
 *    单次、单方向、不重复的扫描，在这种量级上分不出信号和漂移。
 */
#define CONTROL_DEFAULT_OFFSET      (30.0f)     /* 实测标定，见上 */

/**
 * 默认中心角度（ADC 码）—— 2026-10-09 在本台设备上实测。
 * ⚠️ 逐台标定，换设备必须重新测。官方标称 2050，正常范围 1900~2200。
 */
#define CONTROL_DEFAULT_CENTER      (2086U)

/**
 * 默认倒下判定半窗口（ADC 码）。官方取 500（约 ±41°）。
 * 只用于**运行中**检测「立着立着倒了」，见 control.h 的两窗口说明。
 */
#define CONTROL_DEFAULT_RANGE       (500U)

/**
 * 默认启动准入半窗口（ADC 码），约 ±12°。
 *
 * **必须显著小于 CONTROL_DEFAULT_RANGE，原因见 control.h**：
 * 本台实测「摆杆自由下垂」时盲区读数是 1883，而 CENTER 是 2086，
 * 相差 203 码。若启动窗口取到 ±203 以上，垂着的摆杆就会被放行。
 *
 * 取 150（≈12°）留了约 50 码余量，同时手扶摆杆的精度（±3°≈±37 码）
 * 完全够用 —— 手扶读数实测在 2080~2090。
 */
#define CONTROL_DEFAULT_START       (150U)

/*
 * ── 自动启摆（swing-up）的三组参数 ────────────────────────────────
 *
 * 算法与下面几个默认值都取自参考工程 `16-倒立摆-自动启摆` 的 1 ms 状态机
 * （状态 1 判定 / 21~24 与 31~34 两组脉冲 / 4 双环）。完整说明见 control.h。
 * ────────────────────────────────────────────────────────────────
 */

/**
 * 启摆脉冲占空比（PWM 码）。默认 630 = **参考的 35% × 18**。
 *
 * 参考的 `START_PWM = 35`，而它的 PWM 时基是 ARR=99（满量程 100），
 * 所以那是 **35% 占空比**；本工程满量程 1800，35% × 1800 = 630。
 * 与增益、OFFSET 用的是同一条 ×18 换算规则。
 *
 * ⚠️ 参考注释原文：「此值需要根据自己的设备对应更改，一般在 30~40 之间；
 *    力度太小摆杆始终摆不上去，力度太大摆杆经常摆过头」。
 *    所以它是**逐台标定量**，做成了可在线改的参数（`SET SWP`）。
 */
#define CONTROL_DEFAULT_SWP         (630.0f)    /* 35 × 18 */

/**
 * 启摆脉冲宽度（毫秒）。默认 100，对应参考的 `START_TIME = 100`。
 *
 * ⚠️ 参考注释：「太小则力度发不出来，太大则不利于共振启摆，一般 80~120」。
 */
#define CONTROL_DEFAULT_SWT         (100.0f)

/**
 * 判定相的采样间隔（毫秒）。对齐参考实现的 40 ms。
 *
 * 判定要连取 3 个点、看"中间点是不是极值"。间隔太小会被 ADC 噪声骗到；
 * 太大则要等摆幅已经很大才认得出顶点，白白少注能量。
 */
#define CONTROL_SWING_JUDGE_MS      (40U)

/**
 * 交棒判定要在 `START` 窗口内**连续**满足多少毫秒（见 swing_tick()）。
 *
 * 用「连续 N 毫秒」而不是「连续两次采样」：判定点每 40 ms 才有一个，
 * 而摆杆从竖直附近掠过时可能一次采样都落不进 `START`（±150）这个窄窗口，
 * 那就永远等不到交棒。改成每 1 ms 查一次、连续 5 ms 满足才认 ——
 * 既抓得住快掠，又不会被 ADC 的 ±8 码噪声骗到。
 */
#define CONTROL_SWING_CATCH_MS      (5U)

/**
 * 启摆超时（毫秒）。**这一条是本工程加的，参考实现没有。**
 *
 * 参考的状态机会一直泵能量、直到摆杆进窗口为止。无人看管时这不行：
 * 万一方向符号反了、或者 SWP 标定得不对，电机会以 35% 占空比无限冲击下去。
 * 30 秒对正常启摆是足够的（参考实测量级是几秒），到点就停机，
 * 并在 `swing_result` 里记成 `CONTROL_SWING_TIMEOUT`。
 */
#define CONTROL_SWING_TIMEOUT_MS    (30000U)

/* ----------------------------------------------------------------- 状态 */

static bool             s_initialized = false;
static control_state_t  s_state       = CONTROL_STATE_STOP;
static control_params_t s_params;
static control_status_t s_status;

static pid_t s_angle_pid;       /* 内环：角度 */
static pid_t s_pos_pid;         /* 外环：位置 */

static int32_t s_pos_target   = 0;      /* 位置环目标，单独存（int32 语义）*/
static uint8_t s_count_angle  = 0U;     /* 角度环分频计数 */
static uint8_t s_count_pos    = 0U;     /* 位置环分频计数 */

/* —— 自动启摆的子状态 ——
 *
 * 与参考实现的状态编号对应关系：
 *   SWING_PULSE1 / PULSE1_WAIT  ←→  21 / 22（或 31 / 32）
 *   SWING_PULSE2 / PULSE2_WAIT  ←→  23 / 24（或 33 / 34）
 *   SWING_JUDGE                 ←→  1
 */
typedef enum {
    SWING_JUDGE = 0,        /**< 判定相：等摆杆到顶点，或等它进窗口 */
    SWING_PULSE1,           /**< 打第一个脉冲 */
    SWING_PULSE1_WAIT,      /**< 第一个脉冲计时中 */
    SWING_PULSE2,           /**< 打第二个脉冲（方向与第一个相反）*/
    SWING_PULSE2_WAIT,      /**< 第二个脉冲计时中 */
} swing_phase_t;

static swing_phase_t s_swing_phase   = SWING_JUDGE;
static int8_t        s_swing_dir     = 1;    /* 第一个脉冲的方向；+1 对应参考的 21 */
static uint16_t      s_swing_timer   = 0U;   /* 脉冲倒计时，毫秒 */
static uint32_t      s_swing_elapsed = 0U;   /* 已启摆时间，毫秒（超时用）*/
static uint8_t       s_count_judge   = 0U;   /* 判定采样分频计数 */
static uint8_t       s_ang_count     = 0U;   /* 已攒够几个判定采样（不足 3 个不判定）*/
static uint8_t       s_swing_hits    = 0U;   /* 连续几个毫秒落在 START 窗口内（交棒用）*/
static uint16_t      s_ang[3]        = {0U, 0U, 0U};  /* 连续三次判定采样，[0] 最新 */

/* ----------------------------------------------------------------- 内部 */

/**
 * @brief 角度是否落在以中心为准、半宽为 half 的窗口内。
 *
 * 用 int32 算，因为 center - half 可能为负（center 小、half 大时），
 * 用 uint16 会让 [负, ...] 这种区间绕成一个巨大的正数。
 *
 * 两个窗口（启动 / 倒下）共用这一段比较，只是传的 half 不同，
 * 免得写两遍还写岔。理由见 control.h。
 */
static bool angle_in_window(uint16_t angle, uint16_t half)
{
    const int32_t lo = (int32_t)s_params.center_angle - (int32_t)half;
    const int32_t hi = (int32_t)s_params.center_angle + (int32_t)half;

    return ((int32_t)angle >= lo) && ((int32_t)angle <= hi);
}

/**
 * @brief 把角度环的输出（含静摩擦补偿）写到电机上。
 *
 * 补偿只叠在非零输出上，输出恰好为 0 时保持 0 —— 否则 pid 想让电机停，
 * 补偿却硬把它推起来，平衡点附近会一直抖。官方也是这么写的。
 */
static void apply_motor(float angle_out)
{
    int16_t duty;

    if (angle_out > 0.0f) {
        duty = (int16_t)(angle_out + s_params.pwm_offset);
    } else if (angle_out < 0.0f) {
        duty = (int16_t)(angle_out - s_params.pwm_offset);
    } else {
        duty = 0;
    }

    /* 叠加后可能略微超出 ±BSP_MOTOR_DUTY_MAX，bsp_motor_set_duty 会夹紧，
     * 所以这里不用自己判断（越界夹紧是驱动层的既定契约）。 */
    (void)bsp_motor_set_duty(duty);
    s_status.pwm = duty;
}

/* ----------------------------------------------------------------- 启摆 */

/**
 * @brief 按方向打一个启摆脉冲，并同步快照。
 *
 * 启摆**不叠静摩擦补偿**（不走 apply_motor）：脉冲本身就是"用力推一下"，
 * 再叠一个 offset 只会让标定多一个变量。
 */
static void swing_pulse(int8_t sign)
{
    const int16_t duty = (int16_t)((sign > 0) ? s_params.swing_pwm
                                              : -s_params.swing_pwm);

    (void)bsp_motor_set_duty(duty);
    s_status.pwm = duty;
}

/**
 * @brief 启摆完成：交棒给双环 PID。
 *
 * ⚠️ **这里不走 `control_start()`**，直接搭 RUN 状态。两个理由：
 *
 *   ① 交棒判定（swing_tick 里）用的就是 `start_range`，与 `control_start()`
 *      的准入条件**完全一样**，再查一遍是白做。
 *   ② 角度是**活的**：查完到调用之间它还会动。万一这一瞬间漂出窗口，
 *      `control_start()` 会返回 ERR_NOT_READY —— 于是"交棒"变成了"停机"，
 *      现象是「明明快成了却突然停下」，而且下一轮还会再演一遍。
 */
static void swing_finish(void)
{
    /*
     * 位置清零：对齐参考实现（它进 PID 时做 `Location = 0`）。
     * 启摆过程中横杆转到哪里是不确定的，把当前位置当成新的原点，位置环
     * 才有意义；否则位置误差一上来就是个几百上千码的台阶。
     */
    (void)bsp_encoder_reset();
    s_pos_target = 0;

    pid_reset(&s_angle_pid);
    pid_reset(&s_pos_pid);

    s_angle_pid.actual = (float)s_status.angle;
    s_angle_pid.target = (float)s_params.center_angle;
    s_pos_pid.actual   = 0.0f;
    s_pos_pid.target   = 0.0f;

    s_status.position = 0;
    s_status.angle_target = (float)s_params.center_angle;
    s_status.angle_out    = 0.0f;
    s_status.pos_out      = 0.0f;

    s_count_angle = 0U;
    s_count_pos   = 0U;

    s_state = CONTROL_STATE_RUN;
    /* 快照也要立刻跟上：否则紧接着的一次 STAT / 串口命令会读到上一拍的
     * 旧状态（control_tick 要等下一个 1 ms 才刷新它）。 */
    s_status.state        = CONTROL_STATE_RUN;
    s_status.swing_result = (uint8_t)CONTROL_SWING_OK;
}

/**
 * @brief 启摆判定相：每 CONTROL_SWING_JUDGE_MS 采一个点，看摆杆到顶点没有、
 *        或者已经进窗口了没有。
 *
 * 三个判定条件**逐字照抄参考实现**（它的状态 1），只把 CENTER / RANGE
 * 换成可在线改的参数。三个条件天然互斥：顶点判定要求三个点都在窗口**外**，
 * 进窗口判定要求两个点都在窗口**内**。
 *
 * @note 本实现比参考多一个 `s_ang_count`：每组脉冲打完之后，要**重新攒够
 *       3 个新鲜采样**才开始判定。参考那边 `Angle0/1/2` 是函数静态变量，
 *       回到状态 1 时不复位，所以它第一次判定会掺进脉冲之前的旧值——
 *       那是可能误判出"顶点"的。多攒 120 ms 只让每轮慢一点点，却把这一类
 *       误判整个去掉了。
 */
static void swing_judge(void)
{
    const int32_t c = (int32_t)s_params.center_angle;
    const int32_t r = (int32_t)s_params.center_range;

    s_count_judge++;
    if (s_count_judge < (uint8_t)CONTROL_SWING_JUDGE_MS) {
        return;
    }
    s_count_judge = 0U;

    /* 采样入队：[0] 最新，[2] 最旧 */
    s_ang[2] = s_ang[1];
    s_ang[1] = s_ang[0];
    s_ang[0] = s_status.angle;

    if (s_ang_count < 3U) {
        s_ang_count++;
        return;                         /* 还没攒够，不判定 */
    }

    const int32_t a0 = (int32_t)s_ang[0];
    const int32_t a1 = (int32_t)s_ang[1];
    const int32_t a2 = (int32_t)s_ang[2];

    /* —— 右侧顶点：三点都在右侧区间，且中间点是极小值 ——
     * 「中间点是极值」等价于「摆杆在这里掉头」，也就是摆动的顶点。 */
    if ((a0 > (c + r)) && (a1 > (c + r)) && (a2 > (c + r)) &&
        (a1 < a0) && (a1 < a2)) {
        s_swing_dir   = 1;              /* 对应参考的状态 21：先正向再反向 */
        s_swing_phase = SWING_PULSE1;
        return;
    }

    /* —— 左侧顶点：镜像 —— */
    if ((a0 < (c - r)) && (a1 < (c - r)) && (a2 < (c - r)) &&
        (a1 > a0) && (a1 > a2)) {
        s_swing_dir   = -1;             /* 对应参考的状态 31：先反向再正向 */
        s_swing_phase = SWING_PULSE1;
        return;
    }

    /*
     * ⚠️ 「进窗口就交棒」这一条**不在这里**，而是在 swing_tick() 里每 1 ms 查。
     *    两个原因，都不是风格问题：
     *
     *     ① **窗口必须用 START（±150），不能用 RANGE（±500）。** 参考实现
     *        用的是 CENTER_RANGE，但本台实测「摆杆自由下垂」读数是 1883，
     *        而 CENTER ± 500 = [1586, 2586] —— **1883 正好落在里面**。
     *        照搬的结果是：启摆刚开始判定就把垂着的摆杆判成"进窗口了"，
     *        直接交给双环 PID（还是默认增益），电机立刻猛冲。这个坑在
     *        control.h 的盲区一节早就写过，移植时却差点原样踩进去。
     *
     *     ② 判定点每 40 ms 才一个，而摆杆从竖直附近掠过时可能一个点都
     *        落不进 ±150 的窄窗口，那就永远等不到交棒。
     */
}

/**
 * @brief 启摆状态机的 1 ms 推进。由 control_tick() 在 SWING_UP 状态下调用。
 *
 * 对应参考的 21~24 / 31~34 四步一组：打一个方向的脉冲 → 计时 →
 * 打反方向的脉冲 → 计时 → 回判定相。
 *
 * ⚠️ **两个脉冲首尾相接、中间不留空隙**，这与参考一致。改成"打一下停一下"
 *    会改变注进摆杆的能量，得不偿失。
 */
static void swing_tick(void)
{
    /* —— 超时兜底（参考实现没有这一条，见 CONTROL_SWING_TIMEOUT_MS）—— */
    s_swing_elapsed++;
    if (s_swing_elapsed > CONTROL_SWING_TIMEOUT_MS) {
        /* control_stop() 会把结局记成 ABORTED（那是给"人按停"用的），
         * 所以调完再盖回 TIMEOUT，否则串口上分不出是谁停的。 */
        (void)control_stop();
        s_status.swing_result = (uint8_t)CONTROL_SWING_TIMEOUT;
        return;
    }

    /*
     * ── 交棒判定：每 1 ms 查一次，窗口用 START（±150）──
     *
     * 用 START 而不是 RANGE 是**必须的**，理由见 swing_judge() 的注释：
     * 本台实测自由下垂读数 1883 落在 CENTER±500 内，用 RANGE 会把垂着的
     * 摆杆判成"已经立好了"，直接交棒给默认增益的双环、电机立刻猛冲
     * （2026-10-09 实测踩到过：SWP=0 的一次自检里臂真的动了）。
     *
     * 用 START 也顺带把语义统一了：**交棒条件 == RUN 的准入条件**。
     * 启摆的任务就是"把摆杆弄到 RUN 会接受的状态"，两个窗口在这里合流。
     */
    if (angle_in_window(s_status.angle, s_params.start_range)) {
        s_swing_hits++;
        if (s_swing_hits >= (uint8_t)CONTROL_SWING_CATCH_MS) {
            swing_finish();
            return;
        }
    } else {
        s_swing_hits = 0U;
    }

    const uint16_t swt = (uint16_t)s_params.swing_time;

    switch (s_swing_phase) {
        case SWING_PULSE1:
            swing_pulse(s_swing_dir);
            s_swing_timer = swt;
            s_swing_phase = SWING_PULSE1_WAIT;
            break;

        case SWING_PULSE1_WAIT:
            if (s_swing_timer > 0U) {
                s_swing_timer--;
            }
            if (s_swing_timer == 0U) {
                s_swing_phase = SWING_PULSE2;
            }
            break;

        case SWING_PULSE2:
            swing_pulse((int8_t)(-s_swing_dir));
            s_swing_timer = swt;
            s_swing_phase = SWING_PULSE2_WAIT;
            break;

        case SWING_PULSE2_WAIT:
            if (s_swing_timer > 0U) {
                s_swing_timer--;
            }
            if (s_swing_timer == 0U) {
                /* 这一组打完：滑行，回判定相，重新攒三个点。
                 * 用 coast 而不是 set_duty(0)——脉冲间隙要让摆杆自由摆动。 */
                (void)bsp_motor_coast();
                s_status.pwm  = 0;
                s_count_judge = 0U;
                s_ang_count   = 0U;
                s_swing_phase = SWING_JUDGE;
            }
            break;

        case SWING_JUDGE:
        default:
            swing_judge();
            break;
    }
}

/* ----------------------------------------------------------------- 实现 */

error_t control_init(void)
{
    s_initialized = false;

    /* —— 参数默认值 —— */
    s_params.a_kp = CONTROL_DEFAULT_AKP;
    s_params.a_ki = CONTROL_DEFAULT_AKI;
    s_params.a_kd = CONTROL_DEFAULT_AKD;
    s_params.p_kp = CONTROL_DEFAULT_PKP;
    s_params.p_ki = CONTROL_DEFAULT_PKI;
    s_params.p_kd = CONTROL_DEFAULT_PKD;
    s_params.pwm_offset   = CONTROL_DEFAULT_OFFSET;
    s_params.swing_pwm    = CONTROL_DEFAULT_SWP;
    s_params.swing_time   = CONTROL_DEFAULT_SWT;
    s_params.center_angle = CONTROL_DEFAULT_CENTER;
    s_params.center_range = CONTROL_DEFAULT_RANGE;
    s_params.start_range  = CONTROL_DEFAULT_START;

    /* —— 两个 PID 实例 ——
     * 角度环的输出直接当 PWM，所以限幅就是 PWM 满量程；
     * 位置环的输出是角度偏移量，限幅是 ±100 码。 */
    s_angle_pid.kp        = s_params.a_kp;
    s_angle_pid.ki        = s_params.a_ki;
    s_angle_pid.kd        = s_params.a_kd;
    s_angle_pid.out_min   = -(float)BSP_MOTOR_DUTY_MAX;
    s_angle_pid.out_max   =  (float)BSP_MOTOR_DUTY_MAX;
    s_angle_pid.integ_min = -CONTROL_ANGLE_INTEG_LIMIT;
    s_angle_pid.integ_max =  CONTROL_ANGLE_INTEG_LIMIT;
    s_angle_pid.target    =  (float)s_params.center_angle;

    s_pos_pid.kp        = s_params.p_kp;
    s_pos_pid.ki        = s_params.p_ki;
    s_pos_pid.kd        = s_params.p_kd;
    s_pos_pid.out_min   = -CONTROL_POS_OUT_LIMIT;
    s_pos_pid.out_max   =  CONTROL_POS_OUT_LIMIT;
    s_pos_pid.integ_min = -CONTROL_POS_INTEG_LIMIT;
    s_pos_pid.integ_max =  CONTROL_POS_INTEG_LIMIT;
    s_pos_pid.target    =  (float)s_pos_target;

    pid_reset(&s_angle_pid);
    pid_reset(&s_pos_pid);

    /* —— 快照对齐到当前读数 ——
     * 底层的 init 都跑完了，这两句能拿到真值。不这么做的话 OLED
     * 第一次画出来全是 0，看起来像接线断了。 */
    s_status.angle         = bsp_angle_raw();
    s_status.position      = bsp_encoder_total();
    s_status.speed         = bsp_encoder_delta();
    s_status.angle_target  = s_angle_pid.target;
    s_status.angle_out     = 0.0f;
    s_status.pos_out       = 0.0f;
    s_status.pwm           = 0;
    s_status.state         = CONTROL_STATE_STOP;
    s_status.swing_result  = (uint8_t)CONTROL_SWING_NONE;

    s_angle_pid.actual = (float)s_status.angle;
    s_pos_pid.actual   = (float)s_status.position;

    s_count_angle = 0U;
    s_count_pos   = 0U;
    s_state       = CONTROL_STATE_STOP;

    /* 启摆子状态也一并复位，免得第一次 SWING 用上一次的残留相位 */
    s_swing_phase   = SWING_JUDGE;
    s_swing_dir     = 1;
    s_swing_timer   = 0U;
    s_swing_elapsed = 0U;
    s_count_judge   = 0U;
    s_ang_count     = 0U;
    s_swing_hits    = 0U;
    s_ang[0] = 0U;
    s_ang[1] = 0U;
    s_ang[2] = 0U;

    /* 未启动时保证电机不动。上电时序里 motor_init 已经把占空比清过，
     * 这里是双保险，也顺便覆盖「控制模块比电机晚初始化」的情况。 */
    (void)bsp_motor_coast();

    s_initialized = true;

    return ERR_OK;
}

void control_tick(void)
{
    if (!s_initialized) {
        return;
    }

    /* ── ① 角度：先取上一次的结果，再启动下一次 ──────────────────
     *
     * 顺序是「先取后启」。反过来（先启后取）的话，取到的是刚刚才
     * 启动、肯定还没转完的那次转换，poll 每次都返回 NOT_READY，
     * 实际采样周期就变成了 2 ms 而且抖动。
     *
     * 先取后启时，两次取值的间隔恰好是一个完整的 1 ms 节拍，
     * 采样周期恒定 —— 这对微分项尤其重要，因为 Kd 对采样间隔的
     * 抖动最敏感。 */
    if (bsp_angle_poll() == ERR_OK) {
        s_status.angle = bsp_angle_raw();
    }
    (void)bsp_angle_trigger();

    /* ── ② 编码器 ── */
    (void)bsp_encoder_update();
    s_status.position = bsp_encoder_total();
    s_status.speed    = bsp_encoder_delta();

    /* ── ③ 倒下保护 ───────────────────────────────────────────────
     *
     * 放在最前面（先于两个环），保证「这一拍已经摔了」时绝不会再
     * 产生新的输出。
     *
     * 为什么能奏效、以及为什么它挡不住「一开始就倒着」的情况，
     * 见 control.h 的盲区陷阱一节。 */
    if ((s_state == CONTROL_STATE_RUN) &&
        !angle_in_window(s_status.angle, s_params.center_range)) {
        (void)control_stop();
    }

    /* ── ④ 停止态：保证电机真的不动 ───────────────────────────────
     *
     * 每拍都写一次 coast，而不是「停机时写一次就完了」。理由：
     * 停机之后万一有别的地方动过电机（调试时很常见），下一拍就会被
     * 拉回来。开销是几个寄存器写，可以忽略。
     *
     * ⚠️ 这里的判断**必须是 `== STOP`，不能写成 `!= RUN`**。加了启摆状态
     *    之后，「不是 RUN」把 SWING_UP 也包了进去；写成 `!= RUN` 会把启摆
     *    当成停止态、每拍强制 coast 后 return —— 现象是「发 SWING 之后
     *    完全没反应、电机一动不动」，而且**不报任何错**。 */
    if (s_state == CONTROL_STATE_STOP) {
        (void)bsp_motor_coast();
        s_status.pwm   = 0;
        s_status.state = CONTROL_STATE_STOP;

        /* 停机时角度目标就是中心值本身（外环不工作，没有人去偏移它）。
         * 每拍刷一次，是为了让「停了之后用 SET CENTER 改了中心」
         * 能立刻反映到显示和 STAT 上 —— 否则 ATAR 会一直挂着
         * 上一次运行结束时的陈旧值，看着像控制器还在算。 */
        s_status.angle_target = (float)s_params.center_angle;

        s_count_angle  = 0U;
        s_count_pos    = 0U;
        return;
    }
    s_status.state = s_state;

    /* ── ④b 自动启摆 ─────────────────────────────────────────────
     *
     * 启摆期间两个 PID 环都不算：这是「把摆杆荡上去」的阶段，摆杆本来就
     * 是倒的，做闭环没有意义。倒下保护（③）也只在 RUN 生效，所以这里
     * 不会被自己的保护逻辑打断。 */
    if (s_state == CONTROL_STATE_SWING_UP) {
        swing_tick();
        return;
    }

    /* ── ⑤ 角度环（内环，5 ms）───────────────────────────────────
     *
     * 先算角度环、后算位置环，与官方顺序一致。差别在于：官方这一拍
     * 角度环用的是 50 ms 前设下的目标，本实现也一样 —— 位置环刚算出的
     * 新目标要到下一拍才生效。这个一拍的滞后是串级结构的固有特性
     * （外环本来就慢 10 倍），不需要去「优化」掉。 */
    s_count_angle++;
    if (s_count_angle >= CONTROL_ANGLE_PERIOD_MS) {
        s_count_angle = 0U;

        s_angle_pid.actual = (float)s_status.angle;
        pid_update(&s_angle_pid);

        apply_motor(s_angle_pid.out);

        s_status.angle_out  = s_angle_pid.out;
        s_status.angle_target = s_angle_pid.target;
    }

    /* ── ⑥ 位置环（外环，50 ms）──────────────────────────────────
     *
     * 它的输出不直接给电机，而是去**移动内环的目标角度**，
     * 那个负号的含义见 control.h。 */
    s_count_pos++;
    if (s_count_pos >= CONTROL_POS_PERIOD_MS) {
        s_count_pos = 0U;

        s_pos_pid.actual = (float)s_status.position;
        pid_update(&s_pos_pid);

        s_angle_pid.target = (float)s_params.center_angle - s_pos_pid.out;

        s_status.pos_out = s_pos_pid.out;
    }
}

error_t control_start(void)
{
    if (!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }

    /*
     * 必须已经扶到竖直附近才允许启动 —— 这里用的是**更严的**
     * start_range，不是运行中那个 center_range。
     *
     * 两个窗口为什么要分开，见 control.h。一句话：本台实测「摆杆
     * 自由下垂」时盲区读数是 1883，而 center_range = 500 的窗口是
     * [1586, 2586]，**1883 在里面** —— 用那个窗口检查的话，垂着的
     * 摆杆会被放行。start_range = 150 的窗口是 [1936, 2236]，
     * 1883 被排除在外。
     */
    if (!angle_in_window(s_status.angle, s_params.start_range)) {
        return ERR_NOT_READY;
    }

    /*
     * 清掉两个环的积分和历史误差。
     *
     * 不清的后果很具体：上一次运行结束时积分可能停在一个很大的值上，
     * 下一次启动的第一拍就会把这个积压的值直接送进输出，电机猛冲一下
     * —— 对一台倒立摆来说这一下足以把摆杆甩翻。
     */
    pid_reset(&s_angle_pid);
    pid_reset(&s_pos_pid);

    /* 重新对齐输入，并让目标从一个干净的状态出发 */
    s_angle_pid.actual = (float)s_status.angle;
    s_angle_pid.target = (float)s_params.center_angle;
    s_pos_pid.actual   = (float)s_status.position;
    s_pos_pid.target   = (float)s_pos_target;

    s_count_angle = 0U;
    s_count_pos   = 0U;
    s_state       = CONTROL_STATE_RUN;
    /* 快照立刻跟上：否则 RUN 命令之后紧接的一条 STAT 会读到上一拍的旧状态 */
    s_status.state = CONTROL_STATE_RUN;

    s_status.pos_out      = 0.0f;
    s_status.angle_target = s_angle_pid.target;
    /* 显式 RUN 没有经过启摆，把上一次的启摆结局清掉 —— 否则 SWR 会挂着一个
     * 陈旧的"成功/超时"，让人以为刚才这次是启摆起来的。 */
    s_status.swing_result = (uint8_t)CONTROL_SWING_NONE;

    return ERR_OK;
}

error_t control_swing_up(void)
{
    if (!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }

    /* 已经在双环控制中：什么都不做（幂等），不要把正在稳住的摆杆打断 */
    if (s_state == CONTROL_STATE_RUN) {
        return ERR_OK;
    }

    /*
     * ⚠️ 这里**故意不判断"角度是不是已经立好了"**，也就是无条件启摆。
     *
     * 原先有一条捷径：角度落在 START 窗口内就直接转 control_start()。
     * 2026-10-10 真机实测把它否掉了：摆杆被上一轮甩下来之后还在慢慢摆，
     * 路过窗口时静息读数一度是 1949（在 ±150 内），于是 SWING 不但没启摆，
     * 反而把"正在摆动的摆杆"当成"已经立好"，直接交给双环 → 电机猛冲 →
     * 摆杆被甩飞 → 倒下保护停机。
     *
     * 根因是这条捷径在猜一个它猜不准的事：**某一瞬间落在窗口内，
     * 既不等于"摆杆是立着的"，也不等于"它是静止的"**（这跟 control.h
     * 盲区一节里那句「读数稳定 ≠ 读数是有效的」是同一类错误）。
     *
     * 所以 SWING 只做一件事：无条件启摆。它是"从任意姿态把它弄起来"，
     * 不该去猜姿态。**要手扶启动就用 RUN** —— 那条路径的准入检查没变。
     */

    /*
     * 开始启摆。第一组脉冲对应参考实现的状态 21（先正向、再反向），
     * 这不是随便定的：摆杆此刻多半垂在盲区里、读数无效，先用一记脉冲
     * 把它踢出盲区，后面的顶点判定才成立。参考的注释也是这个意思
     * （「使摆杆离开角度传感器盲区，避免盲区干扰」）。
     */
    s_swing_phase   = SWING_PULSE1;
    s_swing_dir     = 1;
    s_swing_timer   = 0U;
    s_swing_elapsed = 0U;
    s_count_judge   = 0U;
    s_ang_count     = 0U;
    s_swing_hits    = 0U;
    s_ang[0] = 0U;
    s_ang[1] = 0U;
    s_ang[2] = 0U;

    s_status.swing_result = (uint8_t)CONTROL_SWING_NONE;
    s_status.angle_target = (float)s_params.center_angle;
    s_status.angle_out    = 0.0f;
    s_status.pos_out      = 0.0f;

    s_state = CONTROL_STATE_SWING_UP;
    /* 快照立刻跟上，否则 SWING 命令回的那条 ST= 会读到上一拍的旧状态 */
    s_status.state = CONTROL_STATE_SWING_UP;

    return ERR_OK;
}

error_t control_stop(void)
{
    /*
     * 如果这次停机打断的是一次启摆，把结局记成 ABORTED。
     * 串口上要能分出「人（或 agent）按停的」和「启摆超时自己停的」——
     * 这两种情况下一步该做的事完全不同。
     */
    if (s_state == CONTROL_STATE_SWING_UP) {
        s_status.swing_result = (uint8_t)CONTROL_SWING_ABORTED;
    }

    s_state = CONTROL_STATE_STOP;

    /* 启摆子状态一并复位：下次 SWING 从"打第一组脉冲"重新开始，
     * 不会带着上一次的相位/计时继续跑。 */
    s_swing_phase   = SWING_JUDGE;
    s_swing_dir     = 1;
    s_swing_timer   = 0U;
    s_swing_elapsed = 0U;
    s_count_judge   = 0U;
    s_ang_count     = 0U;
    s_swing_hits    = 0U;

    /*
     * 把两个环的输出清掉，但**保留** angle / position（那两个是实测值，
     * 停机后依然有效，屏幕上还要继续显示）。
     *
     * 不清的后果：停机那一刻的输出（可能是个很大的值，比如摆杆倒下前
     * 控制器正在使劲）会一直挂在快照里，屏幕上「AOu 1186」而
     * 「PWM 0」同时出现 —— 看着像控制器还在输出而电机没动，
     * 实际只是残留。清掉就没有这个歧义。
     */
    s_status.angle_out    = 0.0f;
    s_status.pos_out      = 0.0f;
    s_status.angle_target = (float)s_params.center_angle;
    s_status.pwm          = 0;
    /* 快照立刻跟上：否则紧接着的一条 STAT 会读到上一拍的旧状态 */
    s_status.state        = CONTROL_STATE_STOP;

    /* 停机 = 滑行，不是刹车。倒立摆停机时摆杆是自由摆动的，
     * 用刹车会让横杆急停、摆杆被惯性甩得更凶。 */
    return bsp_motor_coast();
}

bool control_is_running(void)
{
    return (s_state == CONTROL_STATE_RUN);
}

error_t control_zero_position(void)
{
    if (!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }

    (void)bsp_encoder_reset();

    s_pos_target       = 0;
    s_status.position  = 0;
    s_pos_pid.target   = 0.0f;
    s_pos_pid.actual   = 0.0f;

    return ERR_OK;
}

int32_t control_position_target(void)
{
    return s_pos_target;
}

error_t control_set_position_target(int32_t target)
{
    if (!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }

    if ((target > CONTROL_POS_TARGET_LIMIT) || (target < -CONTROL_POS_TARGET_LIMIT)) {
        return ERR_INVALID_PARAM;
    }

    s_pos_target     = target;
    s_pos_pid.target = (float)target;

    return ERR_OK;
}

void control_get_status(control_status_t *out)
{
    if (out == NULL) {
        return;
    }

    *out = s_status;
}

const control_params_t *control_params(void)
{
    return &s_params;
}

error_t control_param_set(control_param_t p, float value)
{
    if (!s_initialized) {
        return ERR_NOT_INITIALIZED;
    }
    if ((int)p < 0 || p >= CONTROL_PARAM_COUNT) {
        return ERR_INVALID_PARAM;
    }

    /*
     * 先挡掉 NaN 和 inf。
     *
     * 不挡的后果：NaN 一旦进了增益，两个环的输出全变 NaN，
     * 转成 int16_t 是未定义行为（实际会得到 0 或某个垃圾值），
     * 现象是「电机突然不响应任何指令」，而且从 OLED 上看不出来
     * （NaN 打印出来也像个数）。挡在入口最省事。
     *
     * `value != value` 是判断 NaN 的标准写法（NaN 不等于自己），
     * 不需要引入 <math.h>。
     */
    if ((value != value) || (value > 1.0e30f) || (value < -1.0e30f)) {
        return ERR_INVALID_PARAM;
    }

    /*
     * ── 并发：这里为什么不关中断 ────────────────────────────────
     *
     * 参数是 32 位对齐的 float，在 Cortex-M3 上**单条 STR 指令写入，
     * 不会撕裂**（不存在「写了一半」的中间态）。所以中断里读到的
     * 要么是旧值、要么是新值，都是合法的浮点数。
     *
     * 真正需要临界区的是「多个字段必须同时生效」（比如切换整套参数），
     * 而本接口一次只改**一个**字段，不存在这种需求。
     *
     * 那要不要担心「角度环读到新 Kp、位置环还读到旧 Kp」？不需要：
     * 一条 SET 命令只改一个参数，而 control_tick 每 1 ms 跑一次，
     * 两条命令之间至少隔着几十个节拍。所以实际不会出现混合状态。
     *
     * 结论：加了临界区反而是「保护了一个不存在的问题」，还增加了
     * 代码复杂度。这里选择不加，并把理由写清楚。
     * ────────────────────────────────────────────────────────────
     */
    switch (p) {
        case CONTROL_PARAM_AKP:
            s_params.a_kp = value;
            s_angle_pid.kp = value;
            break;
        case CONTROL_PARAM_AKI:
            s_params.a_ki = value;
            s_angle_pid.ki = value;
            break;
        case CONTROL_PARAM_AKD:
            s_params.a_kd = value;
            s_angle_pid.kd = value;
            break;
        case CONTROL_PARAM_PKP:
            s_params.p_kp = value;
            s_pos_pid.kp = value;
            break;
        case CONTROL_PARAM_PKI:
            s_params.p_ki = value;
            s_pos_pid.ki = value;
            break;
        case CONTROL_PARAM_PKD:
            s_params.p_kd = value;
            s_pos_pid.kd = value;
            break;

        case CONTROL_PARAM_CENTER:
            /* 必须是合法的 ADC 码。越界的后果是窗口整个落到量程外，
             * 于是「角度永远不在窗口内」—— 按 RUN 立刻停，很难查。 */
            if (value < 0.0f || value > (float)BSP_ANGLE_RAW_MAX) {
                return ERR_INVALID_PARAM;
            }
            s_params.center_angle = (uint16_t)value;
            break;

        case CONTROL_PARAM_RANGE:
            /* 下限 1 而不是 0：RANGE=0 的窗口是一个点，实际上等价于
             * 「永远不在窗口内」，同样是「按 RUN 立刻停」那种难查的故障。
             * 上限取半个量程，比这更大的窗口没有意义。 */
            if (value < 1.0f || value > 2048.0f) {
                return ERR_INVALID_PARAM;
            }
            s_params.center_range = (uint16_t)value;
            break;

        case CONTROL_PARAM_START:
            /* 同样的下限理由（0 等价于「永远不允许启动」）。
             * 这里**不**检查 start_range <= center_range：那样会制造
             * 一个「必须先设 RANGE 再设 START」的顺序陷阱，而 agent
             * 完全可能反着来。约定写在 control.h 里，靠文档而不是靠报错。 */
            if (value < 1.0f || value > 2048.0f) {
                return ERR_INVALID_PARAM;
            }
            s_params.start_range = (uint16_t)value;
            break;

        case CONTROL_PARAM_OFFSET:
            /* 只允许非负：补偿的方向由输出的符号决定，负的 offset
             * 会变成「朝反方向推」，那不是补偿而是新的故障源。 */
            if (value < 0.0f || value >= (float)BSP_MOTOR_DUTY_MAX) {
                return ERR_INVALID_PARAM;
            }
            s_params.pwm_offset = value;
            break;

        case CONTROL_PARAM_SWP:
            /* 与 OFFSET 同一条约束：脉冲方向由状态机决定，负值没有意义。
             * **允许 0** —— 那是「只判定、不打脉冲」，调试状态机时有用，
             * 而且此时电机保证不动（脉冲占空比为 0）。 */
            if (value < 0.0f || value >= (float)BSP_MOTOR_DUTY_MAX) {
                return ERR_INVALID_PARAM;
            }
            s_params.swing_pwm = value;
            break;

        case CONTROL_PARAM_SWT:
            /* 下限 1：0 毫秒的脉冲等于没打，只会让状态机空转。
             * 上限 1000：参考的推荐区间是 80~120；超过 1 秒已经不是
             * 「一次瞬时冲击」而是持续驱动了，拦住它。 */
            if (value < 1.0f || value > 1000.0f) {
                return ERR_INVALID_PARAM;
            }
            s_params.swing_time = value;
            break;

        default:
            return ERR_INVALID_PARAM;
    }

    return ERR_OK;
}

float control_param_get(control_param_t p)
{
    switch (p) {
        case CONTROL_PARAM_AKP:    return s_params.a_kp;
        case CONTROL_PARAM_AKI:    return s_params.a_ki;
        case CONTROL_PARAM_AKD:    return s_params.a_kd;
        case CONTROL_PARAM_PKP:    return s_params.p_kp;
        case CONTROL_PARAM_PKI:    return s_params.p_ki;
        case CONTROL_PARAM_PKD:    return s_params.p_kd;
        case CONTROL_PARAM_CENTER: return (float)s_params.center_angle;
        case CONTROL_PARAM_RANGE:  return (float)s_params.center_range;
        case CONTROL_PARAM_START:  return (float)s_params.start_range;
        case CONTROL_PARAM_OFFSET: return s_params.pwm_offset;
        case CONTROL_PARAM_SWP:    return s_params.swing_pwm;
        case CONTROL_PARAM_SWT:    return s_params.swing_time;
        default:                   return 0.0f;
    }
}

const char *control_param_name(control_param_t p)
{
    static const char *const names[CONTROL_PARAM_COUNT] = {
        "AKP", "AKI", "AKD", "PKP", "PKI", "PKD",
        "CENTER", "RANGE", "START", "OFFSET", "SWP", "SWT",
    };

    if ((int)p < 0 || p >= CONTROL_PARAM_COUNT) {
        return "?";
    }

    return names[p];
}
