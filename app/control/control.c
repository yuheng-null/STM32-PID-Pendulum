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
 * 位置环的积分限幅。同样取 out_max / ki = 100 / 0.18 ≈ 555 向下圆整。
 */
#define CONTROL_POS_INTEG_LIMIT     (500.0f)

/*
 * ── 默认增益：官方值 × 18 ──────────────────────────────────────────
 *
 * 官方 00-PID综合测试程序 V1.0/V1.1 的 Mode3（倒立摆模式）参数是：
 *
 *     角度环  Kp=0.25  Ki=0.009  Kd=0.41    OutMax=100  OutMin=-100
 *     位置环  Kp=0.52  Ki=0.01   Kd=4.56    OutMax=100  OutMin=-100
 *     OFFSET_PWM = 5
 *
 * 而它的 PWM 时基是 ARR=99（PWM.c 里 `TIM_Period = 100 - 1`），
 * 也就是**占空比满量程对应 100 个计数**。本工程的 ARR=1799，
 * 满量程对应 1800 个计数 —— **正好差 18 倍**。
 *
 * 增益要跟着量程走，因为真正决定物理行为的是**占空比**而不是裸计数：
 *
 *     官方：duty = Kp_official × e / 100
 *     本机：duty = Kp_ours     × e / 1800
 *     两者 duty 相同 ⟹ Kp_ours = Kp_official × 18
 *
 * 所以下面的默认值 = 官方值 × 18，对应的是**同一个物理强度的控制器**。
 * 照抄官方原值（0.25 之类）会得到一个弱 18 倍的控制器，摆杆立不住，
 * 而且会误以为「官方参数是错的」。
 *
 * ⚠️ 这些只是**整定的起点**，不是最终值。本轮不调参。
 * ⚠️ 位置环的输出限幅（±100 码）**不乘 18** —— 它的单位是角度码，
 *    与 PWM 量程无关。
 * ────────────────────────────────────────────────────────────────
 */
#define CONTROL_DEFAULT_AKP         (4.50f)     /* 0.25  × 18 */
#define CONTROL_DEFAULT_AKI         (0.162f)    /* 0.009 × 18 */
#define CONTROL_DEFAULT_AKD         (7.38f)     /* 0.41  × 18 */
#define CONTROL_DEFAULT_PKP         (9.36f)     /* 0.52  × 18 */
#define CONTROL_DEFAULT_PKI         (0.18f)     /* 0.01  × 18 */
#define CONTROL_DEFAULT_PKD         (82.08f)    /* 4.56  × 18 */

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
 *    本轮先按换算值放，调参阶段再细调。
 */
#define CONTROL_DEFAULT_OFFSET      (90.0f)     /* 5 × 18 */

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

    s_angle_pid.actual = (float)s_status.angle;
    s_pos_pid.actual   = (float)s_status.position;

    s_count_angle = 0U;
    s_count_pos   = 0U;
    s_state       = CONTROL_STATE_STOP;

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
     * 拉回来。开销是几个寄存器写，可以忽略。 */
    if (s_state != CONTROL_STATE_RUN) {
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

    s_status.pos_out      = 0.0f;
    s_status.angle_target = s_angle_pid.target;

    return ERR_OK;
}

error_t control_stop(void)
{
    s_state = CONTROL_STATE_STOP;

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
        default:                   return 0.0f;
    }
}

const char *control_param_name(control_param_t p)
{
    static const char *const names[CONTROL_PARAM_COUNT] = {
        "AKP", "AKI", "AKD", "PKP", "PKI", "PKD",
        "CENTER", "RANGE", "START", "OFFSET",
    };

    if ((int)p < 0 || p >= CONTROL_PARAM_COUNT) {
        return "?";
    }

    return names[p];
}
