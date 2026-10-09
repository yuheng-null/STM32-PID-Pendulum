/**
 * @file    console.c
 * @brief   串口调参控制台的实现。协议规格见 console.h。
 */

#include "console.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "main.h"                   /* 借它引入 stm32f1xx_hal.h（要 HAL_GetTick）*/

#include "bsp_serial.h"
#include "control.h"

/* ----------------------------------------------------------------- 配置 */

/** 一条命令的最大长度（不含结尾的 '\n'）。超了就报错并丢弃这一行。 */
#define CONSOLE_LINE_MAX        (64U)

/** 响应缓冲大小。够放下最长的 `GET ALL`（约 100 字符）。 */
#define CONSOLE_RESP_MAX        (192U)

/** 一条命令最多几个词（SET 需要 3 个：SET 名字 值）*/
#define CONSOLE_ARG_MAX         (4U)

/** 每次从接收缓冲最多取多少字节 */
#define CONSOLE_READ_CHUNK      (32U)

/**
 * STREAM 模式的上报周期（毫秒）。
 *
 * 20 ms = 50 Hz。选它是因为调参时最关心的横杆晃动大约在几 Hz，
 * 50 Hz 采样留了 10 倍余量，一根 USB 转串口完全带得动。
 *
 * ⚠️ 实际周期会被主循环里的 OLED 刷新（约 30 ms）拖出抖动 ——
 *    见 console.h 的说明。要更稳的波形就把刷新周期调长，或者
 *    用 STREAM 0 关掉、改用主动的 STAT 轮询。
 */
#define CONSOLE_STREAM_PERIOD_MS    (20U)

/* ----------------------------------------------------------------- 状态 */

static char     s_line[CONSOLE_LINE_MAX + 1U];
static uint8_t  s_line_len      = 0U;
static bool     s_line_overflow = false;    /* 这一行超长，执行时直接丢弃 */

/**
 * 上一个字符是不是 '\r'。
 * 用来把 Windows 风格的 CRLF 当成**一次**换行 —— 否则每条命令都会
 * 被执行两遍（第二遍是空行，回一个 ERR，让上位机误以为有故障）。
 */
static bool     s_saw_cr        = false;

static bool     s_stream_on     = false;
static uint32_t s_stream_last_ms = 0U;

/* ----------------------------------------------------------------- 响应缓冲 */
/*
 * 把响应先攒进缓冲、最后用**一次** bsp_serial_write 发出去。
 *
 * 为什么不直接把每一段分别写进串口：发送环形缓冲只有 256 字节，
 * 分段写虽然也不会丢（满了会截断），但一条被打断的响应读起来很乱，
 * 而且分段写之间如果有别的打印插进来，agent 会解析到半条记录。
 * 攒齐了一次发，天然避免这两件事。
 */

typedef struct {
    char    buf[CONSOLE_RESP_MAX];
    uint16_t len;
} resp_t;

/** 追加一个字符。缓冲满了就丢弃（不越界、不报错）——响应被截断比崩溃好。 */
static void resp_ch(resp_t *r, char c)
{
    if (r->len < (uint16_t)(CONSOLE_RESP_MAX - 1U)) {
        r->buf[r->len] = c;
        r->len++;
    }
}

static void resp_str(resp_t *r, const char *s)
{
    while (*s != '\0') {
        resp_ch(r, *s);
        s++;
    }
}

static void resp_reset(resp_t *r)
{
    r->len = 0U;
}

/** 无符号十进制。不用 snprintf：避免把整套 printf 拖进 Flash（约 3 KB）。 */
static void resp_u32(resp_t *r, uint32_t v)
{
    char    tmp[10];
    uint8_t n = 0U;

    if (v == 0U) {
        resp_ch(r, '0');
        return;
    }

    while ((v > 0U) && (n < sizeof(tmp))) {
        tmp[n] = (char)('0' + (v % 10U));
        v /= 10U;
        n++;
    }

    while (n > 0U) {
        n--;
        resp_ch(r, tmp[n]);
    }
}

static void resp_i32(resp_t *r, int32_t v)
{
    uint32_t mag;

    if (v < 0) {
        resp_ch(r, '-');
        /* 取相反数要用 uint32 中转：int32 的 -(-2147483648) 会溢出 */
        mag = (uint32_t)(-(v + 1)) + 1U;
    } else {
        mag = (uint32_t)v;
    }

    resp_u32(r, mag);
}

/**
 * 定点小数，固定三位（如 -82.080）。
 *
 * 这是本模块**唯一**打印浮点的地方。不用 `printf("%f")` 的原因：
 * 那需要链接 `-u _printf_float`，代价约 6 KB Flash，而本工程只用来
 * 显示几个增益，不值当。
 */
static void resp_f3(resp_t *r, float v)
{
    int32_t milli;
    uint32_t frac;

    /* 防御：NaN/超大值不该走到这里（control_param_set 会挡），
     * 但计算 `v * 1000` 溢出是未定义行为，宁可先夹一下。 */
    if (v != v) {
        resp_str(r, "nan");
        return;
    }
    if (v > 2.0e6f)  { v =  2.0e6f; }
    if (v < -2.0e6f) { v = -2.0e6f; }

    /* 四舍五入到千分之一。加 ±0.5 而不是用 roundf()，省一个库依赖。 */
    milli = (int32_t)((v * 1000.0f) + ((v >= 0.0f) ? 0.5f : -0.5f));

    if (milli < 0) {
        resp_ch(r, '-');
        milli = -milli;
    }

    resp_u32(r, (uint32_t)(milli / 1000));
    resp_ch(r, '.');

    frac = (uint32_t)(milli % 1000);
    resp_ch(r, (char)('0' + (frac / 100U)));
    resp_ch(r, (char)('0' + ((frac / 10U) % 10U)));
    resp_ch(r, (char)('0' + (frac % 10U)));
}

/** 补上 "\r\n" 并整条发出去。 */
static void resp_send(resp_t *r)
{
    resp_ch(r, '\r');
    resp_ch(r, '\n');

    (void)bsp_serial_write((const uint8_t *)r->buf, (uint32_t)r->len);
}

static void resp_error(const char *reason, const char *detail)
{
    resp_t r;

    resp_reset(&r);
    resp_str(&r, "ERR ");
    resp_str(&r, reason);
    if (detail != NULL) {
        resp_ch(&r, ' ');
        resp_str(&r, detail);
    }
    resp_send(&r);
}

/* ----------------------------------------------------------------- 数值解析 */
/*
 * 自己解析，不用 atof/strtol：newlib 的 strtod 会拖进一大坨
 * 国际化（locale）和浮点解析代码，而我们只需要「十进制、可带小数点」
 * 这一种形态。自己写反而更短，而且能明确区分「格式错」和「越界」。
 */

/**
 * @brief 解析十进制浮点。
 *
 * 接受：`0`  `-12`  `+3.5`  `.5`  `0.28`
 * 拒绝：空串、`1.2.3`、`abc`、`1e5`（不支持科学计数法，调参用不上）、
 *       任何夹杂非数字字符的输入。
 *
 * @return true 解析成功
 */
static bool parse_float(const char *s, float *out)
{
    bool     neg = false;
    bool     any_digit = false;
    float    value = 0.0f;
    float    scale = 0.1f;
    bool     in_frac = false;
    int      int_digits = 0;

    if (*s == '\0') {
        return false;
    }

    if ((*s == '-') || (*s == '+')) {
        neg = (*s == '-');
        s++;
    }

    while (*s != '\0') {
        const char c = *s;

        if (c == '.') {
            if (in_frac) {
                return false;           /* 第二个小数点 */
            }
            in_frac = true;
            s++;
            continue;
        }

        if ((c < '0') || (c > '9')) {
            return false;               /* 混进了非数字字符 */
        }

        any_digit = true;

        if (!in_frac) {
            /* 整数部分。限制位数，避免 `99999999999999` 把 value 撑成 inf */
            if (int_digits > 8) {
                return false;
            }
            value = (value * 10.0f) + (float)(c - '0');
            int_digits++;
        } else {
            value += ((float)(c - '0')) * scale;
            scale *= 0.1f;
        }

        s++;
    }

    if (!any_digit) {
        return false;                   /* 只有符号或只有小数点 */
    }

    *out = neg ? -value : value;

    return true;
}

/**
 * @brief 解析十进制整数（int32）。
 *
 * 只接受可选的符号 + 数字，不接受小数点。位置目标用得到。
 */
static bool parse_i32(const char *s, int32_t *out)
{
    bool     neg = false;
    bool     any_digit = false;
    int32_t  value = 0;

    if (*s == '\0') {
        return false;
    }

    if ((*s == '-') || (*s == '+')) {
        neg = (*s == '-');
        s++;
    }

    while (*s != '\0') {
        const char c = *s;

        if ((c < '0') || (c > '9')) {
            return false;
        }

        /* 超过 9 位就当成非法，省得处理溢出。位置量级是 ±4080，
         * 远远用不到这么长。 */
        if (value > 999999999) {
            return false;
        }

        value = (value * 10) + (c - '0');
        any_digit = true;
        s++;
    }

    if (!any_digit) {
        return false;
    }

    *out = neg ? -value : value;

    return true;
}

/* ----------------------------------------------------------------- 分词 */

/**
 * @brief 按空格/制表符切词，就地写入 '\0'。
 *
 * 空串返回 0 个词（调用者应当忽略——只有换行没有内容的行是正常的，
 * 比如串口工具连按两次回车）。
 */
static uint8_t tokenize(char *line, char **tok, uint8_t max_tok)
{
    uint8_t n = 0U;
    char   *p = line;

    while ((*p != '\0') && (n < max_tok)) {
        while ((*p == ' ') || (*p == '\t')) {
            p++;
        }
        if (*p == '\0') {
            break;
        }

        tok[n] = p;
        n++;

        while ((*p != '\0') && (*p != ' ') && (*p != '\t')) {
            p++;
        }
        if (*p != '\0') {
            *p = '\0';
            p++;
        }
    }

    return n;
}

/** 就地转成大写（只对命令词和参数名用，不碰数值） */
static void to_upper(char *s)
{
    while (*s != '\0') {
        if ((*s >= 'a') && (*s <= 'z')) {
            *s = (char)(*s - 'a' + 'A');
        }
        s++;
    }
}

/* ----------------------------------------------------------------- 参数名表 */

/** 参数名 → 枚举。找不到返回 CONTROL_PARAM_COUNT（哨兵值）。 */
static control_param_t param_lookup(const char *name)
{
    for (int i = 0; i < (int)CONTROL_PARAM_COUNT; i++) {
        if (strcmp(name, control_param_name((control_param_t)i)) == 0) {
            return (control_param_t)i;
        }
    }

    return CONTROL_PARAM_COUNT;
}

/* ----------------------------------------------------------------- 命令实现 */

static void cmd_set(char *const *tok, uint8_t n)
{
    resp_t          r;
    control_param_t p;
    float           value;

    if (n < 3U) {
        resp_error("USAGE", "SET <NAME> <VALUE>");
        return;
    }

    p = param_lookup(tok[1]);
    if (p == CONTROL_PARAM_COUNT) {
        resp_error("UNKNOWN_PARAM", tok[1]);
        return;
    }

    if (!parse_float(tok[2], &value)) {
        resp_error("BAD_VALUE", tok[2]);
        return;
    }

    if (control_param_set(p, value) != ERR_OK) {
        /* 走到这里说明格式没问题，是**取值范围**被拒了。
         * 与 BAD_VALUE 分开报，agent 才知道该改格式还是该改数。 */
        resp_error("OUT_OF_RANGE", tok[1]);
        return;
    }

    resp_reset(&r);
    resp_str(&r, "OK ");
    resp_str(&r, tok[1]);
    resp_ch(&r, '=');
    resp_f3(&r, control_param_get(p));
    resp_send(&r);
}

static void cmd_get(char *const *tok, uint8_t n)
{
    resp_t r;

    if (n < 2U) {
        resp_error("USAGE", "GET <NAME>|ALL");
        return;
    }

    if (strcmp(tok[1], "ALL") == 0) {
        resp_reset(&r);
        resp_str(&r, "OK");
        for (int i = 0; i < (int)CONTROL_PARAM_COUNT; i++) {
            resp_ch(&r, ' ');
            resp_str(&r, control_param_name((control_param_t)i));
            resp_ch(&r, '=');
            resp_f3(&r, control_param_get((control_param_t)i));
        }
        resp_str(&r, " TARGET=");
        resp_i32(&r, control_position_target());
        resp_send(&r);
        return;
    }

    const control_param_t p = param_lookup(tok[1]);
    if (p == CONTROL_PARAM_COUNT) {
        resp_error("UNKNOWN_PARAM", tok[1]);
        return;
    }

    resp_reset(&r);
    resp_str(&r, "OK ");
    resp_str(&r, tok[1]);
    resp_ch(&r, '=');
    resp_f3(&r, control_param_get(p));
    resp_send(&r);
}

static void cmd_stat(void)
{
    resp_t          r;
    control_status_t st;

    control_get_status(&st);

    resp_reset(&r);
    resp_str(&r, "OK ANGLE=");  resp_u32(&r, st.angle);
    resp_str(&r, " POS=");      resp_i32(&r, st.position);
    resp_str(&r, " SPD=");      resp_i32(&r, (int32_t)st.speed);
    resp_str(&r, " ATAR=");     resp_f3(&r, st.angle_target);
    resp_str(&r, " AOUT=");     resp_f3(&r, st.angle_out);
    resp_str(&r, " POUT=");     resp_f3(&r, st.pos_out);
    resp_str(&r, " PWM=");      resp_i32(&r, (int32_t)st.pwm);
    resp_str(&r, " RUN=");      resp_u32(&r, (st.state == CONTROL_STATE_RUN) ? 1U : 0U);
    /*
     * ST 是「总状态」（0 停 / 1 双环 / 2 启摆），RUN 是「双环在不在跑」。
     * 两个都给是有意的：启摆期间 RUN=0 而 ST=2，**光看 RUN 分不出
     * 「停着」和「正在启摆」**，而 agent 对这两种情况的反应完全不同。
     *
     * SWR 是上一次启摆的结局（0 没启摆过 / 1 成功 / 2 超时 / 3 被打断）。
     * 同样是"停着"，超时停和被按停要分开。
     */
    resp_str(&r, " ST=");       resp_u32(&r, (uint32_t)st.state);
    resp_str(&r, " SWR=");      resp_u32(&r, (uint32_t)st.swing_result);
    resp_send(&r);
}

static void cmd_run(void)
{
    resp_t r;
    const error_t e = control_start();

    resp_reset(&r);

    if (e == ERR_OK) {
        resp_str(&r, "OK RUN=1");
    } else if (e == ERR_NOT_READY) {
        /* 这是**最常遇到**的失败，而且原因很具体，必须说清楚，
         * 否则 agent 只会看到「启动不了」而反复重试。 */
        resp_str(&r, "ERR NOT_READY ANGLE_OUT_OF_WINDOW");
    } else {
        resp_str(&r, "ERR NOT_INITIALIZED");
    }

    resp_send(&r);
}

static void cmd_swing(void)
{
    resp_t          r;
    control_status_t st;
    const error_t   e = control_swing_up();

    resp_reset(&r);

    if (e != ERR_OK) {
        resp_str(&r, "ERR NOT_INITIALIZED");
        resp_send(&r);
        return;
    }

    /* 把结果状态回给 agent：
     *   ST=1 → 角度本来就在启动窗口内，直接进了双环（没启摆）
     *   ST=2 → 正在启摆，摆杆荡进窗口后会自动转成 ST=1
     * 两者后续该做的事不同，所以要分得开。 */
    control_get_status(&st);
    resp_str(&r, "OK ST=");
    resp_u32(&r, (uint32_t)st.state);
    resp_send(&r);
}

static void cmd_stop(void)
{
    resp_t r;

    (void)control_stop();

    resp_reset(&r);
    resp_str(&r, "OK RUN=0");
    resp_send(&r);
}

static void cmd_zero(void)
{
    resp_t r;

    (void)control_zero_position();

    resp_reset(&r);
    resp_str(&r, "OK POS=0 TARGET=0");
    resp_send(&r);
}

static void cmd_target(char *const *tok, uint8_t n)
{
    resp_t   r;
    int32_t  target;

    if (n < 2U) {
        resp_error("USAGE", "TARGET <int>");
        return;
    }

    if (!parse_i32(tok[1], &target)) {
        resp_error("BAD_VALUE", tok[1]);
        return;
    }

    /* 限位检查由 control_set_position_target() 负责（±CONTROL_POS_TARGET_LIMIT，
     * 与按键路径共用同一份规则），这里只负责把它翻译成给 agent 看的报文。 */
    if (control_set_position_target(target) != ERR_OK) {
        resp_error("OUT_OF_RANGE", tok[1]);
        return;
    }

    resp_reset(&r);
    resp_str(&r, "OK TARGET=");
    resp_i32(&r, target);
    resp_send(&r);
}

static void cmd_stream(char *const *tok, uint8_t n)
{
    resp_t r;

    if (n < 2U) {
        resp_error("USAGE", "STREAM <0|1>");
        return;
    }

    if ((strcmp(tok[1], "0") != 0) && (strcmp(tok[1], "1") != 0)) {
        resp_error("BAD_VALUE", tok[1]);
        return;
    }

    s_stream_on      = (tok[1][0] == '1');
    s_stream_last_ms = HAL_GetTick();

    resp_reset(&r);
    resp_str(&r, "OK STREAM=");
    resp_u32(&r, s_stream_on ? 1U : 0U);
    resp_send(&r);
}

static void cmd_help(void)
{
    resp_t r;

    /*
     * 命令表和参数表都**动态拼**出来，不写死字符串 ——
     * 以后加一个参数或一条命令，只需要改枚举，HELP 自动跟上，
     * 不会出现「文档说支持、实际不支持」这种事。
     */
    resp_reset(&r);
    resp_str(&r, "OK CMD=");
    resp_str(&r, "SET,GET,STAT,RUN,SWING,STOP,ZERO,TARGET,STREAM,HELP");
    resp_str(&r, " PARAM=");
    for (int i = 0; i < (int)CONTROL_PARAM_COUNT; i++) {
        if (i > 0) {
            resp_ch(&r, ',');
        }
        resp_str(&r, control_param_name((control_param_t)i));
    }
    resp_send(&r);
}

/* ----------------------------------------------------------------- 分发 */

static void console_execute(void)
{
    char   *tok[CONSOLE_ARG_MAX];
    uint8_t n;

    s_line[s_line_len] = '\0';
    s_line_len = 0U;

    if (s_line_overflow) {
        s_line_overflow = false;
        resp_error("LINE_TOO_LONG", NULL);
        return;
    }

    n = tokenize(s_line, tok, CONSOLE_ARG_MAX);
    if (n == 0U) {
        return;                     /* 空行（连按回车）不是错误，静默忽略 */
    }

    to_upper(tok[0]);
    if (n >= 2U) {
        to_upper(tok[1]);           /* 参数名/关键字；数值不受影响 */
    }

    if (strcmp(tok[0], "SET") == 0) {
        cmd_set(tok, n);
    } else if (strcmp(tok[0], "GET") == 0) {
        cmd_get(tok, n);
    } else if (strcmp(tok[0], "STAT") == 0) {
        cmd_stat();
    } else if (strcmp(tok[0], "RUN") == 0) {
        cmd_run();
    } else if (strcmp(tok[0], "SWING") == 0) {
        cmd_swing();
    } else if (strcmp(tok[0], "STOP") == 0) {
        cmd_stop();
    } else if (strcmp(tok[0], "ZERO") == 0) {
        cmd_zero();
    } else if (strcmp(tok[0], "TARGET") == 0) {
        cmd_target(tok, n);
    } else if (strcmp(tok[0], "STREAM") == 0) {
        cmd_stream(tok, n);
    } else if (strcmp(tok[0], "HELP") == 0) {
        cmd_help();
    } else {
        resp_error("UNKNOWN_CMD", tok[0]);
    }
}

static void console_feed(uint8_t ch)
{
    /* 换行：'\r' 单独也算一次（老式串口工具只发 CR）。
     * CRLF 只当一次 —— 见 s_saw_cr 的说明。 */
    if ((ch == '\n') || (ch == '\r')) {
        if ((ch == '\n') && s_saw_cr) {
            s_saw_cr = false;
            return;
        }
        s_saw_cr = (ch == '\r');
        console_execute();
        return;
    }
    s_saw_cr = false;

    /* 退格：手工敲命令时有用。0x7F 是大多数终端的退格键。 */
    if ((ch == 0x08U) || (ch == 0x7FU)) {
        if (s_line_len > 0U) {
            s_line_len--;
        }
        return;
    }

    /* 其余控制字符（比如 Tab、转义序列）一概丢弃：
     * 它们进不了命令语法，留着只会让分词莫名其妙地失败。 */
    if (ch < 0x20U) {
        return;
    }

    if (s_line_len >= CONSOLE_LINE_MAX) {
        /* 超长：标记并继续吃字符，**等换行来了再报错**。
         * 如果在这里直接报错，后面剩下的字符会被当成一条新命令，
         * 于是 agent 收到一串莫名其妙的 ERR。 */
        s_line_overflow = true;
        return;
    }

    s_line[s_line_len] = (char)ch;
    s_line_len++;
}

/* ----------------------------------------------------------------- 数据流 */

/**
 * @brief STREAM 模式：按周期吐一行 CSV 实时数据，供 SerialPlot 画曲线。
 *
 * 列序固定为  ANGLE,POS,SPD,ATAR,AOUT,POUT,PWM,RUN
 * 与 `STAT` 的字段一一对应，顺序也一致 —— 这样 agent 只要解析过 STAT，
 * 就知道每一列是什么。
 */
static void console_stream_tick(void)
{
    resp_t           r;
    control_status_t st;

    if (!s_stream_on) {
        return;
    }

    const uint32_t now = HAL_GetTick();
    if ((now - s_stream_last_ms) < CONSOLE_STREAM_PERIOD_MS) {
        return;
    }
    s_stream_last_ms = now;

    control_get_status(&st);

    resp_reset(&r);
    resp_u32(&r, st.angle);                                   resp_ch(&r, ',');
    resp_i32(&r, st.position);                                resp_ch(&r, ',');
    resp_i32(&r, (int32_t)st.speed);                          resp_ch(&r, ',');
    resp_f3(&r, st.angle_target);                             resp_ch(&r, ',');
    resp_f3(&r, st.angle_out);                                resp_ch(&r, ',');
    resp_f3(&r, st.pos_out);                                  resp_ch(&r, ',');
    resp_i32(&r, (int32_t)st.pwm);                            resp_ch(&r, ',');
    resp_u32(&r, (st.state == CONTROL_STATE_RUN) ? 1U : 0U);
    resp_send(&r);
}

/* ----------------------------------------------------------------- 对外接口 */

error_t console_init(void)
{
    s_line_len       = 0U;
    s_line_overflow  = false;
    s_saw_cr         = false;
    s_stream_on      = false;
    s_stream_last_ms = 0U;

    return ERR_OK;
}

bool console_is_streaming(void)
{
    return s_stream_on;
}

void console_poll(void)
{
    uint8_t  chunk[CONSOLE_READ_CHUNK];
    uint32_t n;

    /* 一次把接收缓冲抽干。取出多少处理多少，绝不等下一个字节。 */
    while ((n = bsp_serial_read(chunk, (uint32_t)sizeof(chunk))) > 0U) {
        for (uint32_t i = 0U; i < n; i++) {
            console_feed(chunk[i]);
        }
    }

    console_stream_tick();
}
