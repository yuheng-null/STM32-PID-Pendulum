/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_oled.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"

#include "bsp_tick.h"
#include "bsp_key.h"
#include "bsp_pot.h"
#include "bsp_serial.h"

/* 倒立摆三件套：电机、编码器、角度传感器 */
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_angle.h"

/* 双环 PID 控制器 + 串口调参控制台 */
#include "control.h"
#include "console.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * 屏幕上各元素的位置（单位：像素）。
 *
 * 版式是**左右双列**（仿官方）：左列是内环（角度环），右列是外环（位置环），
 * 这样调参时一眼能看出「改的是哪个环、两个环现在各在干什么」。
 *
 * Font_6x8 宽 6 像素、屏宽 128，一行放得下 21 个字符。
 * 两列各占 10 个字符，第二列从第 11 个字符（x=66）开始。
 */
#define OLED_LINE_TITLE_Y       (0U)    /* 标题行，Font_7x10，占 0~9 */
#define OLED_COL0_X             (0U)    /* 左列：角度环 */
#define OLED_COL1_X             (66U)   /* 右列：位置环 */

/*
 * 数据行行距 8 像素 —— 刚好等于 Font_6x8 的字高，行间不留空隙。
 * 标题占 0~9，5 行数据从 12 开始：12~19、20~27、28~35、36~43、44~51，
 * 底部 54~61 留给按键提示。
 */
#define OLED_ROW_Y0             (12U)
#define OLED_ROW_STEP           (8U)

/* 按键提示行 */
#define OLED_HINT_Y             (54U)

/* 状态显示的位置（标题行右侧） */
#define OLED_STATE_X            (96U)

/* PC13 LED 心跳周期（毫秒） */
#define LED_HEARTBEAT_MS        (500U)

/*
 * 屏幕重画周期（毫秒）。
 *
 * ⚠️ 这个值不是「随便定个好看的数」，它直接决定主循环的最坏周期，
 *    进而决定串口命令的吞吐和 STREAM 的采样率。
 *
 *    **实测：刷一整屏要约 122 ms。**
 *    128×64 的显存是 1024 字节，全屏刷新是「整块重发」，走软件 I2C
 *    在约 100 kHz 下就是 1024×9 bit ÷ 100 kHz ≈ 92 ms，加上命令字节
 *    和函数开销正好落到这个量级。
 *
 *    （老注释里写的「约 30 ms」是错的，那个数对应约 300 kHz 的 I2C，
 *      这个软件 I2C 根本达不到。以此为准。）
 *
 *    于是这个周期**必须显著大于 122 ms**：等于或小于它的话，
 *    「距上次刷屏已过 N 毫秒」这个条件每次循环都立刻成立，
 *    主循环就退化成「刷屏、刷屏、刷屏……」的连续循环，
 *    周期被锁死在 122 ms，串口在这段时间里根本拿不到 CPU。
 *    实测：把这里设成 100 时，STREAM 从 50 行/秒掉到 8.2 行/秒。
 *
 *    取 500 ms：屏幕 2 Hz 刷新，人眼看着完全够（调参时盯的是数字
 *    对不对，不是动画）；主循环有约 75% 的时间是空闲的，
 *    STREAM 能跑到 40 行/秒上下，命令响应也在 120 ms 以内。
 *
 *    这里**不用死区判断**（老版本那套）：新界面里 Out 那两列每 5 ms
 *    都在变，死区永远成立，等于每轮主循环都重画 —— 正是要避免的情形。
 */
#define DISPLAY_PERIOD_MS       (500U)

/* 每次按 K2/K3 增减的位置目标步长（边沿）。408 = 横杆转一圈。 */
#define POS_TARGET_STEP         (408)

/*
 * 本台设备的中心角度 = 2086（2026-10-09 手扶实测）。
 *
 * 这个值现在已经搬到控制层去了（control.c 的 CONTROL_DEFAULT_CENTER），
 * 因为它属于「被控对象的标定」，不属于应用逻辑。想改中心角度有两条路：
 *   · 串口：`SET CENTER 2086`
 *   · 或改 control.c 的默认值（掉电后仍生效的那个）
 *
 * 标定依据（12.22 码/度、满量程 335° 与手册 333° 吻合、盲区说明）
 * 记在 control.h 的文件头里，那里才是它该在的地方。
 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* OLED 设备实例（含 1 KB 显存），文件级静态，不占栈 */
static ssd1306_t s_oled;

/*
 * 现在**没有**「应用层运行状态」这个变量了。
 *
 * 老版本这里有一对 s_run / s_duty_cmd，因为那时应用层要自己拿捏
 * 「按 K1 切状态、把占空比写到电机」。现在这件事整个归 control 模块管
 * （它有自己的状态机和双环），main.c 只管转发按键和显示。
 *
 * 要紧的是**状态只有一份**：如果 main.c 再存一份 s_run，就和
 * control 内部那份成了两个真相，迟早会出现「屏幕显示 RUN、
 * 实际已经因为摔倒自动停机了」这种对不上的情况。
 * 需要的时候问 `control_is_running()` 就行。
 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t oled_u32_to_dec(uint32_t value, char *out);
static uint8_t oled_i32_to_dec(int32_t value, char *out);
static void    oled_cell(uint16_t x, uint16_t y, const char *label, int32_t value, uint8_t digits);
static void    oled_cell_f2(uint16_t x, uint16_t y, const char *label, float value);
static void    oled_redraw(void);
static void    app_pos_target_step(int32_t delta);
static void    app_handle_keys(void);
static void    app_refresh_display(void);
static void    app_fatal_blink(uint8_t code);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  把无符号整数按十进制转成字符串。
  * @param  value 待转换的值
  * @param  out   输出缓冲，调用者需保证至少 11 字节
  * @retval 写入的字符数（不含结尾 '\0'）
  * @note   不用 snprintf，避免为了显示一个数字把整套 printf 拖进 Flash。
  */
static uint8_t oled_u32_to_dec(uint32_t value, char *out)
{
  char    tmp[10];
  uint8_t n = 0U;

  if (value == 0U) {
    out[0] = '0';
    out[1] = '\0';
    return 1U;
  }

  /* 先逆序取各位，再翻回来 */
  while ((value > 0U) && (n < sizeof(tmp))) {
    tmp[n] = (char)('0' + (value % 10U));
    value /= 10U;
    n++;
  }

  for (uint8_t i = 0U; i < n; i++) {
    out[i] = tmp[n - 1U - i];
  }
  out[n] = '\0';

  return n;
}

/**
  * @brief  把有符号整数按十进制转成字符串（带负号）。
  * @param  value 待转换的值
  * @param  out   输出缓冲，调用者需保证至少 13 字节
  * @retval 写入的字符数（不含结尾 '\0'）
  * @note   用于编码器的位置/速度与电机占空比——这些量都有正负。
  */
static uint8_t oled_i32_to_dec(int32_t value, char *out)
{
  if (value < 0) {
    /*
     * 取绝对值时先 +1 再取反再加 1，避免对 INT32_MIN 直接取反溢出。
     * 本工程用不到这个极值，但这样写不需要额外的边界假设。
     */
    out[0] = '-';
    return (uint8_t)(1U + oled_u32_to_dec((uint32_t)(-(value + 1)) + 1U, &out[1]));
  }

  return oled_u32_to_dec((uint32_t)value, out);
}

/**
  * @brief  在任意坐标写一段以 '\0' 结尾的文本（统一用 Font_6x8）。
  * @param  x,y  左上角像素坐标
  * @note   一行 128 像素 / 每字符 6 像素 = **最多 21 个字符**，
  *         超出的部分会被 ssd1306_write_string 静默截断。
  */
static void oled_text(uint16_t x, uint16_t y, const char *text)
{
  (void)ssd1306_set_cursor(&s_oled, x, y);
  (void)ssd1306_write_string(&s_oled, text, &Font_6x8, SSD1306_WHITE);
}

/**
  * @brief  把浮点转成定点两位小数的字符串（如 "-1800.00"）。
  * @param  value 待转换的值
  * @param  out   输出缓冲，调用者需保证至少 13 字节
  * @retval 写入的字符数（不含结尾 '\0'）
  *
  * 屏幕上不显示浮点的原始精度：增益的有效位数就到百分位，
  * 多打几位只是让数字在跳，反而看不清。
  */
static uint8_t oled_f2_to_dec(float value, char *out)
{
  char     num[13];
  uint8_t  k = 0U;
  int32_t  cents;
  uint32_t mag;

  /* 防御：NaN 进去会让下面两个比较都不成立，cents 变成垃圾值。
   * 正常的增益不会走到这里（control_param_set 已经挡过）。 */
  if (value != value) {
    out[0] = 'n'; out[1] = 'a'; out[2] = 'n'; out[3] = '\0';
    return 3U;
  }

  /* 夹一下再乘 100：不夹的话 value*100 可能溢出 int32，是未定义行为 */
  if (value >  99999.0f) { value =  99999.0f; }
  if (value < -99999.0f) { value = -99999.0f; }

  /* 四舍五入到百分位。加 ±0.5 而不是用 roundf()，省一个库依赖。 */
  cents = (int32_t)((value * 100.0f) + ((value >= 0.0f) ? 0.5f : -0.5f));

  if (cents < 0) {
    out[k] = '-';
    k++;
  }

  /* 取相反数走 uint32 中转：int32 的 -(-2147483648) 会溢出 */
  mag = (cents < 0) ? ((uint32_t)(-(cents + 1)) + 1U) : (uint32_t)cents;

  (void)oled_u32_to_dec(mag / 100U, num);
  for (const char *p = num; *p != '\0'; p++) {
    out[k] = *p;
    k++;
  }

  out[k] = '.';                                   k++;
  out[k] = (char)('0' + ((mag % 100U) / 10U));    k++;
  out[k] = (char)('0' + (mag % 10U));             k++;
  out[k] = '\0';

  return k;
}

/**
  * @brief  写一个「3 字符标签 + 右对齐数值」的显示单元。
  * @param  x,y    单元左上角像素坐标
  * @param  label  标签，固定 3 个字符（如 "AKP"）
  * @param  value  数值
  * @param  digits 数值占的字符宽度，本界面统一用 7
  *
  * 整个单元固定 3+digits 个字符宽，左右两列各排一列。
  *
  * ⚠️ 数值**必须补齐**（右对齐），不能不足就让后面的内容左移 ——
  *    否则数值位数一变整行就会左右跳动，看起来像屏幕在抖，
  *    而且两列对不齐。这是显示代码里最常见的毛病。
  */
static void oled_cell(uint16_t x, uint16_t y, const char *label, int32_t value, uint8_t digits)
{
  char    num[13];
  char    msg[16];
  uint8_t k = 0U;
  uint8_t n;

  for (const char *p = label; *p != '\0'; p++) {
    msg[k] = *p;
    k++;
  }

  n = oled_i32_to_dec(value, num);

  for (uint8_t i = n; i < digits; i++) {
    msg[k] = ' ';
    k++;
  }
  for (uint8_t i = 0U; i < n; i++) {
    msg[k] = num[i];
    k++;
  }

  msg[k] = '\0';

  oled_text(x, y, msg);
}

/**
  * @brief  同 oled_cell，但数值是浮点、固定两位小数。
  *
  * 始终按 7 个字符宽补齐，所以和一个用 oled_cell 的单元排在同一列时
  * 也是对齐的。
  */
static void oled_cell_f2(uint16_t x, uint16_t y, const char *label, float value)
{
  char    num[13];
  char    msg[16];
  uint8_t k = 0U;
  uint8_t n;

  for (const char *p = label; *p != '\0'; p++) {
    msg[k] = *p;
    k++;
  }

  n = oled_f2_to_dec(value, num);

  for (uint8_t i = n; i < 7U; i++) {
    msg[k] = ' ';
    k++;
  }
  for (uint8_t i = 0U; i < n; i++) {
    msg[k] = num[i];
    k++;
  }

  msg[k] = '\0';

  oled_text(x, y, msg);
}

/**
  * @brief  整屏重画并按内容重新上屏。
  *
  * 版式（左右双列，左=内环角度环，右=外环位置环）：
  *
  *     Pendulum              RUN
  *     AKP   4.50   PKP   9.36
  *     AKI   0.16   PKI   0.18
  *     AKD   7.38   PKD  82.08
  *     Ang   2086   Loc      0
  *     ATr   2086   LTr      0
  *     AOu    123   POu   0.00
  *
  * **没有按键提示行**，是刻意的取舍：6 行数据已经把 64 像素的屏占满
  * （标题 0~9，数据 12~59），再挤一行提示就得砍掉一整行数据。
  * 而这一屏是为**调参**服务的，两个环的目标值比按键提示有用得多；
  * 按键功能在 README 和 docs/ 里都有。
  *
  * 整屏重画而不是只改变化的那一块：这个规模下更简单，也绝不会留残影
  * （比如数值从 1000 掉到 999 时旧的那一位干不掉）。代价是每次约 30 ms
  * 的 I2C 传输，所以调用它的频率由调用者把关（见 DISPLAY_PERIOD_MS）。
  */
static void oled_redraw(void)
{
  control_status_t        st;

  control_get_status(&st);

  ssd1306_fill(&s_oled, SSD1306_BLACK);

  /* 标题 + 运行状态。
   * 状态直接问 control 模块，不在 main 里另存一份 —— 见 PV 段的说明。
   *
   * 三态：STOP / RUN / SWG（自动启摆中）。这里**不能再用
   * control_is_running()** —— 它问的是"双环在不在跑"，启摆期间返回 false，
   * 屏上就会显示成 STOP，看着像没启动。用快照里的 state 才准。 */
  (void)ssd1306_set_cursor(&s_oled, 0U, OLED_LINE_TITLE_Y);
  (void)ssd1306_write_string(&s_oled, "Pendulum", &Font_7x10, SSD1306_WHITE);
  oled_text(OLED_STATE_X, OLED_LINE_TITLE_Y,
            (st.state == CONTROL_STATE_RUN)      ? "RUN" :
            (st.state == CONTROL_STATE_SWING_UP) ? "SWG" : "STOP");

  const uint16_t x0  = OLED_COL0_X;
  const uint16_t x1  = OLED_COL1_X;
  const uint16_t y0  = OLED_ROW_Y0;
  const uint16_t y1  = (uint16_t)(OLED_ROW_Y0 + (OLED_ROW_STEP * 1U));
  const uint16_t y2  = (uint16_t)(OLED_ROW_Y0 + (OLED_ROW_STEP * 2U));
  const uint16_t y3  = (uint16_t)(OLED_ROW_Y0 + (OLED_ROW_STEP * 3U));
  const uint16_t y4  = (uint16_t)(OLED_ROW_Y0 + (OLED_ROW_STEP * 4U));
  const uint16_t y5  = (uint16_t)(OLED_ROW_Y0 + (OLED_ROW_STEP * 5U));

  /* ① 三个增益 —— 调参时眼睛盯得最多的就是这几行。
   *    用 control_params() 拿的是**当前生效值**，和串口改的是同一份。 */
  const control_params_t *pp = control_params();

  oled_cell_f2(x0, y0, "AKP", pp->a_kp);
  oled_cell_f2(x1, y0, "PKP", pp->p_kp);
  oled_cell_f2(x0, y1, "AKI", pp->a_ki);
  oled_cell_f2(x1, y1, "PKI", pp->p_ki);
  oled_cell_f2(x0, y2, "AKD", pp->a_kd);
  oled_cell_f2(x1, y2, "PKD", pp->p_kd);

  /* ② 两个环的实测量。
   *    "Ang" 用 st.angle 而不是再采一次：取的是控制环**正在用的**
   *    那一个采样，显示和实际参与运算的完全一致。 */
  oled_cell(x0, y3, "Ang", (int32_t)st.angle, 7U);
  oled_cell(x1, y3, "Loc", st.position, 7U);

  /* ③ 两个环的目标值。
   *    ATr 在运行中会随外环输出微动 —— 那是正常的，正是串级结构
   *    在「挪动内环目标」。它一直等于 CENTER 反而说明外环没在干活。 */
  oled_cell(x0, y4, "ATr", (int32_t)st.angle_target, 7U);
  oled_cell(x1, y4, "LTr", control_position_target(), 7U);

  /* ④ 两个环的输出。
   *    AOu 是限幅后、**未叠静摩擦补偿**的值（补偿量见串口 GET OFFSET）；
   *    真正写到电机上的是 st.pwm，想看它就串口打 STAT。 */
  oled_cell(x0, y5, "AOu", (int32_t)st.angle_out, 7U);
  oled_cell_f2(x1, y5, "POu", st.pos_out);

  (void)ssd1306_update_screen(&s_oled);
}

/**
  * @brief  按步长移动位置目标，越界就**不动**（不是夹到边界）。
  *
  * 夹到边界的坏处：按到边界之后继续按没任何反馈，用户以为按键坏了。
  * 不动至少和「已经到头了」是一致的表现，而且控制台里 TARGET 命令
  * 会明确回 OUT_OF_RANGE。
  */
static void app_pos_target_step(int32_t delta)
{
  const int32_t target = control_position_target() + delta;

  if ((target >= -CONTROL_POS_TARGET_LIMIT) && (target <= CONTROL_POS_TARGET_LIMIT)) {
    (void)control_set_position_target(target);
  }
}

/**
  * @brief  处理按键事件。
  *
  * 按键分工（**改成了官方那套语义**）：
  *   K1 —— 启动 / 停止控制（启动时若摆杆不在竖直附近，会自动启摆）
  *   K2 —— 位置目标 +408（横杆正转一圈）
  *   K3 —— 位置目标 −408（横杆反转一圈）
  *   K4 —— 位置清零（位置计数与目标一起归零）并停机
  *
  * 老版本的 K2/K3 是「直接加减电机占空比」，那是纯驱动测试用的。
  * 有了闭环之后手动给占空比没有意义（控制环每 5 ms 就会覆盖掉），
  * 所以改成调位置目标。
  *
  * ⚠️ 每个键都要用 while 取干净，而不是 if 一次：一次完整的按放会
  *    同时挂起 PRESS 和 RELEASE 两个事件，只取一次会把 RELEASE 留下
  *    （见 bsp/key/bsp_key.h 的说明）。这里只响应按下沿。
  */
static void app_handle_keys(void)
{
  for (uint8_t i = 0U; i < (uint8_t)KEY_ID_COUNT; i++) {
    key_event_t ev;

    while ((ev = bsp_key_take_event((key_id_t)i)) != KEY_EVENT_NONE) {
      if (ev != KEY_EVENT_PRESS) {
        continue;                       /* 松开沿不处理 */
      }

      switch ((key_id_t)i) {
        case KEY_ID_K1:
          /* 非停止态（双环运行中 或 正在启摆）→ 停；停止态 → 启动。
           *
           * 启动用 control_swing_up() 而不是 control_start()：
           *   · 摆杆已经在竖直附近 → 它内部直接转成双环控制，行为与旧的
           *     K1 完全一样（扶着摆杆按 K1 的用法没变）；
           *   · 摆杆垂着 → 它自己去荡摆杆。以前这种情况是**静默失败**
           *     （control_start 返回 ERR_NOT_READY，屏幕上什么都不变），
           *     现在会自己荡起来。
           *
           * ⚠️ 判"停"必须连启摆一起算。`control_is_running()` 在启摆期间
           *    返回 false，只用它的话，**启摆中再按一次 K1 会变成"重新启摆"**
           *    而不是停止。所以这里看总状态 state。 */
          {
            control_status_t st;
            control_get_status(&st);

            if (st.state != CONTROL_STATE_STOP) {
              (void)control_stop();
            } else {
              (void)control_swing_up();
            }
          }
          break;

        case KEY_ID_K2:
          app_pos_target_step((int32_t)POS_TARGET_STEP);
          break;

        case KEY_ID_K3:
          app_pos_target_step(-(int32_t)POS_TARGET_STEP);
          break;

        case KEY_ID_K4:
          (void)control_zero_position();
          (void)control_stop();
          break;

        default:
          break;
      }
    }
  }
}

/**
  * @brief  按固定周期重画屏幕。
  *
  * 周期由 DISPLAY_PERIOD_MS 决定，**不按内容变化触发** —— 理由见那里的
  * 说明：新界面里 Out 那两列每 5 ms 都在变，任何死区都会永远成立，
  * 等于每轮主循环都重画，反而把主循环彻底堵死。
  */
static void app_refresh_display(void)
{
  static uint32_t last_ms = 0U;

  /*
   * STREAM 开着时**完全不刷屏**。
   *
   * 理由：刷一屏要 ~122 ms 的 CPU（见 DISPLAY_PERIOD_MS），这段时间里
   * console_poll() 拿不到 CPU，STREAM 的数据就会出现一个 122 ms 的空洞，
   * 采样率也从 50 掉到 38 行/秒。而上位机在画曲线的时候，屏幕根本没人在看。
   *
   * 所以串流期间把这 122 ms 全部让给串口 —— 换到的是**满速且等间距**的
   * 50 行/秒。这对 PID 调参是有实际价值的：曲线的横轴时间戳均匀，
   * 才能从波形的周期和衰减判断增益是否合适。
   *
   * 串流一关（STREAM 0），下一轮循环就恢复刷新，不用做别的动作。
   */
  if (console_is_streaming()) {
    return;
  }

  const uint32_t now_ms = HAL_GetTick();
  if ((now_ms - last_ms) < DISPLAY_PERIOD_MS) {
    return;
  }
  last_ms = now_ms;

  oled_redraw();
}

/**
  * @brief  初始化失败时的错误指示：快闪 code 次，停 1 秒，循环。
  *
  * 闪烁次数对应失败的模块，便于不用调试器就定位问题：
  *   1 次 = OLED（bsp_oled_init 失败，屏没应答）
  *   2 次 = 系统节拍（bsp_tick_init 失败）
  *   3 次 = 按键（bsp_key_init 失败）
  *   4 次 = 电位器（bsp_pot_init 失败，ADC 自校准没成功）
  *   5 次 = 串口（bsp_serial_init 失败，接收中断没武装上）
  *   6 次 = 电机（bsp_motor_init 失败，PWM 没启动起来）
  *   7 次 = 编码器（bsp_encoder_init 失败）
  *   8 次 = 角度传感器（bsp_angle_init 失败，ADC1 自校准没成功）
  *   9 次 = 节拍任务注册失败（槽位满了，见 BSP_TICK_MAX_HANDLERS）
  *  10 次 = 控制器（control_init 失败）
  *  11 次 = 串口控制台（console_init 失败）
  *
  * @note 本函数不返回。
  */
static void app_fatal_blink(uint8_t code)
{
  while (1) {
    for (uint8_t i = 0U; i < code; i++) {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(120);
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(120);
    }
    HAL_Delay(1000);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  /* OLED 挂在 PB8(SCL)/PB9(SDA)，走软件模拟 I2C，由 bsp_oled 自行接管引脚 */
  if (bsp_oled_init(&s_oled) != ERR_OK) {
    app_fatal_blink(1U);
  }

  /*
   * 初始化顺序有依赖：先把 1 ms 节拍跑起来，按键才能把自己的采样挂上去。
   * 反过来会注册失败（此时还没有节拍可挂）。
   */
  if (bsp_tick_init() != ERR_OK) {
    app_fatal_blink(2U);
  }
  if (bsp_key_init() != ERR_OK) {
    app_fatal_blink(3U);
  }

  /*
   * 电位器走 ADC2（把 ADC1 留给后面的模块）。
   * 必须在 CubeMX 生成的 MX_ADC2_Init() 之后调用——
   * 那边才把通道、采样时间、扫描模式写进寄存器。
   */
  if (bsp_pot_init() != ERR_OK) {
    app_fatal_blink(4U);
  }

  /*
   * 串口。必须在 CubeMX 生成的 MX_USART1_UART_Init() 之后调用——
   * 那边才把波特率、字长、校验这些寄存器配置写下去，
   * 本模块只负责把接收中断武装起来。
   */
  if (bsp_serial_init() != ERR_OK) {
    app_fatal_blink(5U);
  }

  /*
   * ── 倒立摆三件套 ──────────────────────────────────────────────
   * 顺序有讲究：先电机、再编码器、最后角度传感器。
   *
   * 电机最先，是因为它的初始化会把占空比置 0、方向脚置 0——
   * 也就是「上电不转」。这一步越早做，摆杆在初始化期间被动到的
   * 可能性越小。
   *
   * 三者都必须在 CubeMX 生成的 MX_TIM2_Init() / MX_TIM3_Init() /
   * MX_ADC1_Init() 之后调用（那三个函数在 main 开头已经调过）。
   */
  if (bsp_motor_init() != ERR_OK) {
    app_fatal_blink(6U);
  }
  if (bsp_encoder_init() != ERR_OK) {
    app_fatal_blink(7U);
  }
  if (bsp_angle_init() != ERR_OK) {
    app_fatal_blink(8U);
  }

  /*
   * 控制器。必须在三个驱动都初始化之后 —— control_init() 会读一次
   * 角度和编码器来对齐内部快照，驱动没起来的话读到的是 0。
   */
  if (control_init() != ERR_OK) {
    app_fatal_blink(10U);
  }

  /*
   * 串口控制台（调参协议）。只是个协议层，不碰 USART 寄存器，
   * 但要在 bsp_serial_init() 之后。
   */
  if (console_init() != ERR_OK) {
    app_fatal_blink(11U);
  }

  /*
   * ── 把控制环挂到 1 ms 节拍上 ──────────────────────────────────
   *
   * 这是本工程**唯一**挂在节拍上的任务（按键模块自己也占一个槽位）。
   *
   * ⚠️ 控制环为什么必须在中断里、不能留在主循环：
   *    主循环里刷一次屏要 ~30 ms 的软件 I2C，而角度环的周期是 5 ms。
   *    放在主循环里，角度环的实际周期会在 5~35 ms 之间乱跳 ——
   *    等于给控制器灌了一路巨大的周期噪声，摆杆根本立不住。
   *    挂在节拍上才有稳定的 5 ms / 50 ms。
   *
   * 主循环剩下的都是对时间不敏感的事：按键、串口、显示。
   */
  if (bsp_tick_register(control_tick) != ERR_OK) {
    app_fatal_blink(9U);
  }

  /*
   * 画一次初始界面。
   * 控制器此刻是 STOP，电机已经被 control_init() 置成滑行 ——
   * 上电时摆杆应该是静止下垂的，这一刻不能有任何动作。
   */
  oled_redraw();

  uint32_t last_led_ms = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*
     * 前台主循环：按键、串口、显示、心跳灯。
     *
     * 注意这里**没有 HAL_Delay**。在循环里死等会让所有周期性动作
     * 互相拖累（比如屏幕每 30 ms 刷一次，若用延时来定时，
     * 串口的处理周期就变成了「30 ms + 延时」）。
     * 所以一律用「查时间、时间到了才做」的非阻塞写法。
     *
     * ⚠️ 主循环里**没有**「把控制结果写到电机」这一步：那是
     *    control_tick() 在中断里做的。主循环里再写一次只会和它打架。
     *    同理也没有角度采样 —— 采样在中断里按 1 ms 恒定节奏做，
     *    这里是 5~35 ms 的抖动节奏，采出来的值不能用。
     */
    console_poll();
    app_handle_keys();
    app_refresh_display();

    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - last_led_ms) >= LED_HEARTBEAT_MS) {
      last_led_ms = now_ms;
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1799;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_AIN1_Pin|MOTOR_AIN2_Pin|OLED_SCL_Pin|OLED_SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY1_Pin KEY2_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_AIN1_Pin MOTOR_AIN2_Pin */
  GPIO_InitStruct.Pin = MOTOR_AIN1_Pin|MOTOR_AIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY3_Pin KEY4_Pin */
  GPIO_InitStruct.Pin = KEY3_Pin|KEY4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
