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

/* printf 走 bsp_serial 的发送缓冲（_write 已在那里重定向） */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 屏幕上各元素的位置（单位：像素）。集中在这里，改版式不用翻代码。 */
#define OLED_LINE_TITLE_Y       (0U)    /* 标题，Font_7x10，占 0~9      */

/*
 * 数据行行距 10 像素。Font_6x8 高 8，所以 10 像素只留 2 像素间隙，
 * 刚好不重叠又紧凑。标题占 0~9，数据行从 12 开始：
 *   12~19、22~29、32~39、42~49，底部 54~61 留给按键提示。
 */
#define OLED_ROW_Y0             (12U)
#define OLED_ROW_STEP           (10U)

/* 按键提示行 */
#define OLED_HINT_Y             (54U)

/* PC13 LED 心跳周期（毫秒） */
#define LED_HEARTBEAT_MS        (500U)

/*
 * 角度采样周期（毫秒）。
 *
 * 5 ms 是参考实现里角度环的控制周期——现在就按这个周期采，
 * 等接上 PID 时不用改采样节奏。
 */
#define ANGLE_SAMPLE_MS         (5U)

/* 每次按键增减的占空比（百分点） */
#define MOTOR_DUTY_STEP         (10)

/*
 * 显示死区。只有变化超过阈值才重画屏幕。
 *
 * 为什么需要：ADC 末位总在抖（±1~2 LSB），而刷一屏要 ~30 ms 的 I2C 传输。
 * 若「一变就重画」，摆杆静止时屏幕也在不停重刷，既浪费又让末位数字闪。
 */
#define ANGLE_DISPLAY_DEADBAND  (8U)
#define ENCODER_DISPLAY_DEADBAND (2)

/*
 * ── 本台设备的标定值（2026-10-09 实测，用 bsp_angle_raw() 读）────────
 * 下面这些**不是猜的**，是在这台机器上量出来的。换一台设备要重新量。
 *
 *   摆杆竖直（手扶测得）  raw ≈ 2086   ⚠️ 见下面的「测法」说明
 *   水平向左（逆时针90°） raw ≈  960
 *   水平向右（顺时针90°） raw ≈ 3160
 *
 * 由此推算：
 *   · 12.22 LSB/度（每 90° 约 1100 LSB，两方向对称到 4.7% 以内）
 *   · 满量程 4095 LSB ÷ 12.22 = 335°，与手册标称的「有效角度 333°」
 *     吻合到 0.6% —— 说明 ADC 线性、通道正确、传感器型号相符
 *   · 读数随「顺时针转动而增大」
 *
 * ⚠️ 测法要交代清楚：2086 是**用手扶到竖直**读的，手扶精度就是 3° 量级
 *    （12.22 LSB/度 → 2086 与理想中点 2048 差 38 LSB ≈ 3.1°），
 *    所以这个数**还带着手测误差**，不能当作最终值。
 *
 *    **权威测法是让摆杆自由下垂**：自由悬挂的摆杆，重心必然停在支点
 *    正下方 —— 这就是「竖直」的定义（由重力给出，不是由人眼给出）。
 *    松手等它停稳再读，误差就消掉了。调 PID 之前应当这样复核一次。
 *
 * ⚠️ 但无论如何**不能用理想的 2048**：2048 是电位器的**电气中点**，
 *    与「摆杆竖直」没有任何物理必然联系 —— 两者是否重合取决于机械
 *    安装相位。参考实现给的是一个**区间**（1900~2200）而不是一个数，
 *    正说明中心值要被测出来。
 *    用错中心的后果（参考实现明确警告过）：摆杆总是往一个方向跑、
 *    位置环出现稳态误差。3° 看着小，但角度环增益很大，会被放大。
 *
 * ⚠️ 参考实现取「中心 ±500 LSB」为可调控区间，换算到本台设备约
 *    ±41°。超出这个范围就说明摆杆已经倒了，应当停机而不是继续调。
 * ────────────────────────────────────────────────────────────────
 */
#define ANGLE_CENTER_MEASURED   (2086U)   /* 手扶测得，待自由下垂复核 */
#define ANGLE_LSB_PER_DEGREE    (12.22)
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
 * 上一次真正画到屏幕上的值。
 * 用它和新采样值比较，决定要不要重画——见 ANGLE_DISPLAY_DEADBAND 的说明。
 */
static uint16_t s_shown_angle = 0U;
static int32_t  s_shown_total = 0;
static int16_t  s_shown_duty  = 0;
static bool     s_shown_run   = false;

/*
 * 应用层的运行状态与占空比目标值。
 *
 * 为什么要有 s_duty_cmd 而不是直接问 bsp_motor_duty()：
 * 停机时驱动层被强制成 0（coast），但用户按 K2/K3 设的值要留着，
 * 下次按 K1 启动时接着用。所以「命令值」存在应用层，
 * 「实际值」由驱动层维护，两者分开。
 */
static int16_t   s_duty_cmd = 0;
static bool      s_run      = false;
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
static void    oled_row(uint16_t y, const char *text);
static void    oled_redraw(void);
static void    app_1ms_task(void);
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
  * @brief  在指定行写一行以 '\0' 结尾的文本（统一用 Font_6x8）。
  * @param  y    该行顶部像素坐标
  * @param  text 待写字符串
  * @note   一行 128 像素 / 每字符 6 像素 = **最多 21 个字符**，
  *         超出的部分会被 ssd1306_write_string 静默截断。
  */
static void oled_row(uint16_t y, const char *text)
{
  (void)ssd1306_set_cursor(&s_oled, 0U, y);
  (void)ssd1306_write_string(&s_oled, text, &Font_6x8, SSD1306_WHITE);
}

/**
  * @brief  写一行「标签 + 右对齐数值」。
  * @param  y      该行顶部像素坐标
  * @param  label  左侧标签（如 "Spd"）
  * @param  value  数值（有符号）
  * @param  digits 数值占的字符宽度（含负号）
  *
  * 右对齐是必要的：不补空格的话，数值从 4 位掉到 3 位时后面的内容
  * 会整体左移，看起来像在「抖」。
  */
static void oled_row_value(uint16_t y, const char *label, int32_t value, uint8_t digits)
{
  char    num[13];
  char    msg[22];
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

  oled_row(y, msg);
}

/**
  * @brief  角度行：同时显示原始值与折算电压。
  *
  * 两个值一起显示是有意的——**电压是验证接线的判据**：
  * 把摆杆从一端缓慢转到另一端，电压应从接近 0 连续变到接近 3300 mV。
  * 恒为 0 或恒为满量程就说明接线有问题，而不是代码问题。
  */
static void oled_draw_angle_row(uint16_t y)
{
  char    num[13];
  char    msg[22];
  uint8_t k = 0U;
  uint8_t n;

  msg[k] = 'A'; k++;
  msg[k] = 'n'; k++;
  msg[k] = 'g'; k++;

  n = oled_u32_to_dec((uint32_t)bsp_angle_raw(), num);
  for (uint8_t i = n; i < 5U; i++) { msg[k] = ' '; k++; }
  for (uint8_t i = 0U; i < n; i++) { msg[k] = num[i]; k++; }

  msg[k] = ' '; k++;

  n = oled_u32_to_dec((uint32_t)bsp_angle_millivolt(), num);
  for (uint8_t i = n; i < 4U; i++) { msg[k] = ' '; k++; }
  for (uint8_t i = 0U; i < n; i++) { msg[k] = num[i]; k++; }

  msg[k] = 'm'; k++;
  msg[k] = 'V'; k++;
  msg[k] = '\0';

  oled_row(y, msg);
}

/**
  * @brief  占空比行：数值 + 运行状态。
  * @param  run true 显示 RUN，false 显示 STOP
  */
static void oled_draw_duty_row(uint16_t y, int16_t duty, bool run)
{
  char        num[13];
  char        msg[22];
  uint8_t     k = 0U;
  const char *state = run ? "RUN" : "STOP";
  const uint8_t n = oled_i32_to_dec((int32_t)duty, num);

  msg[k] = 'D'; k++;
  msg[k] = 'u'; k++;
  msg[k] = 't'; k++;
  msg[k] = 'y'; k++;

  for (uint8_t i = n; i < 5U; i++) { msg[k] = ' '; k++; }
  for (uint8_t i = 0U; i < n; i++) { msg[k] = num[i]; k++; }

  msg[k] = ' '; k++;
  msg[k] = ' '; k++;

  for (const char *p = state; *p != '\0'; p++) { msg[k] = *p; k++; }

  msg[k] = '\0';

  oled_row(y, msg);
}

/**
  * @brief  整屏重画并按内容重新上屏。
  *
  * 整屏重画而不是只改变化的那一块：这个规模下更简单，也绝不会留残影
  * （比如数值从 1000 掉到 999 时旧的那一位干不掉）。代价是每次约 30 ms
  * 的 I2C 传输，所以**调用前必须先用死区判断值是否真的变了**
  * （见 app_refresh_display），否则摆杆静止时屏幕也在不停空刷。
  */
static void oled_redraw(void)
{
  ssd1306_fill(&s_oled, SSD1306_BLACK);

  /* 标题 */
  (void)ssd1306_set_cursor(&s_oled, 0U, OLED_LINE_TITLE_Y);
  (void)ssd1306_write_string(&s_oled, "Pendulum", &Font_7x10, SSD1306_WHITE);

  oled_draw_angle_row(OLED_ROW_Y0);
  oled_row_value((uint16_t)(OLED_ROW_Y0 + (OLED_ROW_STEP * 1U)),
                 "Spd", (int32_t)bsp_encoder_delta(), 5U);
  oled_row_value((uint16_t)(OLED_ROW_Y0 + (OLED_ROW_STEP * 2U)),
                 "Pos", bsp_encoder_total(), 7U);
  oled_draw_duty_row((uint16_t)(OLED_ROW_Y0 + (OLED_ROW_STEP * 3U)), s_duty_cmd, s_run);

  oled_row(OLED_HINT_Y, "K1 run  K2+  K3-  K4 clr");

  (void)ssd1306_update_screen(&s_oled);
}

/**
  * @brief  1 ms 节拍任务：只推进一次编码器游标。
  *
  * 整个任务只做一件事——读一次 TIM3 的 CNT 并累加到位置里。这是纯
  * 寄存器访问，几十个周期就结束，符合「快进快出」。
  *
  * ⚠️ **角度采样不在这里**：bsp_angle_sample() 内部靠 HAL_GetTick()
  *    做超时判断，而 1 ms 节拍所在的 TIM1 抢占优先级（1）高于
  *    SysTick（15），在节拍任务里 uwTick 不涨——一旦某次转换没有正常
  *    完成，等待会永久卡死，整个系统从此失去节拍。它留在主循环。
  */
static void app_1ms_task(void)
{
  (void)bsp_encoder_update();
}

/**
  * @brief  处理按键事件并同步电机状态。
  *
  * 按键分工：
  *   K1 —— 启动 / 停止（停止 = 滑行，不是刹车）
  *   K2 —— 占空比 +10
  *   K3 —— 占空比 -10
  *   K4 —— 编码器位置清零，并停机
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
          s_run = !s_run;
          break;

        case KEY_ID_K2:
          s_duty_cmd = (int16_t)(s_duty_cmd + MOTOR_DUTY_STEP);
          if (s_duty_cmd > (int16_t)BSP_MOTOR_DUTY_MAX) {
            s_duty_cmd = (int16_t)BSP_MOTOR_DUTY_MAX;
          }
          break;

        case KEY_ID_K3:
          s_duty_cmd = (int16_t)(s_duty_cmd - MOTOR_DUTY_STEP);
          if (s_duty_cmd < -(int16_t)BSP_MOTOR_DUTY_MAX) {
            s_duty_cmd = -(int16_t)BSP_MOTOR_DUTY_MAX;
          }
          break;

        case KEY_ID_K4:
          (void)bsp_encoder_reset();
          s_duty_cmd = 0;
          s_run      = false;
          break;

        default:
          break;
      }
    }
  }
}

/**
  * @brief  把应用层的运行状态/占空比落到电机上。
  *
  * 每轮主循环都调一次：未运行时强制滑行（保证「STOP」状态下电机一定
  * 不转，哪怕 K2/K3 改过占空比），运行时才把命令值写下去。
  * 每次只是几个寄存器写，开销可以忽略。
  */
static void app_apply_motor(void)
{
  if (s_run) {
    (void)bsp_motor_set_duty(s_duty_cmd);
  } else {
    (void)bsp_motor_coast();
  }
}

/**
  * @brief  按周期采样角度，并在数值变化超过死区时重画屏幕。
  *
  * 死区是必需的：刷一屏要约 30 ms 的 I2C 传输，而编码器位置在电机
  * 转动时每个循环都在变。不设死区就会以主循环的速度（远高于 30 ms
  * 一次）不停重画，屏幕反而永远刷不完。
  */
static void app_refresh_display(void)
{
  static uint32_t last_sample_ms = 0U;

  const uint32_t now_ms = HAL_GetTick();
  if ((now_ms - last_sample_ms) < ANGLE_SAMPLE_MS) {
    return;                             /* 还没到下一次采样时刻 */
  }
  last_sample_ms = now_ms;

  if (bsp_angle_sample() != ERR_OK) {
    return;                             /* 本次读取失败，保留上次的值 */
  }

  const uint16_t angle   = bsp_angle_raw();
  const int32_t  total   = bsp_encoder_total();
  const int32_t  angle_d = (int32_t)angle - (int32_t)s_shown_angle;
  const int32_t  total_d = total - s_shown_total;

  const bool dirty = (angle_d >= (int32_t)ANGLE_DISPLAY_DEADBAND) ||
                     (angle_d <= -(int32_t)ANGLE_DISPLAY_DEADBAND) ||
                     (total_d >= (int32_t)ENCODER_DISPLAY_DEADBAND) ||
                     (total_d <= -(int32_t)ENCODER_DISPLAY_DEADBAND) ||
                     (s_duty_cmd != s_shown_duty) ||
                     (s_run != s_shown_run);

  if (!dirty) {
    return;
  }

  s_shown_angle = angle;
  s_shown_total = total;
  s_shown_duty  = s_duty_cmd;
  s_shown_run   = s_run;

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
   * 把编码器游标累加挂到 1 ms 节拍上。
   * 这是本任务唯一的节拍使用者——角度采样留在主循环（理由见
   * app_1ms_task 的说明）。节拍还剩 2 个槽位留给以后的 PID 控制环。
   */
  if (bsp_tick_register(app_1ms_task) != ERR_OK) {
    app_fatal_blink(9U);
  }

  /*
   * 先采一次，让屏幕第一次画出来就有真实读数而不是全 0。
   * 位置也一并刷新，作为显示死区的比较基准。
   */
  (void)bsp_angle_sample();
  (void)bsp_encoder_update();
  s_shown_angle = bsp_angle_raw();
  s_shown_total = bsp_encoder_total();
  s_shown_duty  = 0;
  s_shown_run   = false;

  /* 画一次初始界面。之后只在读数变化超过死区时重画。 */
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
     * 前台主循环：处理按键、把状态落到电机、按周期采样角度并刷屏、翻 LED。
     *
     * 注意这里**没有 HAL_Delay**。在循环里死等会让所有周期性动作
     * 互相拖累（比如屏幕每 30 ms 刷一次，若用延时来定时，
     * 角度的采样周期就变成了「30 ms + 延时」）。
     * 所以一律用「查时间、时间到了才做」的非阻塞写法。
     *
     * 每次循环只做一次 HAL_GetTick() 比较，几乎不耗时；
     * 真正干活（采样、刷屏）都由各自的时间条件把关。
     */
    app_handle_keys();
    app_apply_motor();
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
  htim2.Init.Prescaler = 35;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
