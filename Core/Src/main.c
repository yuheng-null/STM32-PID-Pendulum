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
 * 4 路电位器各占一行，行距 12 像素：文字高 8、进度条高 7，都不重叠。
 *   第 0 行 10~17、第 1 行 22~29、第 2 行 34~41、第 3 行 46~53
 * 底部 55~62 留给说明行。
 */
#define OLED_POT_LINE_Y0        (10U)
#define OLED_POT_LINE_STEP      (12U)
#define OLED_POT_FOOTER_Y       (55U)

/* 每行左侧 "RP1 2048" 共 8 字符 = 48 像素 */
#define OLED_POT_LABEL_X        (0U)

/* 进度条：从 x=52 画到 x=125，宽 74 像素，高 7 像素 */
#define OLED_POT_BAR_X0         (52U)
#define OLED_POT_BAR_X1         (125U)
#define OLED_POT_BAR_H          (7U)

/* PC13 LED 心跳周期（毫秒） */
#define LED_HEARTBEAT_MS        (500U)

/* 电位器采样周期（毫秒）。人手拧旋钮，20 ms（50 Hz）足够跟手。 */
#define POT_SAMPLE_MS           (20U)

/*
 * 显示死区（LSB）。只有某一路变化超过这个值才重画屏幕。
 *
 * 为什么需要：ADC 末位总在抖（±1~2 LSB），若「一变就重画」，旋钮静止时
 * 屏幕也会以 50 Hz 不停重画整屏（每次约 30 ms 的 I2C 传输），既浪费又
 * 让末位数字闪个不停。取 8 LSB（满量程的 0.2%）既滤掉噪声，
 * 又远小于人能分辨的转动幅度。
 */
#define POT_DISPLAY_DEADBAND    (8U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* OLED 设备实例（含 1 KB 显存），文件级静态，不占栈 */
static ssd1306_t s_oled;

/*
 * 上一次真正画到屏幕上的值。
 * 用它和最新采样值比较，决定要不要重画——见 POT_DISPLAY_DEADBAND 的说明。
 */
static uint16_t s_shown_raw[POT_ID_COUNT];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t oled_u32_to_dec(uint32_t value, char *out);
static void    oled_draw_pot_line(uint16_t y, pot_id_t id);
static void    oled_redraw(void);
static void    app_drain_key_events(void);
static void    app_refresh_pots(void);
static void    app_serial_service(void);
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
  * @brief  在指定行画一路电位器：左侧 "RP1 2048"，右侧一条进度条。
  * @param  y  该行顶部像素坐标
  * @param  id 电位器编号
  */
static void oled_draw_pot_line(uint16_t y, pot_id_t id)
{
  char           digits[6];
  char           msg[10];
  const uint16_t raw = bsp_pot_raw(id);
  const uint8_t  nd  = oled_u32_to_dec(raw, digits);   /* 1~4 位 */
  uint8_t        k   = 0U;

  msg[k] = 'R';
  k++;
  msg[k] = 'P';
  k++;
  msg[k] = (char)('1' + (char)id);
  k++;
  msg[k] = ' ';
  k++;

  /*
   * 数值右对齐到 4 位，位数不足时前面补空格。
   * 不补的话 4095 掉到 512 时后面几位会跟着左移，看起来像在「抖」。
   */
  for (uint8_t i = 0U; i < (uint8_t)(4U - nd); i++) {
    msg[k] = ' ';
    k++;
  }
  for (uint8_t i = 0U; i < nd; i++) {
    msg[k] = digits[i];
    k++;
  }
  msg[k] = '\0';

  (void)ssd1306_set_cursor(&s_oled, OLED_POT_LABEL_X, y);
  (void)ssd1306_write_string(&s_oled, msg, &Font_6x8, SSD1306_WHITE);

  /* 进度条外框：空载时也能看出量程有多宽，比只有填充更直观 */
  ssd1306_draw_rectangle(&s_oled, OLED_POT_BAR_X0, y, OLED_POT_BAR_X1,
                         (uint16_t)(y + OLED_POT_BAR_H - 1U), SSD1306_WHITE);

  /* 框内可填充的宽度：外框左右各占 1 像素 */
  const uint16_t inner  = (uint16_t)(OLED_POT_BAR_X1 - OLED_POT_BAR_X0 - 1U);
  uint32_t       filled = ((uint32_t)raw * inner) / BSP_POT_RAW_MAX;

  if (filled > (uint32_t)inner) {
    filled = inner;                 /* 读数超出满量程时夹住，防止越界 */
  }

  if (filled > 0U) {
    ssd1306_fill_rectangle(&s_oled, (uint16_t)(OLED_POT_BAR_X0 + 1U), (uint16_t)(y + 1U),
                           (uint16_t)(OLED_POT_BAR_X0 + filled),
                           (uint16_t)(y + OLED_POT_BAR_H - 2U), SSD1306_WHITE);
  }
}

/**
  * @brief  整屏重画并按内容重新上屏。
  *
  * 整屏重画而不是只改变化的那一块：这个规模下更简单，也绝不会留残影
  * （比如数值从 1000 掉到 999 时旧的那一位干不掉）。代价是每次约 30 ms
  * 的 I2C 传输，所以**调用前必须先用死区判断值是否真的变了**
  * （见 POT_DISPLAY_DEADBAND），否则旋钮静止时屏幕也在以 50 Hz 空刷。
  */
static void oled_redraw(void)
{
  ssd1306_fill(&s_oled, SSD1306_BLACK);

  /* 标题 */
  (void)ssd1306_set_cursor(&s_oled, 0U, OLED_LINE_TITLE_Y);
  (void)ssd1306_write_string(&s_oled, "Pot Test", &Font_7x10, SSD1306_WHITE);

  /* 4 路电位器 */
  for (uint8_t i = 0U; i < (uint8_t)POT_ID_COUNT; i++) {
    oled_draw_pot_line((uint16_t)(OLED_POT_LINE_Y0 + (i * OLED_POT_LINE_STEP)), (pot_id_t)i);
  }

  /* 说明行。一行 128 像素 / 每字符 6 像素 = 最多 21 字符，超了会被截断 */
  (void)ssd1306_set_cursor(&s_oled, 0U, OLED_POT_FOOTER_Y);
  (void)ssd1306_write_string(&s_oled, "0..4095  Vref 3.3V", &Font_6x8, SSD1306_WHITE);

  (void)ssd1306_update_screen(&s_oled);
}

/**
  * @brief  把 4 个键的事件取干净。
  *
  * 本版屏幕整屏给了电位器，按键暂时没有显示位置，但模块本身照常工作
  * （bsp_key_init() 仍然调用，1 ms 采样与消抖都在跑）。
  * 这里只是把事件取走，免得 pending 标志一直堆着；
  * 以后要做「按键翻页」时，把事件接到页切换上即可。
  */
static void app_drain_key_events(void)
{
  for (uint8_t i = 0U; i < (uint8_t)KEY_ID_COUNT; i++) {
    while (bsp_key_take_event((key_id_t)i) != KEY_EVENT_NONE) {
      /* 本版不处理，取走即可 */
    }
  }
}

/**
  * @brief  按固定周期采样电位器；值变化超过死区才重画屏幕。
  */
static void app_refresh_pots(void)
{
  static uint32_t last_sample_ms = 0U;

  const uint32_t now_ms = HAL_GetTick();
  if ((now_ms - last_sample_ms) < POT_SAMPLE_MS) {
    return;
  }
  last_sample_ms = now_ms;

  if (bsp_pot_sample() != ERR_OK) {
    return;
  }

  /* 死区判断：任何一路变化超过阈值就重画 */
  bool dirty = false;
  for (uint8_t i = 0U; i < (uint8_t)POT_ID_COUNT; i++) {
    const uint16_t v = bsp_pot_raw((pot_id_t)i);
    const uint16_t d = (v > s_shown_raw[i]) ? (uint16_t)(v - s_shown_raw[i])
                                            : (uint16_t)(s_shown_raw[i] - v);

    if (d >= POT_DISPLAY_DEADBAND) {
      dirty = true;
      break;
    }
  }

  if (!dirty) {
    return;
  }

  for (uint8_t i = 0U; i < (uint8_t)POT_ID_COUNT; i++) {
    s_shown_raw[i] = bsp_pot_raw((pot_id_t)i);
  }

  oled_redraw();
}

/**
  * @brief  串口自检服务：回显收到的字节，并按周期发一行状态。
  *
  * ⚠️ 这是**临时验证代码**，用来证明串口双向都通，验证完可以整块删掉。
  *
  * 它同时验证三件事：
  *   1. 发送链路 —— 每 500 ms 发一行，PC 端能读到就说明 TX 通；
  *   2. 接收链路 —— 收到的字节原样回显，PC 端发什么就收回什么；
  *   3. printf 重定向 —— 那一行就是用 printf 拼的，能出来说明 _write 接对了。
  *
  * 回显这段顺带说明了一个正确的读法：**有多少取多少**，
  * 不假设「一条命令一次到齐」——串口是按字节流来的，
  * 一次 bsp_serial_read() 可能只拿到半个报文，也可能拿到两条半。
  */
static void app_serial_service(void)
{
  static uint32_t last_tx_ms = 0U;
  static uint32_t counter    = 0U;

  /* 收到的字节原样回显，证明接收链路通 */
  uint8_t        echo[32];
  const uint32_t n = bsp_serial_read(echo, (uint32_t)sizeof(echo));
  if (n > 0U) {
    (void)bsp_serial_write(echo, n);
  }

  /* 每 500 ms 发一行：计数器 + 4 路电位器原始值 */
  const uint32_t now_ms = HAL_GetTick();
  if ((now_ms - last_tx_ms) >= 500U) {
    last_tx_ms = now_ms;
    counter++;
    printf("PID_Pendulum tick=%u rp1=%u rp2=%u rp3=%u rp4=%u\r\n",
           (unsigned int)counter,
           (unsigned int)bsp_pot_raw(POT_ID_RP1),
           (unsigned int)bsp_pot_raw(POT_ID_RP2),
           (unsigned int)bsp_pot_raw(POT_ID_RP3),
           (unsigned int)bsp_pot_raw(POT_ID_RP4));
  }
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

  /* 先采一次，让屏幕第一次画出来就有真实读数而不是全 0 */
  (void)bsp_pot_sample();
  for (uint8_t i = 0U; i < (uint8_t)POT_ID_COUNT; i++) {
    s_shown_raw[i] = bsp_pot_raw((pot_id_t)i);
  }

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
     * 前台主循环做三件事：取走按键事件、按周期采电位器、翻 LED。
     *
     * 注意这里**没有 HAL_Delay**。在循环里死等会让所有周期性动作
     * 互相拖累（比如屏幕每 30 ms 刷一次，若用延时来定时，
     * 电位器的采样周期就变成了「30 ms + 延时」，旋钮跟手感变差）。
     * 所以一律用「查时间、时间到了才做」的非阻塞写法。
     *
     * 每次循环只做一次 HAL_GetTick() 比较，几乎不耗时；
     * 真正干活（采样、刷屏）都由各自的时间条件把关。
     */
    app_drain_key_events();
    app_refresh_pots();
    app_serial_service();

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
  HAL_GPIO_WritePin(GPIOB, OLED_SCL_Pin|OLED_SDA_Pin, GPIO_PIN_RESET);

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
