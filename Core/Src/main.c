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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 屏幕上各元素的位置（单位：像素）。集中在这里，改版式不用翻代码。 */
#define OLED_LINE_TITLE_Y       (0U)    /* 标题，Font_7x10，占 0~9      */
#define OLED_LINE_STATE_Y       (12U)   /* 4 个键的实时状态，占 12~19   */
#define OLED_LINE_RULE_Y        (22U)   /* 分隔线                       */
#define OLED_LINE_CNT_12_Y      (25U)   /* K1/K2 计数，占 25~32         */
#define OLED_LINE_CNT_34_Y      (35U)   /* K3/K4 计数，占 35~42         */
#define OLED_LINE_EDGE_Y        (45U)   /* 最近按下/松开，占 45~52      */
#define OLED_LINE_INFO_Y        (55U)   /* 消抖参数说明，占 55~62       */

/* 状态块 "K1[ ]" 共 5 字符 = 30 像素，4 块间隔 32 像素，共占 0~125 */
#define OLED_KEY_STATE_X0       (0U)
#define OLED_KEY_STATE_STEP     (32U)

/* 计数块 "K1:0" 放在左右两列 */
#define OLED_KEY_CNT_X0         (0U)
#define OLED_KEY_CNT_STEP       (64U)

/* PC13 LED 心跳周期（毫秒） */
#define LED_HEARTBEAT_MS        (500U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
/* OLED 设备实例（含 1 KB 显存），文件级静态，不占栈 */
static ssd1306_t s_oled;

/*
 * 每个键的按下次数。计数是**应用层的策略**，不是驱动的职责——
 * 驱动只负责报「电平」和「边沿」，至于用什么口径统计由应用决定。
 */
static uint32_t s_press_count[KEY_ID_COUNT];

/* 最近一次产生按下 / 松开事件的键；KEY_ID_COUNT 表示「还没有过」 */
static uint8_t s_last_down = (uint8_t)KEY_ID_COUNT;
static uint8_t s_last_up   = (uint8_t)KEY_ID_COUNT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t oled_u32_to_dec(uint32_t value, char *out);
static uint8_t str_append(char *dst, uint8_t n, const char *src);
static void    oled_draw_key_state(uint16_t x, key_id_t id);
static void    oled_draw_key_count(uint16_t x, uint16_t y, key_id_t id);
static void    oled_redraw(void);
static void    app_poll_keys(void);
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
  * @brief  把字符串接到缓冲末尾，返回新的写入位置。
  */
static uint8_t str_append(char *dst, uint8_t n, const char *src)
{
  while (*src != '\0') {
    dst[n] = *src;
    n++;
    src++;
  }
  return n;
}

/**
  * @brief  在指定横向位置画一个按键状态块，形如 "K1[ ]"，按下时显示 "K1[X]"。
  * @param  id 按键编号，块里的数字和方括号状态都跟着它变
  */
static void oled_draw_key_state(uint16_t x, key_id_t id)
{
  char block[6];

  block[0] = 'K';
  block[1] = (char)('1' + (char)id);
  block[2] = '[';
  block[3] = bsp_key_is_pressed(id) ? 'X' : ' ';
  block[4] = ']';
  block[5] = '\0';

  (void)ssd1306_set_cursor(&s_oled, x, OLED_LINE_STATE_Y);
  (void)ssd1306_write_string(&s_oled, block, &Font_6x8, SSD1306_WHITE);
}

/**
  * @brief  在指定位置画一个按键计数，形如 "K1:12"。
  */
static void oled_draw_key_count(uint16_t x, uint16_t y, key_id_t id)
{
  char msg[12];

  msg[0] = 'K';
  msg[1] = (char)('1' + (char)id);
  msg[2] = ':';
  (void)oled_u32_to_dec(s_press_count[id], &msg[3]);

  (void)ssd1306_set_cursor(&s_oled, x, y);
  (void)ssd1306_write_string(&s_oled, msg, &Font_6x8, SSD1306_WHITE);
}

/**
  * @brief  整屏重画并按内容重新上屏。
  *
  * 每次按键事件都整屏重画，而不是只改变化的那一小块。
  * 在这个规模下整屏重画更简单、也绝不会留下残影（比如计数从 10 变成 9
  * 时旧的那一位数字干不掉）。代价是每次约 30 ms 的 I2C 传输，
  * 按键事件的频率远低于此，完全可以接受。
  */
static void oled_redraw(void)
{
  char    buf[24];
  uint8_t n;

  ssd1306_fill(&s_oled, SSD1306_BLACK);

  /* 标题 */
  (void)ssd1306_set_cursor(&s_oled, 0U, OLED_LINE_TITLE_Y);
  (void)ssd1306_write_string(&s_oled, "Key Test", &Font_7x10, SSD1306_WHITE);

  /* 4 个键的实时状态 */
  for (uint8_t i = 0U; i < (uint8_t)KEY_ID_COUNT; i++) {
    oled_draw_key_state((uint16_t)(OLED_KEY_STATE_X0 + (i * OLED_KEY_STATE_STEP)),
                        (key_id_t)i);
  }

  /* 分隔线 */
  ssd1306_draw_line(&s_oled, 0U, OLED_LINE_RULE_Y, SSD1306_WIDTH - 1U, OLED_LINE_RULE_Y,
                    SSD1306_WHITE);

  /* 各键按下次数：K1/K2 一行，K3/K4 一行 */
  oled_draw_key_count(OLED_KEY_CNT_X0, OLED_LINE_CNT_12_Y, KEY_ID_K1);
  oled_draw_key_count((uint16_t)(OLED_KEY_CNT_X0 + OLED_KEY_CNT_STEP), OLED_LINE_CNT_12_Y,
                      KEY_ID_K2);
  oled_draw_key_count(OLED_KEY_CNT_X0, OLED_LINE_CNT_34_Y, KEY_ID_K3);
  oled_draw_key_count((uint16_t)(OLED_KEY_CNT_X0 + OLED_KEY_CNT_STEP), OLED_LINE_CNT_34_Y,
                      KEY_ID_K4);

  /* 最近一次按下和松开的键，用来验证两种边沿都被正确上报 */
  n = str_append(buf, 0U, "down:");
  if (s_last_down < (uint8_t)KEY_ID_COUNT) {
    buf[n] = (char)('K');
    n++;
    buf[n] = (char)('1' + s_last_down);
    n++;
  } else {
    buf[n] = '-';
    n++;
  }

  n = str_append(buf, n, "  up:");
  if (s_last_up < (uint8_t)KEY_ID_COUNT) {
    buf[n] = (char)('K');
    n++;
    buf[n] = (char)('1' + s_last_up);
    n++;
  } else {
    buf[n] = '-';
    n++;
  }
  buf[n] = '\0';

  (void)ssd1306_set_cursor(&s_oled, 0U, OLED_LINE_EDGE_Y);
  (void)ssd1306_write_string(&s_oled, buf, &Font_6x8, SSD1306_WHITE);

  /* 消抖参数，静态说明。
     一行 128 像素 / 每字符 6 像素 = 最多 21 个字符，超了会被 write_string 截断 */
  (void)ssd1306_set_cursor(&s_oled, 0U, OLED_LINE_INFO_Y);
  (void)ssd1306_write_string(&s_oled, "debounce 20 ms", &Font_6x8, SSD1306_WHITE);

  (void)ssd1306_update_screen(&s_oled);
}

/**
  * @brief  轮询 4 个键的事件；有任何变化就重画屏幕。
  *
  * 事件是边沿，读取即清除。一个键可能同时攒着「按下」和「松开」两个事件
  * （比如主循环正忙着刷屏时用户完成了一次按放），所以要用 while 取干净。
  */
static void app_poll_keys(void)
{
  bool dirty = false;

  for (uint8_t i = 0U; i < (uint8_t)KEY_ID_COUNT; i++) {
    const key_id_t id = (key_id_t)i;
    key_event_t    ev;

    while ((ev = bsp_key_take_event(id)) != KEY_EVENT_NONE) {
      if (ev == KEY_EVENT_PRESS) {
        s_press_count[i]++;
        s_last_down = i;
      } else {
        s_last_up = i;
      }
      dirty = true;
    }
  }

  if (dirty) {
    oled_redraw();
  }
}

/**
  * @brief  初始化失败时的错误指示：快闪 code 次，停 1 秒，循环。
  *
  * 闪烁次数对应失败的模块，便于不用调试器就定位问题：
  *   1 次 = OLED（bsp_oled_init 失败，屏没应答）
  *   2 次 = 系统节拍（bsp_tick_init 失败）
  *   3 次 = 按键（bsp_key_init 失败）
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

  /* 画一次初始界面。之后只在按键事件发生时重画。 */
  oled_redraw();

  uint32_t last_led_ms = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*
     * 前台主循环只做两件事：处理按键事件、翻 LED。
     *
     * 注意这里**没有 HAL_Delay**。如果像上一版那样在循环里死等 500 ms，
     * 按键事件的响应会被拖到最长 500 ms——按一下要过半秒屏幕才动，
     * 看起来就像按键坏了。所以要改成「查时间、时间到了才做」的非阻塞写法。
     *
     * 按键采样本身不在这里：它挂在 1 ms 定时中断上，采样间隔恒定，
     * 与主循环此刻在忙什么（比如正在花 30 ms 刷屏）无关。
     */
    app_poll_keys();

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
  while (1) {
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
