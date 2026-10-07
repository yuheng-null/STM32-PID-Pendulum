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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 屏幕上各元素的位置（单位：像素）。集中在这里，改版式不用翻代码。 */
#define OLED_LINE_TITLE_Y       (0U)    /* 标题，Font_7x10，占 0~9   */
#define OLED_LINE_SCAN_Y        (11U)   /* 扫描结果，Font_6x8，占 11~18 */
#define OLED_LINE_RULE_Y        (20U)   /* 分隔线 */
#define OLED_LINE_MAIN_Y        (23U)   /* 主信息，Font_11x18，占 23~40 */
#define OLED_LINE_PIN_Y         (43U)   /* 接线说明，Font_6x8，占 43~50 */
#define OLED_LINE_COUNT_Y       (53U)   /* 循环计数，Font_6x8，占 53~60 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* OLED 设备实例（含 1 KB 显存），文件级静态，不占栈 */
static ssd1306_t s_oled;

/* 主循环节拍计数，显示在屏幕右下角 */
static uint32_t loop_count = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t oled_u32_to_dec(uint32_t value, char *out);
static void    oled_draw_static_screen(void);
static void    oled_update_counter(uint32_t count);
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
  * @brief  绘制不随时间变化的那些内容，只在开机时画一次。
  */
static void oled_draw_static_screen(void)
{
  uint8_t       found[BSP_OLED_SCAN_MAX];
  const uint8_t count = bsp_oled_bus_scan(found, BSP_OLED_SCAN_MAX);
  char          msg[24];

  ssd1306_fill(&s_oled, SSD1306_BLACK);

  (void)ssd1306_set_cursor(&s_oled, 0U, OLED_LINE_TITLE_Y);
  (void)ssd1306_write_string(&s_oled, "PID Pendulum", &Font_7x10, SSD1306_WHITE);

  /*
   * 打印总线扫描结果。这一段的作用是自检：
   * 能扫到 0x3C 就说明 PB8/PB9 接线、上拉、时序全部正确。
   */
  (void)ssd1306_set_cursor(&s_oled, 0U, OLED_LINE_SCAN_Y);
  if ((count == 1U) && (found[0] == BSP_OLED_I2C_ADDR)) {
    (void)ssd1306_write_string(&s_oled, "scan: 0x3C ok", &Font_6x8, SSD1306_WHITE);
  } else {
    (void)ssd1306_write_string(&s_oled, "scan: ", &Font_6x8, SSD1306_WHITE);
    (void)oled_u32_to_dec(count, msg);
    (void)ssd1306_write_string(&s_oled, msg, &Font_6x8, SSD1306_WHITE);
    (void)ssd1306_write_string(&s_oled, " dev!", &Font_6x8, SSD1306_WHITE);
  }

  /* 分隔线 */
  ssd1306_draw_line(&s_oled, 0U, OLED_LINE_RULE_Y, SSD1306_WIDTH - 1U, OLED_LINE_RULE_Y,
                    SSD1306_WHITE);

  (void)ssd1306_set_cursor(&s_oled, 0U, OLED_LINE_MAIN_Y);
  (void)ssd1306_write_string(&s_oled, "OLED OK", &Font_11x18, SSD1306_WHITE);

  (void)ssd1306_set_cursor(&s_oled, 0U, OLED_LINE_PIN_Y);
  (void)ssd1306_write_string(&s_oled, "PB8=SCL PB9=SDA", &Font_6x8, SSD1306_WHITE);

  (void)ssd1306_update_screen(&s_oled);
}

/**
  * @brief  刷新右下角那个循环计数。
  * @note   先把整行清掉再写，否则数字位数变少时会留下上一帧的残影。
  */
static void oled_update_counter(uint32_t count)
{
  char msg[12];

  ssd1306_fill_rectangle(&s_oled, 0U, OLED_LINE_COUNT_Y,
                         SSD1306_WIDTH - 1U,
                         (uint16_t)(OLED_LINE_COUNT_Y + 7U),
                         SSD1306_BLACK);

  (void)ssd1306_set_cursor(&s_oled, 0U, OLED_LINE_COUNT_Y);
  (void)ssd1306_write_string(&s_oled, "loop: ", &Font_6x8, SSD1306_WHITE);
  (void)oled_u32_to_dec(count, msg);
  (void)ssd1306_write_string(&s_oled, msg, &Font_6x8, SSD1306_WHITE);

  (void)ssd1306_update_screen(&s_oled);
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
  /* USER CODE BEGIN 2 */
  /*
   * OLED 挂在 PB8(SCL)/PB9(SDA)，走软件模拟 I2C，由 bsp_oled 自行接管引脚。
   * 若屏幕没有应答，用 LED 快闪（10 Hz）作为错误指示——此时屏幕上不会有任何东西，
   * 否则「黑屏」既有可能是接线问题，也有可能是代码问题，无法区分。
   */
  if (bsp_oled_init(&s_oled) != ERR_OK) {
    while (1) {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(50);
    }
  }

  oled_draw_static_screen();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*
     * 每 500 ms 一个节拍：
     *   · PC13 LED 翻转一次（1 Hz 方波，肉眼看到的是「亮 0.5s / 灭 0.5s」）
     *   · OLED 右下角的计数 +1 并重新上屏
     *
     * 计数与 LED 同节奏，因此 LED 闪一下、屏幕上的数字就该跟着跳一下；
     * 两者不一致就说明主循环被某处阻塞了。
     */
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    loop_count++;
    oled_update_counter(loop_count);
    HAL_Delay(500);
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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
