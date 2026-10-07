/**
 * @file    bsp_oled.h
 * @brief   板载 0.96 寸 OLED 的板级支持包。
 *
 * 本层的职责是「把这块板子上的这块屏接起来」：
 *   · 指定 SCL/SDA 具体接在哪两个引脚上
 *   · 提供该板子特有的延时实现
 *   · 把软件 I2C 总线和 SSD1306 设备驱动组装成一个可用的对象
 *
 * 应用层只跟本文件打交道，不需要知道底下是软件 I2C 还是硬件 I2C。
 */

#ifndef BSP_OLED_H
#define BSP_OLED_H

#include <stdint.h>

#include "error.h"
#include "ssd1306.h"

/** 本板 OLED 模块的 I2C 7 位地址（模块背面 SA0 接地时为 0x3C） */
#define BSP_OLED_I2C_ADDR       (SSD1306_I2C_ADDR_7BIT)

/** 一次总线扫描最多能返回的器件数（7 位地址空间共 128 个） */
#define BSP_OLED_SCAN_MAX       (16U)

/*
 * 扫描范围。I2C 规范里 0x00~0x07 与 0x78~0x7F 是保留地址
 * （后者还包含 10 位地址前缀和广播地址），扫它们没有意义。
 */
#define BSP_OLED_SCAN_ADDR_FIRST    (0x08U)
#define BSP_OLED_SCAN_ADDR_LAST     (0x77U)

/**
 * @brief 初始化板载 OLED。
 *
 * 依次完成：使能 GPIO 时钟 → 建立软件 I2C 总线 → 初始化 SSD1306。
 *
 * @param dev 设备实例，由调用者提供存储（通常用文件级静态变量）
 *
 * @return error_t ERR_OK 成功；ERR_INVALID_PARAM 入参为空；
 *                 ERR_COMMUNICATION 屏幕无应答（检查接线与供电）
 *
 * @note 本函数内部有约 100 ms 的阻塞等待，请在调度器起来之前调用。
 */
error_t bsp_oled_init(ssd1306_t *dev);

/**
 * @brief 扫描 OLED 所在 I2C 总线。
 *
 * 用于排查接线问题：正常情况下应当只扫到 0x3C 一个地址。
 *
 * @param found 存放扫到的 7 位地址
 * @param max   数组容量
 *
 * @return uint8_t 实际扫到的器件个数
 *
 * @note 必须在 bsp_oled_init() 之后调用（依赖已建立的总线）。
 */
uint8_t bsp_oled_bus_scan(uint8_t *found, uint8_t max);

/**
 * @brief 板级毫秒延时，作为 ssd1306_t::delay_ms 注入给设备驱动。
 */
void bsp_oled_delay_ms(uint32_t ms);

#endif /* BSP_OLED_H */
