/**
 * @file    bsp_oled.c
 * @brief   板载 OLED 的板级支持实现。
 */

#include "bsp_oled.h"

#include "i2c_soft.h"

/*
 * ── 本板的 OLED 接线 ────────────────────────────────────────────────
 *   OLED SCL ── PB8
 *   OLED SDA ── PB9
 *   OLED VCC ── 3V3
 *   OLED GND ── GND
 *
 * 为什么这两个引脚由软件模拟 I2C 直接接管、而不是先在 CubeMX 里配成
 * GPIO_Output：
 *   PB8/PB9 并不是 STM32F103 的默认 I2C 引脚，CubeMX 里也没有「软件 I2C」
 *   这种外设可以选。位翻转总线的引脚时序属于总线实现的一部分，
 *   由总线驱动自己配置更内聚，也避免 CubeMX 重新生成代码时把模式改回推挽。
 * ────────────────────────────────────────────────────────────────────
 */
#define BSP_OLED_SCL_PORT       (GPIOB)
#define BSP_OLED_SCL_PIN        (GPIO_PIN_8)
#define BSP_OLED_SDA_PORT       (GPIOB)
#define BSP_OLED_SDA_PIN        (GPIO_PIN_9)

/** 本板唯一的一条 OLED 总线。板级资源天然只有一份，故用文件级静态。 */
static i2c_soft_t s_oled_bus;

/** 总线是否已建立，供总线扫描判断前置条件 */
static bool s_oled_ready = false;

void bsp_oled_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

error_t bsp_oled_init(ssd1306_t *dev)
{
    if (dev == NULL) {
        return ERR_INVALID_PARAM;
    }

    /* CubeMX 当前只使能了 GPIOA/C/D 的时钟，PB8/PB9 需要自己开 GPIOB */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    const i2c_soft_cfg_t bus_cfg = {
        .scl_port = BSP_OLED_SCL_PORT,
        .scl_pin  = BSP_OLED_SCL_PIN,
        .sda_port = BSP_OLED_SDA_PORT,
        .sda_pin  = BSP_OLED_SDA_PIN,
        .freq_hz  = I2C_SOFT_DEFAULT_FREQ_HZ,
    };

    error_t err = i2c_soft_init(&s_oled_bus, &bus_cfg);
    if (err != ERR_OK) {
        return err;
    }

    dev->bus      = i2c_soft_iface(&s_oled_bus);
    dev->addr     = BSP_OLED_I2C_ADDR;
    dev->delay_ms = bsp_oled_delay_ms;

    if (dev->bus == NULL) {
        return ERR_NOT_INITIALIZED;
    }

    s_oled_ready = true;

    return ssd1306_init(dev);
}

uint8_t bsp_oled_bus_scan(uint8_t *found, uint8_t max)
{
    uint8_t count = 0U;

    if ((found == NULL) || (max == 0U) || (!s_oled_ready)) {
        return 0U;
    }

    for (uint8_t addr = BSP_OLED_SCAN_ADDR_FIRST; addr <= BSP_OLED_SCAN_ADDR_LAST; addr++) {
        uint8_t dummy = 0U;

        /*
         * 探测手段：向该地址发起一次单字节读。
         * 器件存在就会在自己的地址帧后回 ACK，总线层因此返回 ERR_OK；
         * 不存在则无人应答，返回 ERR_COMMUNICATION。
         * 这里读到的内容丢掉不用，只需要「有没有人应答」这一位信息。
         */
        if (s_oled_bus.iface.read(s_oled_bus.iface.ctx, addr, &dummy, 1U) == ERR_OK) {
            if (count < max) {
                found[count] = addr;
            }
            count++;
        }
    }

    return (count < max) ? count : max;
}
