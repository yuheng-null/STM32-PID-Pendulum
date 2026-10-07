/**
 * @file    bsp_oled.c
 * @brief   板载 OLED 的板级支持实现。
 */

#include "bsp_oled.h"

#include "i2c_soft.h"
#include "main.h"                   /* CubeMX 生成的引脚宏 OLED_SCL_Pin 等 */

/*
 * ── 本板的 OLED 接线 ────────────────────────────────────────────────
 *   OLED SCL ── PB8
 *   OLED SDA ── PB9
 *   OLED VCC ── 3V3
 *   OLED GND ── GND
 *
 * 引脚宏 OLED_SCL_Pin / OLED_SCL_GPIO_Port 由 CubeMX 从 .ioc 生成（见 main.h），
 * 配置为「开漏输出 + 上拉 + 高速」，由 MX_GPIO_Init() 完成。
 *
 * 关于「谁配置这两个引脚」：
 *   CubeMX 会配一遍，i2c_soft_init() 里也会再配一遍（同样的开漏/上拉/高速）。
 *   看起来重复，但这是有意的：总线驱动应当自包含——它可能被用在不归 CubeMX
 *   管的引脚上。两处的电气参数必须保持一致，改一处就要改另一处。
 * ────────────────────────────────────────────────────────────────────
 */

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

    /* GPIOB 的时钟由 CubeMX 生成的 MX_GPIO_Init() 使能（PB8/PB9 已在 .ioc 里） */

    const i2c_soft_cfg_t bus_cfg = {
        .scl_port = OLED_SCL_GPIO_Port,
        .scl_pin  = OLED_SCL_Pin,
        .sda_port = OLED_SDA_GPIO_Port,
        .sda_pin  = OLED_SDA_Pin,
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
