/*
 * RW007 WiFi 模组 SPI 移植 —— 星火一号（RT-Spark）SPI2
 *
 * 引脚取自 RT-Thread 官方 BSP stm32f407-rt-spark 的 board/Kconfig：
 *   config BSP_USING_RW007_WLAN  "Enable Rw007 Wlan Base on SPI2"
 *     RW007_CS_PIN       = 90    -> PF10
 *     RW007_BOOT0_PIN    = 29    -> PB13（与 SPI2_SCK 同脚）
 *     RW007_BOOT1_PIN    = 90    -> PF10（与 CS 同脚）
 *     RW007_INT_BUSY_PIN = 107   -> PG11
 *     RW007_RST_PIN      = 111   -> PG15
 * 判断依据（GET_PIN 编码 = port*16 + pin，port A=0 … G=6）：
 *   90 = 5*16+10 = PF10，29 = 1*16+13 = PB13，107 = 6*16+11 = PG11，111 = 6*16+15 = PG15。
 * 与本项目已用引脚无冲突：SPI2 三根线 PB13/PC2/PC3 本来就在用（W25Q64 走 CS=PB12，
 * RW007 另占 PF10，两个设备挂同一总线不同片选，互不影响）。
 *
 * ⚠️ 本文件整段受 BSP_USING_RW007_WLAN 守卫：没在 RT-Thread Settings 里勾 RW007 时
 *    它根本不参与编译（applications/SConscript 是 Glob('*.c')，不加守卫会编不过）。
 */

#include <rtthread.h>

#ifdef BSP_USING_RW007_WLAN

#include <rtdevice.h>
#include <drv_spi.h>
#include <board.h>
#include <spi_wifi_rw007.h>

#define DBG_TAG "rw007"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* Kconfig 未给出引脚配置时（本工程 libraries/Kconfig 还没补那几项），用官方默认值兜底 */
#ifndef RW007_SPI_BUS_NAME
#define RW007_SPI_BUS_NAME    "spi2"
#endif
#ifndef RW007_CS_PIN
#define RW007_CS_PIN          GET_PIN(F, 10)
#endif
#ifndef RW007_BOOT0_PIN
#define RW007_BOOT0_PIN       GET_PIN(B, 13)   /* = SPI2_SCK */
#endif
#ifndef RW007_BOOT1_PIN
#define RW007_BOOT1_PIN       GET_PIN(F, 10)   /* = CS */
#endif
#ifndef RW007_INT_BUSY_PIN
#define RW007_INT_BUSY_PIN    GET_PIN(G, 11)
#endif
#ifndef RW007_RST_PIN
#define RW007_RST_PIN         GET_PIN(G, 15)
#endif

/* rw007 包内部实现，模组有数据要送时靠它进中断下半部 */
extern void spi_wifi_isr(int vector);

/*
 * 上电时序（与官方 rw007_stm32_port.c 完全一致，rt-spark 已知可用）：
 *   拉低 RST 100ms -> 拉高 -> 轮询 INT/BUSY 变高（模组退出 busy）-> 再等 200ms
 *   -> INT 改上拉输入 -> 之后才能挂中断。
 * 注意：RW007 的启动模式在 rt-spark 上由模组/板载电阻硬件strapping固定为
 * SPI RAW，官方代码从不操作 BOOT 脚（Kconfig 里的 BOOT0=PB13/BOOT1=PF10 实际
 * 就是 SPI 的 SCK/CS，并非模组的 BOOT 脚）。所以这里只做 RST + 等 BUSY。
 * 若 BUSY 始终不拉高，说明模组根本没起来（没电/没焊好/BOOT strap 不对），属硬件层。
 * 这里用 3s 超时保护，避免模组没就绪时系统死等卡死。
 */
static void rw007_gpio_init(void)
{
    rt_pin_mode(RW007_RST_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(RW007_INT_BUSY_PIN, PIN_MODE_INPUT_PULLDOWN);

    rt_pin_write(RW007_RST_PIN, PIN_LOW);
    rt_thread_mdelay(100);
    rt_pin_write(RW007_RST_PIN, PIN_HIGH);

    /* 等模组退出 busy 状态（加超时保护，约 3s，避免模组没就绪时系统死等卡死） */
    int busy_wait = 0;
    while (!rt_pin_read(RW007_INT_BUSY_PIN))
    {
        rt_thread_delay(5);
        if (++busy_wait > 600)
            break;
    }

    rt_thread_mdelay(200);
    rt_pin_mode(RW007_INT_BUSY_PIN, PIN_MODE_INPUT_PULLUP);
}

int wifi_spi_device_init(void)
{
    char sn_version[32] = {0};

    rw007_gpio_init();

#if (RT_VER_NUM >= 0x50000)
    /* 5.0 起 attach 直接收 pin index */
    rt_hw_spi_device_attach(RW007_SPI_BUS_NAME, "wspi", RW007_CS_PIN);
#else
    /* 4.x 传的是 (GPIO_TypeDef*, pin mask)，用 index 反推端口基址 */
    GPIO_TypeDef *cs_gpiox;
    uint16_t      cs_pin;

    cs_gpiox = (GPIO_TypeDef *)((rt_base_t)GPIOA + (rt_base_t)(RW007_CS_PIN / 16) * 0x0400UL);
    cs_pin   = (uint16_t)(1 << (RW007_CS_PIN % 16));
    rt_hw_spi_device_attach(RW007_SPI_BUS_NAME, "wspi", cs_gpiox, cs_pin);
#endif

    rt_hw_wifi_init("wspi");

    rt_wlan_set_mode(RT_WLAN_DEVICE_STA_NAME, RT_WLAN_STATION);
    rt_wlan_set_mode(RT_WLAN_DEVICE_AP_NAME, RT_WLAN_AP);

    rw007_sn_get(sn_version);
    rt_kprintf("\nrw007  sn: [%s]\n", sn_version);
    rw007_version_get(sn_version);
    rt_kprintf("rw007 ver: [%s]\n\n", sn_version);

    return 0;
}
INIT_APP_EXPORT(wifi_spi_device_init);

static void int_wifi_irq(void *p)
{
    ((void)p);
    spi_wifi_isr(0);
}

/* rw007 包会回调这个函数来挂中断（BSP 必须提供） */
void spi_wifi_hw_init(void)
{
    rt_pin_attach_irq(RW007_INT_BUSY_PIN, PIN_IRQ_MODE_FALLING, int_wifi_irq, RT_NULL);
    rt_pin_irq_enable(RW007_INT_BUSY_PIN, RT_TRUE);
}

#endif /* BSP_USING_RW007_WLAN */
