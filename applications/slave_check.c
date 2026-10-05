/*
 * 便携工业调试仪 —— 星火一号（从机）自检程序
 *
 * 提供三个 msh 命令，对应执行计划第 1、3、7 步的验收：
 *   link_check    内部链路 uart3 发一帧，用 USB-TTL 在 PB10/PB11 上量
 *   sensor_check  读三个板载传感器（AHT10 / AP3216C / ICM20608）
 *   rs485_start   初始化 RS485（drv_rs485.c 没有自动初始化），之后用 rs485_test -t xxx 发送
 *
 * 注意：rt_kprintf 不支持 %f，这里全部用整数打印。
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#ifdef PKG_USING_AHT10
#include "aht10.h"
#endif
#ifdef PKG_USING_AP3216C
#include "ap3216c.h"
#endif
#ifdef PKG_USING_ICM20608
#include "icm20608.h"
#endif
#ifdef BSP_USING_RS485
#include "drv_rs485.h"
#endif

#define LINK_UART_NAME   "uart3"     /* PB10(TX) / PB11(RX)，与 D12x 的内部链路 */

/* ---------------------------------------------------------------- 第 1 步 */
static int link_check(int argc, char **argv)
{
    rt_device_t dev = rt_device_find(LINK_UART_NAME);
    const char *msg = "SPARK_OK\r\n";

    if (dev == RT_NULL)
    {
        rt_kprintf("[link] find %s failed! 检查 BSP_USING_UART3 是否打开\n", LINK_UART_NAME);
        return -RT_ERROR;
    }

    if (rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        rt_kprintf("[link] open %s failed\n", LINK_UART_NAME);
        return -RT_ERROR;
    }

    rt_device_write(dev, 0, msg, rt_strlen(msg));
    rt_kprintf("[link] sent \"%s\" on %s (PB10/PB11)\n", "SPARK_OK", LINK_UART_NAME);
    return RT_EOK;
}
MSH_CMD_EXPORT(link_check, send a test frame on uart3);

/* ---------------------------------------------------------------- 第 3 步 */
static void sensor_check(void)
{
#ifdef PKG_USING_AHT10
    aht10_device_t aht10 = aht10_init("i2c3");      /* PE0/PE1 */
    if (aht10)
    {
        int t = (int)(aht10_read_temperature(aht10) * 10);
        int h = (int)(aht10_read_humidity(aht10) * 10);
        rt_kprintf("[AHT10 ] temp=%d.%d C   humi=%d.%d %%\n",
                   t / 10, t < 0 ? (-t) % 10 : t % 10,
                   h / 10, h < 0 ? (-h) % 10 : h % 10);
    }
    else
    {
        rt_kprintf("[AHT10 ] init failed on i2c3 (PE0/PE1)\n");
    }
#endif

#ifdef PKG_USING_AP3216C
    ap3216c_device_t ap = ap3216c_init("i2c2");     /* PF1/PF0 */
    if (ap)
    {
        int lux = (int)(ap3216c_read_ambient_light(ap) * 10);
        rt_kprintf("[AP3216] light=%d.%d lux   ps=%d\n",
                   lux / 10, lux % 10, ap3216c_read_ps_data(ap));
    }
    else
    {
        rt_kprintf("[AP3216] init failed on i2c2 (PF1/PF0)\n");
    }
#endif

#ifdef PKG_USING_ICM20608
    icm20608_device_t icm = icm20608_init("i2c2");  /* PF1/PF0 */
    if (icm)
    {
        rt_int16_t ax, ay, az, gx, gy, gz;
        if (icm20608_get_accel(icm, &ax, &ay, &az) == RT_EOK &&
            icm20608_get_gyro(icm, &gx, &gy, &gz) == RT_EOK)
        {
            rt_kprintf("[ICM206] accel=%d,%d,%d   gyro=%d,%d,%d\n", ax, ay, az, gx, gy, gz);
        }
        else
        {
            rt_kprintf("[ICM206] read failed\n");
        }
    }
    else
    {
        rt_kprintf("[ICM206] init failed on i2c2 (PF1/PF0)\n");
    }
#endif
}
MSH_CMD_EXPORT(sensor_check, read all onboard sensors);

/* ---------------------------------------------------------------- 第 7 步 */
#ifdef BSP_USING_RS485
static int rs485_start(void)
{
    if (rs485_init() == RT_EOK)
    {
        rt_kprintf("[rs485] ready: %s (PG14/PG9, DE=PG8)\n", RS485_UART_DEVICE_NAME);
        rt_kprintf("[rs485] send: rs485_test -t <string>\n");
        return RT_EOK;
    }
    rt_kprintf("[rs485] init failed\n");
    return -RT_ERROR;
}
MSH_CMD_EXPORT(rs485_start, init rs485 (uart6));
#endif
