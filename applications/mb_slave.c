/*
 * 便携工业调试仪 —— 星火一号（从机）Modbus-RTU 从站
 *
 * 跑在 USART3 (PB10/PB11) 上，供 D12x 主机轮询。
 * 自包含实现（CRC16 + FC03/FC06/FC16），不依赖任何在线软件包。
 *
 * 寄存器地图见下方的 REG_* 定义。
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

#define DBG_TAG "mbs"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* ------------------------------------------------------------ 配置 */
#define MB_DEV_NAME      "uart3"
#define MB_SLAVE_ADDR    1
#define MB_BAUD          BAUD_RATE_115200
#define MB_T35_TICKS     5          /* 帧间隔 5 ms，覆盖 9600 以上波特率 */
#define MB_FRAME_TIMEOUT 100        /* 收不齐最小帧长时的兜底丢弃超时 (ms) */
#define MB_FRAME_MAX     256

/* ------------------------------------------------------- 寄存器地图 */
#define REG_HEARTBEAT    0x0000     /* RO 每 500ms 自增，主机据此判活 */
#define REG_UPTIME       0x0001     /* RO 运行秒数 */
#define REG_ERR          0x0002     /* RO b0 AHT10 b1 AP3216C b2 ICM20608 */

#define REG_TEMP         0x0010     /* RO 0.1 °C */
#define REG_HUMI         0x0011     /* RO 0.1 % */
#define REG_LUX          0x0012     /* RO 0.1 lux */
#define REG_PS           0x0013     /* RO 接近值 */

#define REG_ACC_X        0x0020     /* RO 裸 int16，±2g，16384 LSB/g */
#define REG_ACC_Y        0x0021
#define REG_ACC_Z        0x0022
#define REG_GYRO_X       0x0023
#define REG_GYRO_Y       0x0024
#define REG_GYRO_Z       0x0025

#define REG_DO_CTRL      0x0100     /* RW b0 LED_R b1 LED_G b2 LED_B b3 BEEP */
#define REG_PWM_DUTY     0x0101     /* RW 0.1 % */

#define REG_COUNT        0x0120

/* ---------------------------------------------------------- 内部状态 */
static rt_device_t        g_dev       = RT_NULL;
static struct rt_semaphore g_rx_sem;
static rt_uint16_t        g_hold[REG_COUNT];
static rt_uint8_t         g_addr      = MB_SLAVE_ADDR;

/* -------------------------------------------------------------- CRC16 */
static rt_uint16_t mb_crc16(const rt_uint8_t *buf, rt_uint16_t len)
{
    rt_uint16_t crc = 0xFFFF;
    rt_uint16_t i, j;

    for (i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

static void mb_put_u16(rt_uint8_t *p, rt_uint16_t v)
{
    p[0] = (rt_uint8_t)(v >> 8);
    p[1] = (rt_uint8_t)(v & 0xFF);
}

static rt_uint16_t mb_get_u16(const rt_uint8_t *p)
{
    return (rt_uint16_t)((p[0] << 8) | p[1]);
}

/*
 * RTU 线上 CRC 是「低字节在前」，与寄存器数据(大端)正好相反。
 * CRC 必须走下面这两个函数，不能直接复用 mb_put_u16 / mb_get_u16。
 */
static void mb_put_crc(rt_uint8_t *p, rt_uint16_t v)
{
    p[0] = (rt_uint8_t)(v & 0xFF);
    p[1] = (rt_uint8_t)(v >> 8);
}

static rt_uint16_t mb_get_crc(const rt_uint8_t *p)
{
    return (rt_uint16_t)(p[0] | (p[1] << 8));
}

/* 把收到的原始字节打进日志，排查线上问题时一眼看清到底收到了什么 */
static void mb_hexdump(const char *tag, const rt_uint8_t *buf, rt_uint16_t len)
{
    char  line[80];
    int   pos = 0;
    int   i, n = len > 12 ? 12 : len;

    pos += rt_snprintf(line + pos, sizeof(line) - pos, "%s len=%d:", tag, len);
    for (i = 0; i < n; i++)
        pos += rt_snprintf(line + pos, sizeof(line) - pos, " %02X", buf[i]);
    if (len > n)
        rt_snprintf(line + pos, sizeof(line) - pos, " ...");

    LOG_D("%s", line);
}

/* ---------------------------------------------------------- 异常应答 */
static void mb_reply_err(rt_uint8_t func, rt_uint8_t code)
{
    rt_uint8_t  rsp[5];
    rt_uint16_t crc;

    rsp[0] = g_addr;
    rsp[1] = func | 0x80;
    rsp[2] = code;
    crc = mb_crc16(rsp, 3);
    mb_put_crc(&rsp[3], crc);

    mb_hexdump("tx-err", rsp, 5);
    rt_device_write(g_dev, 0, rsp, 5);
}

/* ------------------------------------------------------------ 帧处理 */
static void mb_handle_frame(rt_uint8_t *req, rt_uint16_t len)
{
    rt_uint8_t  func;
    rt_uint16_t crc, calc;
    rt_uint16_t start, qty, i;
    rt_uint8_t  rsp[MB_FRAME_MAX];
    rt_uint16_t rlen = 0;

    if (len < 4)
    {
        mb_hexdump("drop short", req, len);
        return;
    }

    /* 串口助手常在帧尾自动补 CR/LF，先剥掉，方便手动测试 */
    while (len > 4 && (req[len - 1] == 0x0D || req[len - 1] == 0x0A))
        len--;

    /* 只处理本机地址，广播(0)不应答 */
    if (req[0] != g_addr || g_addr == 0)
    {
        mb_hexdump("drop addr", req, len);
        return;
    }

    mb_hexdump("rx", req, len);

    calc = mb_crc16(req, len - 2);
    crc  = mb_get_crc(&req[len - 2]);
    if (crc != calc)
    {
        LOG_D("crc error: got 0x%04X calc 0x%04X len %d", crc, calc, len);
        return;
    }

    func = req[1];

    switch (func)
    {
    case 0x03:  /* 读保持寄存器 */
    {
        rt_uint8_t nbytes;

        if (len < 8)
            return;
        start = mb_get_u16(&req[2]);
        qty   = mb_get_u16(&req[4]);

        if (qty == 0 || qty > 125 || (start + qty) > REG_COUNT)
        {
            mb_reply_err(func, 0x02);   /* 非法数据地址 */
            return;
        }

        nbytes   = (rt_uint8_t)(qty * 2);
        rsp[0]   = g_addr;
        rsp[1]   = func;
        rsp[2]   = nbytes;
        for (i = 0; i < qty; i++)
            mb_put_u16(&rsp[3 + i * 2], g_hold[start + i]);

        rlen = 3 + nbytes;
        break;
    }

    case 0x06:  /* 写单个寄存器 */
    {
        if (len < 8)
            return;
        start = mb_get_u16(&req[2]);
        if (start >= REG_COUNT)
        {
            mb_reply_err(func, 0x02);
            return;
        }
        g_hold[start] = mb_get_u16(&req[4]);

        rsp[0] = g_addr;
        rsp[1] = func;
        mb_put_u16(&rsp[2], start);
        mb_put_u16(&rsp[4], g_hold[start]);
        rlen = 6;
        break;
    }

    case 0x10:  /* 写多个寄存器 */
    {
        rt_uint8_t nbytes;

        if (len < 11)
            return;
        start  = mb_get_u16(&req[2]);
        qty    = mb_get_u16(&req[4]);
        nbytes = req[6];

        if (qty == 0 || qty > 123 || nbytes != qty * 2 ||
            (start + qty) > REG_COUNT || len < (rt_uint16_t)(9 + nbytes))
        {
            mb_reply_err(func, 0x03);   /* 非法数据值 */
            return;
        }

        for (i = 0; i < qty; i++)
            g_hold[start + i] = mb_get_u16(&req[7 + i * 2]);

        rsp[0] = g_addr;
        rsp[1] = func;
        mb_put_u16(&rsp[2], start);
        mb_put_u16(&rsp[4], qty);
        rlen = 6;
        break;
    }

    default:
        mb_reply_err(func, 0x01);       /* 非法功能码 */
        return;
    }

    if (rlen)
    {
        crc = mb_crc16(rsp, rlen);
        mb_put_crc(&rsp[rlen], crc);
        mb_hexdump("tx", rsp, rlen + 2);
        rt_device_write(g_dev, 0, rsp, rlen + 2);
    }
}

/* -------------------------------------------------------- 接收回调 */
static rt_err_t mb_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&g_rx_sem);
    return RT_EOK;
}

/* --------------------------------------------------- 从站收发线程 */
static void mb_slave_entry(void *param)
{
    rt_uint8_t buf[MB_FRAME_MAX];
    rt_uint16_t idx = 0;
    rt_tick_t  last = 0;

    while (1)
    {
        int n = (int)rt_device_read(g_dev, 0, &buf[idx], sizeof(buf) - idx);

        if (n > 0)
        {
            idx += n;
            last = rt_tick_get();
            if (idx >= sizeof(buf))
                idx = 0;                /* 溢出，丢弃重来 */
            continue;
        }

        if (idx > 0)
        {
            rt_tick_t idle = rt_tick_get() - last;

            if (idle >= MB_T35_TICKS)
            {
                /*
                 * 字节流可能比 T3.5 慢到达：短帧先别急着丢，继续等；
                 * 超过 MB_FRAME_TIMEOUT 仍凑不齐最小帧长，才当垃圾丢弃并打印。
                 */
                if (idx >= 4 || idle >= MB_FRAME_TIMEOUT)
                {
                    mb_handle_frame(buf, idx);
                    idx = 0;
                }
                else
                {
                    rt_thread_mdelay(1);
                }
            }
            else
            {
                rt_thread_mdelay(1);
            }
        }
        else
        {
            rt_sem_take(&g_rx_sem, RT_WAITING_FOREVER);
        }
    }
}

/* ------------------------------------------------- 传感器刷新线程 */
static void mb_sensor_entry(void *param)
{
#ifdef PKG_USING_AHT10
    aht10_device_t aht10 = RT_NULL;
#endif
#ifdef PKG_USING_AP3216C
    ap3216c_device_t ap = RT_NULL;
#endif
#ifdef PKG_USING_ICM20608
    icm20608_device_t icm = RT_NULL;
#endif
    rt_uint32_t sec = 0;
    rt_uint16_t err = 0;

#ifdef PKG_USING_AHT10
    aht10 = aht10_init("i2c3");
    if (!aht10) { err |= (1 << 0); LOG_E("AHT10 init failed (i2c3)"); }
#endif
#ifdef PKG_USING_AP3216C
    ap = ap3216c_init("i2c2");
    if (!ap)    { err |= (1 << 1); LOG_E("AP3216C init failed (i2c2)"); }
#endif
#ifdef PKG_USING_ICM20608
    icm = icm20608_init("i2c2");
    if (!icm)   { err |= (1 << 2); LOG_E("ICM20608 init failed (i2c2)"); }
#endif

    g_hold[REG_ERR] = err;

    while (1)
    {
#ifdef PKG_USING_AHT10
        if (aht10)
        {
            g_hold[REG_TEMP] = (rt_uint16_t)(rt_int16_t)(aht10_read_temperature(aht10) * 10);
            g_hold[REG_HUMI] = (rt_uint16_t)(rt_int16_t)(aht10_read_humidity(aht10) * 10);
        }
#endif
#ifdef PKG_USING_AP3216C
        if (ap)
        {
            g_hold[REG_LUX] = (rt_uint16_t)(rt_int16_t)(ap3216c_read_ambient_light(ap) * 10);
            g_hold[REG_PS]  = (rt_uint16_t)ap3216c_read_ps_data(ap);
        }
#endif
#ifdef PKG_USING_ICM20608
        if (icm)
        {
            rt_int16_t ax, ay, az, gx, gy, gz;
            if (icm20608_get_accel(icm, &ax, &ay, &az) == RT_EOK)
            {
                g_hold[REG_ACC_X] = (rt_uint16_t)ax;
                g_hold[REG_ACC_Y] = (rt_uint16_t)ay;
                g_hold[REG_ACC_Z] = (rt_uint16_t)az;
            }
            if (icm20608_get_gyro(icm, &gx, &gy, &gz) == RT_EOK)
            {
                g_hold[REG_GYRO_X] = (rt_uint16_t)gx;
                g_hold[REG_GYRO_Y] = (rt_uint16_t)gy;
                g_hold[REG_GYRO_Z] = (rt_uint16_t)gz;
            }
        }
#endif
        g_hold[REG_HEARTBEAT]++;
        g_hold[REG_UPTIME] = (rt_uint16_t)(++sec);

        rt_thread_mdelay(500);
    }
}

/* ------------------------------------------------------------ 启动 */
static int mb_slave_init(void)
{
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;
    rt_thread_t tid;

    g_dev = rt_device_find(MB_DEV_NAME);
    if (!g_dev)
    {
        rt_kprintf("[mbs] find %s failed\n", MB_DEV_NAME);
        return -RT_ERROR;
    }

    rt_sem_init(&g_rx_sem, "mbs_rx", 0, RT_IPC_FLAG_FIFO);

    if (rt_device_open(g_dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        rt_kprintf("[mbs] open %s failed\n", MB_DEV_NAME);
        return -RT_ERROR;
    }

    cfg.baud_rate = MB_BAUD;
    rt_device_control(g_dev, RT_DEVICE_CTRL_CONFIG, &cfg);
    rt_device_set_rx_indicate(g_dev, mb_rx_ind);

    tid = rt_thread_create("mbs", mb_slave_entry, RT_NULL, 1024, 10, 20);
    if (tid) rt_thread_startup(tid);

    tid = rt_thread_create("mbss", mb_sensor_entry, RT_NULL, 2048, 20, 20);
    if (tid) rt_thread_startup(tid);

    rt_kprintf("[mbs] slave addr=%d on %s @%d, %d regs\n",
               g_addr, MB_DEV_NAME, MB_BAUD, REG_COUNT);
    return RT_EOK;
}
INIT_APP_EXPORT(mb_slave_init);

/* ------------------------------------------------------ 调试命令 */
static void mb_dump(void)
{
    rt_kprintf("[mbs] heartbeat=%d uptime=%ds err=0x%04X\n",
               g_hold[REG_HEARTBEAT], g_hold[REG_UPTIME], g_hold[REG_ERR]);
    rt_kprintf("[mbs] temp=%d.%d C  humi=%d.%d %%\n",
               (rt_int16_t)g_hold[REG_TEMP] / 10, (rt_int16_t)g_hold[REG_TEMP] % 10,
               (rt_int16_t)g_hold[REG_HUMI] / 10, (rt_int16_t)g_hold[REG_HUMI] % 10);
    rt_kprintf("[mbs] lux=%d.%d  ps=%d\n",
               (rt_int16_t)g_hold[REG_LUX] / 10, (rt_int16_t)g_hold[REG_LUX] % 10,
               g_hold[REG_PS]);
    rt_kprintf("[mbs] acc=%d,%d,%d gyro=%d,%d,%d\n",
               (rt_int16_t)g_hold[REG_ACC_X], (rt_int16_t)g_hold[REG_ACC_Y],
               (rt_int16_t)g_hold[REG_ACC_Z], (rt_int16_t)g_hold[REG_GYRO_X],
               (rt_int16_t)g_hold[REG_GYRO_Y], (rt_int16_t)g_hold[REG_GYRO_Z]);
    rt_kprintf("[mbs] do_ctrl=0x%04X pwm=%d\n",
               g_hold[REG_DO_CTRL], g_hold[REG_PWM_DUTY]);
}
MSH_CMD_EXPORT(mb_dump, dump modbus holding registers);
