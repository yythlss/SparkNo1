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
#include <stdlib.h>      /* strtoul：mb_do 命令用 */
#include <string.h>      /* strcmp：mb_fx_stat probe 子命令用 */

#ifdef PKG_USING_AHT10
#include "aht10.h"
#endif
#ifdef PKG_USING_AP3216C
#include "ap3216c.h"
#endif
#ifdef PKG_USING_ICM20608
#include "icm20608.h"
#endif

#ifdef BSP_USING_LED_MATRIX
#include "drv_matrix_led.h"   /* 板载 WS2812 / LED 矩阵（执行器 RGB 用） */
#endif

#ifdef BSP_USING_RS485
#include "drv_rs485.h"        /* 现场总线：RS485 主站透传（第 7 步） */
#endif

#define DBG_TAG "mbs"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* ------------------------------------------------------------ 配置 */
#define MB_DEV_NAME      "uart3"
#define MB_SLAVE_ADDR    1

/* 真实 PWM 输出（功能扩展）：TIM2_CH4（PA3，CubeMX 已配为 PWM Generation）。
 * 由 REG_PWM_DUTY（0.1%，0~1000）驱动，独立于 RGB 亮度演示，可接电机/阀门/调光负载。
 * 需在 menuconfig 开启 BSP_USING_PWM2 + BSP_USING_PWM2_CH4；引脚 PA3 需空闲。
 * 设备未找到时自动回退（仅 RGB 亮度演示，不影响其它功能）。 */
#define DBG_PWM_DEV_NAME  "pwm2"
#define DBG_PWM_CH        4
#define DBG_PWM_PERIOD_NS 1000000U   /* 1 kHz 周期 */
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
#define REG_LED_FX       0x0102     /* RW 灯效模式，见 MB_FX_* */
#define REG_LED_FX_ARG   0x0103     /* RW 灯效参数（速度/方向），随模式含义不同 */

#define REG_RS485_CTRL   0x0300     /* RW 写 1 = 触发一次透传（自清，见 REG_RS485_ST） */
#define REG_RS485_ADDR   0x0301     /* RW 目标从站地址 1~247 */
#define REG_RS485_FUNC   0x0302     /* RW Modbus 功能码（3 读 / 6 写 / 16 写多） */
#define REG_RS485_QTY    0x0303     /* RW 寄存器数（FC03/FC16）或 寄存器个数 */
#define REG_RS485_ST     0x0304     /* RO 状态：0 空闲/1 忙/2 成功/3 超时/4 校验错 */
#define REG_RS485_OUTLEN 0x0305     /* RO 本次发出的请求字节数 */
#define REG_RS485_INLEN  0x0306     /* RO 收到的应答字节数 */
#define REG_RS485_ERRCNT 0x0307     /* RO 累计失败次数（超时+校验错） */
#define REG_RS485_DATA   0x0380     /* RW 透传数据区，最多 RS485_BUF_MAX 个寄存器 */

/* 透传缓冲上限（单位：寄存器 = 字节，数据区是「一个寄存器装一个字节」的紧凑排布）。
 * 一个 Modbus RTU 帧最长 256 字节，这里取 128 字节窗口：
 * 够 FC03 读 ~60 个寄存器（应答 5+2*qty），已经是现场扫描的实用上限。
 *
 * ⚠️ 曾经踩过的坑：这里原本是 64，但收/发循环却用 RS485_BUF_MAX*2 当下标上限，
 *    于是最大下标 = 0x0380 + 127 = 1023，而 g_hold 只有 REG_COUNT=0x03C0=960 项 ——
 *    实打实的越界写。现在窗口、下标上限、REG_COUNT 三者对齐：0x0380+128 = 0x0400。 */
#define RS485_BUF_MAX    128

#define REG_COUNT        0x0400     /* 必须 = REG_RS485_DATA + RS485_BUF_MAX，别再留余量 */

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

/* ------------------------------------------------ 执行器（第 6 步） */
#define PIN_BEEP    GET_PIN(B, 0)   /* 蜂鸣器引脚：星火一号 PB0（若无声改 PB1） */
#define DO_LED_R    (1u << 0)
#define DO_LED_G    (1u << 1)
#define DO_LED_B    (1u << 2)
#define DO_BEEP     (1u << 3)

/* ---- 灯效模式（REG_LED_FX）---- */
#define MB_FX_SOLID      0   /* 纯色：DO_CTRL 三位 + PWM_DUTY 亮度（默认，兼容原行为） */
#define MB_FX_BREATH     1   /* 呼吸：整灯平滑明暗循环 */
#define MB_FX_RAINBOW    2   /* 彩虹：19 颗 hue 均匀铺开并整体旋转 */
#define MB_FX_CHASE      3   /* 跑马：单点高亮绕圈，尾部拖尾衰减 */
#define MB_FX_COLOR_CYCLE 4  /* 颜色循环：整灯按红→绿→蓝→品红→青→黄 渐变 */
#define MB_FX_FLASH      5   /* 闪烁：整灯方波闪，参数为占空比 1~9 */
#define MB_FX_MARQUEE    6   /* 双向扫描：两灯对向运动，到端点折返 */
#define MB_FX_SPARKLE    7   /* 星光闪烁：随机位置随机亮度闪烁 */
#define MB_FX_MAX        8

/* 灯效刷新周期（tick）。灯效线程按此节拍推进动画，参数 REG_LED_FX_ARG 可调速。 */
#define MB_FX_TICK_MS    20u

/* 灯效线程栈大小（本工程走 RT_USING_HEAP 动态栈，此值是分配字节数） */
#define MB_FX_STACK_SIZE 768

/* 板载 SK6805 灯珠数，须与 drv_matrix_led.c 的 BSP_ONBOAR_LED_NUMS 一致（19） */
#ifdef BSP_USING_ONBOARD_LED_MATRIX
#define MB_LED_NUMS      19
#endif

/* 前置声明：mb_actuator_init 里要先调用它，避免隐式声明与 static 定义冲突 */
static void mb_actuator_apply(void);

/* 真实 PWM 设备句柄（TIM2_CH4 / PA3），未启用 BSP_USING_PWM2 时为 RT_NULL */
static rt_device_t g_pwm = RT_NULL;

/* 把 DO_CTRL / PWM_DUTY 落到硬件：蜂鸣器(GPIO 直驱) + 板载 RGB(WS2812, 19 颗 SK6805)。
 * PWM_DUTY 单位 0.1%，范围 0~1000；除映射为 RGB 亮度（演示）外，同时驱动真实 PWM 输出。
 * 注意：led_matrix_fill_rgb() 只写内存缓冲，必须再调 led_matrix_reflash() 才推到灯上。
 * 仅当 menuconfig 开启 BSP_USING_LED_MATRIX + BSP_USING_ONBOARD_LED_MATRIX 时 RGB 才编译生效。 */
static void mb_actuator_init(void)
{
    rt_pin_mode(PIN_BEEP, PIN_MODE_OUTPUT);
    rt_pin_write(PIN_BEEP, PIN_LOW);   /* 默认静音 */

    /* 真实 PWM：启用 TIM2_CH4（PA3）输出，占空比初值取当前 REG_PWM_DUTY */
    g_pwm = rt_device_find(DBG_PWM_DEV_NAME);
    if (g_pwm != RT_NULL) {
        if (rt_pwm_enable(g_pwm, DBG_PWM_CH) == RT_EOK) {
            LOG_I("real PWM enabled on %s ch%d (PA3)", DBG_PWM_DEV_NAME, DBG_PWM_CH);
        } else {
            LOG_W("enable %s ch%d failed", DBG_PWM_DEV_NAME, DBG_PWM_CH);
            g_pwm = RT_NULL;
        }
    } else {
        LOG_I("%s not found, PWM output disabled (enable BSP_USING_PWM2 in menuconfig)",
              DBG_PWM_DEV_NAME);
    }

#ifdef BSP_USING_ONBOARD_LED_MATRIX
    led_matrix_init();                  /* 初始化 WS2812 时序缓冲（幂等，可重复调用） */
#endif
    mb_actuator_apply();
}

/* 由 DO_CTRL/PWM_DUTY 算出基准纯色（供灯效做颜色基准，避免各效果各写一遍判断） */
static void mb_solid_color(rt_uint16_t do_ctrl, rt_uint16_t duty,
                           rt_uint8_t *r, rt_uint8_t *g, rt_uint8_t *b)
{
    rt_uint8_t scale = (rt_uint8_t)((duty * 255u) / 1000u);  /* 占空比 0~1000 → 亮度 0~255 */

    *r = (do_ctrl & DO_LED_R) ? scale : 0;
    *g = (do_ctrl & DO_LED_G) ? scale : 0;
    *b = (do_ctrl & DO_LED_B) ? scale : 0;
}

static void mb_actuator_apply(void)
{
    rt_uint16_t do_ctrl = g_hold[REG_DO_CTRL];
    rt_uint16_t duty    = g_hold[REG_PWM_DUTY];          /* 0.1%，0~1000 */

    rt_pin_write(PIN_BEEP, (do_ctrl & DO_BEEP) ? PIN_HIGH : PIN_LOW);

    /* 真实 PWM 输出：REG_PWM_DUTY(0~1000) → 占空比 0~100%，驱动 TIM2_CH4(PA3)。
     * 与 RGB 亮度演示并存，两者读同一个寄存器。 */
    if (g_pwm != RT_NULL) {
        rt_uint32_t pulse = (DBG_PWM_PERIOD_NS / 1000U) *
                            ((duty > 1000u) ? 1000u : duty);   /* ns */
        rt_pwm_set(g_pwm, DBG_PWM_CH, DBG_PWM_PERIOD_NS, pulse);
    }

#ifdef BSP_USING_ONBOARD_LED_MATRIX
    /* 非纯色模式由灯效线程接管，这里不要抢着写，否则动画会被每 500ms 的周期刷新打断 */
    if (g_hold[REG_LED_FX] == MB_FX_SOLID)
    {
        rt_uint8_t r, g, b;
        mb_solid_color(do_ctrl, duty, &r, &g, &b);
        led_matrix_fill_rgb(r, g, b);    /* 写内存缓冲 */
        led_matrix_reflash();            /* 关键：推到 WS2812 灯上，否则不亮 */
    }
#endif
}

/* ====================== RS485 透传网关（第 7 步） ======================
 * 让主机不用懂 Modbus 也能扫现场设备：主机把「请求帧原始字节」写进
 * REG_RS485_DATA 起始区，再向 REG_RS485_CTRL 写 1 触发；从机把帧发到 RS485，
 * 收回应答后回填同一片数据区并更新状态寄存器。
 *
 * 这样 D12x 侧不需要任何 Modbus 主站栈，纯做「字节搬运」即可。
 *
 * 状态机：0 空闲 / 1 忙 / 2 成功 / 3 超时 / 4 校验错 / 5 帧超长
 * ==================================================================== */
#ifdef BSP_USING_RS485

#define RS485_ST_IDLE     0
#define RS485_ST_BUSY     1
#define RS485_ST_OK       2
#define RS485_ST_TIMEOUT  3
#define RS485_ST_CRC_ERR  4
#define RS485_ST_OVERFLOW 5

/* 收应答：等第一个字节用 RS485_RECV_TIMEOUT_MS，之后按帧间隔累积 */
#define RS485_RECV_TIMEOUT_MS   300
#define RS485_POLL_MS           5

/* 静默发送一帧（自管 DE 方向）。
 * 不用 drv_rs485.c 的 rs485_send_data()：它内部会 rt_kprintf 逐字节打印，
 * 在透传/扫描场景既是噪声又拖慢时序（Modbus RTU 要求帧内连续发送）。 */
static void mb_rs485_tx(rt_uint8_t *buf, rt_uint16_t len)
{
    rt_pin_write(BSP_RS485_RTS_PIN, PIN_HIGH);        /* DE=1 → 发送 */
    rt_device_write(rs485_serial, 0, buf, len);
    /* 等最后一个字节真正移出（串口是同步写完的，这里给一小段收尾时间） */
    rt_thread_mdelay(2);
    rt_pin_write(BSP_RS485_RTS_PIN, PIN_LOW);         /* DE=0 → 接收 */
}

/* ------------------------- LOOPBACK 自测开关 -------------------------
 * 与主机侧 DBG_SLAVE_LOOPBACK 同一套方法论：现场往往「没有第二台 Modbus 设备可扫」，
 * 那就在软件里造一个假从站 —— 把「发到 RS485 再等应答」换成「本地合成一帧合法应答」，
 * 其后的 CRC 校验 → 回填 → 状态机 → 解码打印全部照真跑。
 *
 * 这样能证明的是：网关这条**软件链路**是对的。真机只需 mb_rs485_lb 0 切回真实 RS485。
 * 注意它证不了硬件（DE 时序、终端电阻、波特率），别把自测通过当成链路通。
 */
static rt_uint8_t g_rs485_loopback = 0;   /* 1 = 用合成应答代替真实 RS485 */
static rt_uint8_t g_rs485_lb_badcrc = 0;  /* 1 = 合成应答时故意打坏 CRC，验错误分支 */

/* 按 Modbus 规范合成一帧「假从站」应答，用来喂给网关的后半段处理 */
static rt_uint16_t mb_rs485_sim_rsp(const rt_uint8_t *req, rt_uint16_t req_len,
                                    rt_uint8_t *rsp, rt_uint16_t cap)
{
    rt_uint8_t  func;
    rt_uint16_t n = 0, i, qty, crc;

    if (req_len < 2 || cap < 8)
        return 0;

    func  = req[1];
    rsp[0] = req[0];
    rsp[1] = func;

    if (func == 0x03 && req_len >= 6)
    {
        qty = mb_get_u16(&req[4]);
        if (qty == 0 || qty > 60) qty = 2;          /* 保守，别撑爆 cap */
        rsp[2] = (rt_uint8_t)(qty * 2);
        /* 假数据：0x0100, 0x0101, ... 便于一眼看出回填有没有错位 */
        for (i = 0; i < qty; i++)
            mb_put_u16(&rsp[3 + i * 2], (rt_uint16_t)(0x0100 + i));
        n = (rt_uint16_t)(3 + qty * 2);
    }
    else if ((func == 0x06 || func == 0x10) && req_len >= 6)
    {
        /* 写操作应答 = 回显「起始地址 + 数量/值」 */
        rsp[2] = req[2];
        rsp[3] = req[3];
        rsp[4] = req[4];
        rsp[5] = req[5];
        n = 6;
    }
    else
    {
        rsp[1] = (rt_uint8_t)(func | 0x80);
        rsp[2] = 0x01;                              /* 非法功能码 */
        n = 3;
    }

    if (n + 2 > cap)
        return 0;

    crc = mb_crc16(rsp, n);
    mb_put_crc(&rsp[n], crc);                       /* RTU：CRC 低字节在前 */
    return (rt_uint16_t)(n + 2);
}

/* 接收缓冲（Modbus RTU 最大 256 字节） */
static rt_uint8_t  g_rs485_rxbuf[MB_FRAME_MAX];
static rt_uint16_t g_rs485_rxlen;

/* 由 drv_rs485 的接收线程调用（本工程 BSP_USING_LED_MATRIX_RS485_DEMO 开启时
 * rs485_init() 创建的线程入口就是 slave_check.c 的 led_matrix_receieve_task） */
void mb_rs485_feed_byte(rt_uint8_t byte)
{
    if (g_rs485_rxlen < MB_FRAME_MAX)
    {
        g_rs485_rxbuf[g_rs485_rxlen++] = byte;
    }
    else
    {
        /* 溢出：直接判失败，别让脏数据进 CRC */
        g_hold[REG_RS485_ST] = RS485_ST_CRC_ERR;
    }
}

/* 供 slave_check.c 的接收线程调用：等待一帧收齐（超时则返回 0 长度） */
rt_uint16_t mb_rs485_wait_frame(rt_uint8_t *buf, rt_uint16_t cap, rt_uint32_t timeout_ms)
{
    rt_uint32_t waited = 0;

    g_rs485_rxlen = 0;
    while (waited < timeout_ms)
    {
        /* 有新字节就持续收，直到帧间隔超时 */
        rt_thread_mdelay(RS485_POLL_MS);
        waited += RS485_POLL_MS;
        if (g_rs485_rxlen > 0)
        {
            /* 连续 RS485_POLL_MS 没再收到新字节 → 帧结束 */
            rt_uint16_t last = g_rs485_rxlen;
            rt_thread_mdelay(RS485_POLL_MS * 2);
            if (g_rs485_rxlen == last)
                break;
        }
    }

    if (g_rs485_rxlen == 0 || g_rs485_rxlen > cap)
        return 0;
    for (rt_uint16_t i = 0; i < g_rs485_rxlen; i++)
        buf[i] = g_rs485_rxbuf[i];
    return g_rs485_rxlen;
}

/* 网关线程：等主机触发 → 发帧 → 收应答 → 回填 */
static void mb_rs485_gw_entry(void *parameter)
{
    (void)parameter;

    for (;;)
    {
        rt_uint8_t  req[MB_FRAME_MAX];
        rt_uint8_t  rsp[MB_FRAME_MAX];
        rt_uint16_t req_len, rsp_len, i;
        rt_uint16_t want;

        if (g_hold[REG_RS485_CTRL] != 1)
        {
            rt_thread_mdelay(20);
            continue;
        }

        /* --- 组请求帧 --- */
        g_hold[REG_RS485_CTRL] = 0;              /* 自清，允许下次触发 */
        g_hold[REG_RS485_ST]   = RS485_ST_BUSY;

        /* 请求长度 = 1(地址) + 1(功能码) + 2(数据) + 2(CRC) 的保守上限：
         * 直接用主机填好的字节数寄存器 */
        want = g_hold[REG_RS485_OUTLEN];
        if (want == 0 || want > RS485_BUF_MAX) want = 8;   /* 默认 FC03 读 4 寄存器 */

        req_len = 0;
        for (i = 0; i < want && i < RS485_BUF_MAX; i++)
            req[req_len++] = (rt_uint8_t)(g_hold[REG_RS485_DATA + i] & 0xFFu);

        /* 补 CRC（若主机没自己算好，这里按 Modbus RTU 小端补） */
        if (req_len >= 2)
        {
            rt_uint16_t crc = mb_crc16(req, req_len - 2);
            req[req_len - 2] = (rt_uint8_t)(crc & 0xFFu);
            req[req_len - 1] = (rt_uint8_t)(crc >> 8);
        }

        g_hold[REG_RS485_OUTLEN] = req_len;
        g_hold[REG_RS485_INLEN]  = 0;

        /* --- 发送 --- */
        if (g_rs485_loopback)
        {
            /* 自测：本地合成应答，后面的 CRC/回填/状态机全是真的，只有「线」是假的 */
            rsp_len = mb_rs485_sim_rsp(req, req_len, rsp, MB_FRAME_MAX);
            if (g_rs485_lb_badcrc && rsp_len >= 2)
                rsp[rsp_len - 2] ^= 0xFF;           /* 打坏 CRC 低位，验错误分支 */
            rt_thread_mdelay(5);                    /* 模拟一点传输耗时 */
            LOG_D("rs485 [LB] sim rsp %d bytes", rsp_len);
        }
        else
        {
            mb_rs485_tx(req, req_len);
            LOG_D("rs485 tx %d bytes: %02X %02X ...", req_len, req[0], req[1]);
            rsp_len = mb_rs485_wait_frame(rsp, MB_FRAME_MAX, RS485_RECV_TIMEOUT_MS);
        }

        if (rsp_len == 0)
        {
            g_hold[REG_RS485_ST] = RS485_ST_TIMEOUT;
            g_hold[REG_RS485_ERRCNT]++;
            LOG_W("rs485 timeout");
            continue;
        }

        /* --- 校验 CRC：低字节在前 --- */
        if (rsp_len >= 3)
        {
            rt_uint16_t calc = mb_crc16(rsp, rsp_len - 2);
            rt_uint16_t got  = (rt_uint16_t)(rsp[rsp_len - 2] | (rsp[rsp_len - 1] << 8));
            if (calc != got)
            {
                g_hold[REG_RS485_ST] = RS485_ST_CRC_ERR;
                g_hold[REG_RS485_ERRCNT]++;
                LOG_W("rs485 crc err calc=%04X got=%04X", calc, got);
                continue;
            }
        }

        /* 超窗口的帧不能截断回填 —— 半截帧比没有帧更危险（主机会按长度解析出假数据）。
         * 直接判失败并计数，让调用方看到明确的状态 = toolong。 */
        if (rsp_len > RS485_BUF_MAX)
        {
            g_hold[REG_RS485_ST] = RS485_ST_OVERFLOW;
            g_hold[REG_RS485_ERRCNT]++;
            LOG_W("rs485 frame too long: %d > %d", rsp_len, RS485_BUF_MAX);
            continue;
        }

        /* --- 成功：回填应答原始字节 --- */
        for (i = 0; i < rsp_len; i++)
            g_hold[REG_RS485_DATA + i] = rsp[i];

        g_hold[REG_RS485_INLEN] = rsp_len;
        g_hold[REG_RS485_ST]    = RS485_ST_OK;
        LOG_D("rs485 rx ok, %d bytes", rsp_len);
    }
}

static void mb_rs485_gw_start(void)
{
    rt_thread_t tid;

    tid = rt_thread_create("mbrs485", mb_rs485_gw_entry, RT_NULL, 1024, 15, 20);
    if (tid) rt_thread_startup(tid);
    else    LOG_E("rs485 gw thread failed");
}
#endif /* BSP_USING_RS485 */

/* ====================== 灯效引擎（REG_LED_FX） ======================
 * 由独立线程 mbfx 按 MB_FX_TICK_MS 节拍推进，读 REG_LED_FX / REG_LED_FX_ARG，
 * 用 led_matrix_set_color() 逐颗灯画下一帧，再 led_matrix_reflash() 一次推灯。
 *
 * 关键约束（踩过的坑）：
 *   1) 所有 set_color 只改内存缓冲，整帧画完必须统一 reflash 一次；
 *   2) reflash 内部会等 DMA 空闲，是阻塞调用，所以线程节拍不宜过快；
 *   3) 整段包在 BSP_USING_ONBOARD_LED_MATRIX 下，未开驱动时仍能编译（仅蜂鸣器生效）。
 * ==================================================================== */
#ifdef BSP_USING_ONBOARD_LED_MATRIX

/* --- 简易 HSV→RGB：h 0~359（度），s/v 0~255。用于彩虹类效果，避免浮点。 --- */
static void mb_hsv2rgb(rt_uint16_t h, rt_uint8_t s, rt_uint8_t v,
                       rt_uint8_t *r, rt_uint8_t *g, rt_uint8_t *b)
{
    rt_uint8_t region, rem, p, q, t;

    if (s == 0) { *r = *g = *b = v; return; }

    region = (rt_uint8_t)(h / 60u);
    rem    = (rt_uint8_t)((h % 60u) * 255u / 60u);
    p      = (rt_uint8_t)((v * (255u - s)) / 255u);
    q      = (rt_uint8_t)((v * (255u - (s * rem) / 255u)) / 255u);
    t      = (rt_uint8_t)((v * (255u - (s * (255u - rem)) / 255u)) / 255u);

    switch (region)
    {
    case 0:  *r = v; *g = t; *b = p; break;
    case 1:  *r = q; *g = v; *b = p; break;
    case 2:  *r = p; *g = v; *b = t; break;
    case 3:  *r = p; *g = q; *b = v; break;
    case 4:  *r = t; *g = p; *b = v; break;
    default: *r = v; *g = p; *b = q; break;
    }
}

/* 灯效线程私有状态 */
static rt_uint16_t          g_fx_phase;    /* 各效果自用的相位/位置游标 */
static rt_uint32_t          g_fx_rand;     /* 星光效果的伪随机数（线性同余，避免依赖 rand()） */

static rt_uint32_t mb_fx_next_rand(void)
{
    g_fx_rand = g_fx_rand * 1103515245u + 12345u;
    return (g_fx_rand >> 16) & 0x7FFFu;
}

/* 画一帧：按当前模式把 19 颗灯的缓冲写好（不 reflash） */
static void mb_fx_render_frame(rt_uint16_t fx, rt_uint16_t arg)
{
    rt_uint16_t do_ctrl = g_hold[REG_DO_CTRL];
    rt_uint16_t duty    = g_hold[REG_PWM_DUTY];
    rt_uint8_t  br, bg, bb;              /* 基准色（纯色/呼吸/闪烁用） */
    rt_uint8_t  r, g, b;
    rt_int16_t  i;

    mb_solid_color(do_ctrl, duty ? duty : 1000u, &br, &bg, &bb);
    if (fx == MB_FX_SOLID)
    {
        /* 纯色由 mb_actuator_apply() 负责，这里不画 */
        return;
    }

    switch (fx)
    {
    case MB_FX_BREATH:
        {
            /* 用 0~510 的三角波映射到 0~255 亮度：0→255→0 往返，无突变 */
            rt_uint16_t p = g_fx_phase & 0x1FFu;      /* 0~511 */
            rt_uint8_t  lv = (p < 256u) ? (rt_uint8_t)p
                                         : (rt_uint8_t)(510u - p);
            /* 按 DO_CTRL 位决定通道；PWM_DUTY 只在纯色模式生效，这里保持满亮呼吸 */
            r = (do_ctrl & DO_LED_R) ? lv : 0;
            g = (do_ctrl & DO_LED_G) ? lv : 0;
            b = (do_ctrl & DO_LED_B) ? lv : 0;
            led_matrix_fill_rgb(r, g, b);
        }
        break;

    case MB_FX_RAINBOW:
        {
            /* 19 颗灯 hue 均匀铺开成一条彩虹，整体随 phase 旋转 */
            rt_uint16_t step = 360u / MB_LED_NUMS;
            for (i = 0; i < (rt_int16_t)MB_LED_NUMS; i++)
            {
                rt_uint16_t h = (rt_uint16_t)((g_fx_phase + (rt_uint16_t)i * step) % 360u);
                mb_hsv2rgb(h, 255, 255, &r, &g, &b);
                led_matrix_set_color((rt_uint16_t)i, (pixel_rgb_t){r, g, b});
            }
        }
        break;

    case MB_FX_CHASE:
        {
            /* 头部高亮、逐级衰减的拖尾，arg 控步进速度（1~9） */
            rt_uint8_t head = (rt_uint8_t)(g_fx_phase % MB_LED_NUMS);
            for (i = 0; i < (rt_int16_t)MB_LED_NUMS; i++)
            {
                rt_uint8_t d = (rt_uint8_t)((head - (rt_uint8_t)i + MB_LED_NUMS) % MB_LED_NUMS);
                rt_uint8_t lv = (d < 6u) ? (rt_uint8_t)(255u - d * 42u) : 0;
                mb_hsv2rgb((rt_uint16_t)(i * 18u % 360u), 255, lv, &r, &g, &b);
                led_matrix_set_color((rt_uint16_t)i, (pixel_rgb_t){r, g, b});
            }
        }
        break;

    case MB_FX_COLOR_CYCLE:
        {
            /* 整灯一次只显示一个色，按 60° 步进循环（红黄绿青蓝品红） */
            rt_uint16_t h = (g_fx_phase % 360u);
            mb_hsv2rgb(h, 255, 255, &r, &g, &b);
            led_matrix_fill_rgb(r, g, b);
        }
        break;

    case MB_FX_FLASH:
        {
            /* 方波闪：arg(1~9) 为亮相占空比百分比 */
            rt_uint8_t on_duty = arg ? (rt_uint8_t)(arg * 10u) : 50u;
            if (on_duty > 100u) on_duty = 100u;
            if ((g_fx_phase % 100u) < on_duty)
                led_matrix_fill_rgb(br, bg, bb);
            else
                led_matrix_fill_rgb(0, 0, 0);
        }
        break;

    case MB_FX_MARQUEE:
        {
            /* 两个亮点对向运动、到端点折返。
             * 注意：位置必须恒在 0~N-1。led_matrix_set_color 的边界判断是 `n > N`（非 >=），
             * 传 n==N 会越界写 raw_led_buffer，故这里用三角波把相位折到 0~N-1 再用。 */
            rt_uint8_t period = (rt_uint8_t)(2u * (MB_LED_NUMS - 1u));   /* 0→N-1→0 */
            rt_uint8_t a   = (rt_uint8_t)(g_fx_phase % period);
            rt_uint8_t pos = (a < (MB_LED_NUMS - 1u)) ? a
                                                      : (rt_uint8_t)(period - a);
            rt_uint8_t pos2 = (MB_LED_NUMS - 1u) - pos;
            rt_int16_t k;

            for (k = 0; k < (rt_int16_t)MB_LED_NUMS; k++)
                led_matrix_set_color((rt_uint16_t)k, (pixel_rgb_t){0, 0, 0});
            led_matrix_set_color(pos, (pixel_rgb_t){255, 40, 0});
            led_matrix_set_color(pos2, (pixel_rgb_t){0, 80, 255});
        }
        break;

    case MB_FX_SPARKLE:
        {
            /* 每帧随机挑一颗灯闪一下，其余全灭，营造零星星光 */
            rt_uint8_t lv_on  = (rt_uint8_t)(80u + (mb_fx_next_rand() % 176u));
            rt_uint8_t off_at = (rt_uint8_t)(mb_fx_next_rand() % MB_LED_NUMS);
            for (i = 0; i < (rt_int16_t)MB_LED_NUMS; i++)
                led_matrix_set_color((rt_uint16_t)i, (pixel_rgb_t){0, 0, 0});
            mb_hsv2rgb((rt_uint16_t)(mb_fx_next_rand() % 360u), 255, lv_on, &r, &g, &b);
            led_matrix_set_color(off_at, (pixel_rgb_t){r, g, b});
        }
        break;

    default:
        break;
    }
}

/* 灯效线程：按固定节拍推进动画 */
static void mb_fx_thread_entry(void *parameter)
{
    rt_uint16_t fx, arg, last_fx = MB_FX_SOLID;
    rt_uint8_t  step;
    (void)parameter;

    for (;;)
    {
        fx  = g_hold[REG_LED_FX];
        arg = g_hold[REG_LED_FX_ARG];

        if (fx >= MB_FX_MAX) fx = MB_FX_SOLID;
        if (fx != last_fx) { g_fx_phase = 0; last_fx = fx; }

        if (fx != MB_FX_SOLID)
        {
            /* arg(1~9) 当速度倍率：越大步进越快；0/越界按默认 3 档 */
            step = (arg && arg <= 9u) ? (rt_uint8_t)arg : 3u;

            mb_fx_render_frame(fx, arg);
            led_matrix_reflash();        /* 整帧画完统一推灯一次 */

            g_fx_phase = (rt_uint16_t)(g_fx_phase + step);
        }

        rt_thread_mdelay(MB_FX_TICK_MS);
    }
}

/* 启动灯效线程
 * 注意：本工程开了 RT_USING_HEAP，生效的是 rt_thread_create 的「动态栈」6 参数版本
 *   rt_thread_create(name, entry, parameter, stack_size, priority, tick)
 * 若照抄「静态栈」版本的 7 参数写法（含 stack_start），会报 too many arguments。
 * 与 mb_slave_init() 里 mbs / mbss 的写法保持一致。 */
static void mb_fx_start(void)
{
    rt_thread_t tid;

    tid = rt_thread_create("mbfx", mb_fx_thread_entry, RT_NULL,
                           MB_FX_STACK_SIZE, 12, 5);
    if (tid)
        rt_thread_startup(tid);
    else
        rt_kprintf("[mbfx] create thread failed\n");
}
#endif /* BSP_USING_ONBOARD_LED_MATRIX */

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

    mb_actuator_apply();   /* 写 DO_CTRL / PWM_DUTY 后即时生效（读请求无副作用） */

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

        mb_actuator_apply();   /* 周期刷新执行器状态（覆盖通过 FC06/FC16 写入的值） */

        rt_thread_mdelay(500);
    }
}

/* ------------------------------------------------------------ 启动 */
static int mb_slave_init(void)
{
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;
    rt_thread_t tid;

    /* 执行器与灯效是**本地硬件**，必须在 uart 探测之前初始化。
     * 否则 uart3 一旦缺失（本工程 uart3 与 ESP8266 共用，常被关掉），
     * 下面的 return 会跳过后续全部初始化 → 蜂鸣器不响、RGB 全灭，
     * 且现象极具误导性（命令都正常回显，只是灯不亮）。 */
    mb_actuator_init();
#ifdef BSP_USING_ONBOARD_LED_MATRIX
    mb_fx_start();                 /* 灯效线程（纯色模式下不动作） */
#endif
#ifdef BSP_USING_RS485
    /* RS485 透传网关（第 7 步）同样是**独立本地硬件**，不能挂在 uart3 链路之下：
     * 先让驱动开 uart6 + DE(PG8) + 起接收线程，再起网关线程。 */
    if (rs485_init() == RT_EOK)
    {
        mb_rs485_gw_start();
        rt_kprintf("[mbs] rs485 gateway ready (uart6, PG14/PG9, DE=PG8)\n");
    }
    else
    {
        rt_kprintf("[mbs] rs485 init FAILED, gateway disabled\n");
    }
#endif

    g_dev = rt_device_find(MB_DEV_NAME);
    if (!g_dev)
    {
        rt_kprintf("[mbs] find %s failed (uart3? check BSP_USING_UART3 in menuconfig)\n",
                   MB_DEV_NAME);
        rt_kprintf("[mbs] actuator/led still works; modbus slave link is DOWN\n");
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
    rt_kprintf("[mbs] do_ctrl=0x%04X pwm=%d fx=%u arg=%u\n",
               g_hold[REG_DO_CTRL], g_hold[REG_PWM_DUTY],
               g_hold[REG_LED_FX], g_hold[REG_LED_FX_ARG]);
#ifdef BSP_USING_RS485
    {
        static const char *st[] = {"idle", "busy", "ok", "timeout", "crc-err", "toolong"};
        rt_uint16_t s = g_hold[REG_RS485_ST];
        rt_kprintf("[mbs] rs485: st=%s tx=%d rx=%d err=%d lb=%d\n",
                   (s <= 5) ? st[s] : "?",
                   g_hold[REG_RS485_OUTLEN], g_hold[REG_RS485_INLEN],
                   g_hold[REG_RS485_ERRCNT], (int)g_rs485_loopback);
    }
#endif
}
MSH_CMD_EXPORT(mb_dump, dump modbus holding registers);

/* 执行器手动测试：mb_do <do_ctrl> [pwm_duty]
 * 例：mb_do 0x07 500  -> 红绿蓝全亮、亮度 50%、蜂鸣器关
 *     mb_do 0x08      -> 仅蜂鸣器响（b3） */
static void mb_do(int argc, char **argv)
{
    if (argc >= 2)
        g_hold[REG_DO_CTRL] = (rt_uint16_t)strtoul(argv[1], RT_NULL, 0);
    if (argc >= 3)
        g_hold[REG_PWM_DUTY] = (rt_uint16_t)strtoul(argv[2], RT_NULL, 0);
    mb_actuator_apply();
    mb_dump();
}
MSH_CMD_EXPORT(mb_do, mb_do <do_ctrl> [pwm_duty] : drive actuator);

/* LED 矩阵确定性诊断：跑一条命令就知道宏到底编译进去了没有、底层能不能点亮。
 * 该函数本身不包在 #ifdef 内（永远编译）；内部按宏分支打印明确结论。 */
static void mb_led_diag(int argc, char **argv)
{
#ifdef BSP_USING_ONBOARD_LED_MATRIX
    rt_kprintf("[mbs] ONBOARD_LED_MATRIX = ON, driving ALL-WHITE via PA7/TIM3_CH2\n");
    led_matrix_init();
    led_matrix_fill_rgb(255, 255, 255);
    led_matrix_reflash();
    rt_kprintf("[mbs] reflash done. If LEDs still dark -> HW pin conflict on PA7 or DMA issue.\n");
#else
    rt_kprintf("[mbs] ONBOARD_LED_MATRIX = OFF -> RGB code was COMPILED OUT.\n");
    rt_kprintf("[mbs] Fix: menuconfig enable BSP_USING_LED_MATRIX + BSP_USING_ONBOARD_LED_MATRIX,\n");
    rt_kprintf("[mbs]      then Project->Clean and REBUILD (must clean old .o), reflash.\n");
#endif
}
MSH_CMD_EXPORT(mb_led_diag, diagnose onboard led matrix: mb_led_diag);

/* 灯效手动测试：mb_fx <mode> [arg]
 *   mode: 0 纯色(默认) 1 呼吸 2 彩虹 3 跑马 4 颜色循环 5 闪烁 6 双向扫描 7 星光
 *   arg : 速度 1~9（默认 3）；mode=5 时为占空比 1~9（默认 5）
 * 例：mb_fx 2 5   -> 彩虹，速度 5 档
 *     mb_fx 0     -> 回到纯色（由 mb_do / 寄存器 0x0100 决定颜色亮度） */
static const char *mb_fx_name[MB_FX_MAX] =
{
    "solid", "breath", "rainbow", "chase", "color-cycle", "flash", "marquee", "sparkle"
};

static void mb_fx(int argc, char **argv)
{
    rt_uint16_t mode, arg = 3;

    if (argc < 2)
    {
        rt_kprintf("[mbfx] cur=%u(%s) arg=%u\n",
                   g_hold[REG_LED_FX],
                   (g_hold[REG_LED_FX] < MB_FX_MAX) ? mb_fx_name[g_hold[REG_LED_FX]] : "?",
                   g_hold[REG_LED_FX_ARG]);
        rt_kprintf("[mbfx] 0=solid 1=breath 2=rainbow 3=chase 4=color-cycle"
                   " 5=flash 6=marquee 7=sparkle   [arg=1..9 speed]\n");
        return;
    }

    mode = (rt_uint16_t)strtoul(argv[1], RT_NULL, 0);
    if (mode >= MB_FX_MAX)
    {
        rt_kprintf("[mbfx] bad mode %u (0..%u)\n", mode, (unsigned)(MB_FX_MAX - 1));
        return;
    }
    if (argc >= 3)
    {
        arg = (rt_uint16_t)strtoul(argv[2], RT_NULL, 0);
        if (arg < 1 || arg > 9) arg = 3;
    }

    g_hold[REG_LED_FX]     = mode;
    g_hold[REG_LED_FX_ARG] = arg;

    if (mode == MB_FX_SOLID)
    {
        mb_actuator_apply();   /* 纯色模式由 apply 立刻上色（灯效线程此时不动作） */
        rt_kprintf("[mbfx] fx=0(solid) arg=%u -> applied "
                   "(colour from DO_CTRL 0x%04X / PWM %u)\n",
                   arg, g_hold[REG_DO_CTRL], g_hold[REG_PWM_DUTY]);
    }
    else
    {
        rt_kprintf("[mbfx] fx=%u(%s) arg=%u -> running\n", mode, mb_fx_name[mode], arg);
    }
}
MSH_CMD_EXPORT(mb_fx, mb_fx <mode> [arg] : led matrix effect 0..7);

/* 灯效诊断：确认线程是否真的在跑、寄存器是否被别处改写。
 *
 * 用法：
 *   mb_fx_stat          查看线程句柄 + 当前寄存器 + phase 是否推进
 *   mb_fx_stat probe    临时切到跑马灯测 200ms 活性，然后自动恢复原模式
 *
 * 判读：
 *   thread=0x0                 -> 线程没创建成功
 *   mode=solid + phase 不变    -> **正常**！纯色模式本就不推进 phase（灯由 apply 一次性上色）
 *   mode!=solid + phase 不变   -> 线程卡在 reflash() 的 DMA 等待或被饿死
 *   phase 在变                 -> 线程正常，问题在颜色/亮度/pinmux
 */
static void mb_fx_stat(int argc, char **argv)
{
    rt_uint16_t p1 = 0, p2 = 0, mode, saved_mode, saved_arg;
    (void)argc;
    (void)p1; (void)p2;   /* 宏关闭时这两个变量不会被用到，避免 unused 警告 */

#ifdef BSP_USING_ONBOARD_LED_MATRIX
    rt_thread_t t = rt_thread_find("mbfx");
    /* 用标准 strcmp：本工程 RT-Thread 4.1.1 没有 RT_STRCMP 宏，
       写了会得到 implicit declaration + 链接期 undefined reference */
    rt_bool_t  probe = (argc >= 2) && (strcmp(argv[1], "probe") == 0);

    mode       = g_hold[REG_LED_FX];
    saved_mode = mode;
    saved_arg  = g_hold[REG_LED_FX_ARG];
    if (saved_arg < 1 || saved_arg > 9) saved_arg = 3;

    rt_kprintf("[mbfx] thread=%p  ONBOARD_LED_MATRIX=ON\n", (void *)t);
    if (t == RT_NULL)
    {
        rt_kprintf("[mbfx] thread MISSING -> fx will never animate\n");
        return;
    }
    rt_kprintf("[mbfx] reg: fx=%u arg=%u | do_ctrl=0x%04X pwm=%u\n",
               mode, g_hold[REG_LED_FX_ARG],
               g_hold[REG_DO_CTRL], g_hold[REG_PWM_DUTY]);

    /* probe：先切到跑马灯，否则纯色模式下 phase 恒不变无法判断活性 */
    if (probe)
    {
        g_hold[REG_LED_FX]     = MB_FX_CHASE;
        g_hold[REG_LED_FX_ARG] = 3;
        mode = MB_FX_CHASE;
        rt_thread_mdelay(30);          /* 越过 1~2 个节拍，确认线程已开始消费 */
        rt_kprintf("[mbfx] probe: temporarily switched to chase(3), speed=3\n");
    }

    p1 = g_fx_phase;
    rt_thread_mdelay(150);            /* 跨过 ~7 个灯效节拍 */
    p2 = g_fx_phase;

    if (p1 != p2)
    {
        rt_kprintf("[mbfx] phase: %u -> %u  (ADVANCING, thread alive & unblocked)\n", p1, p2);
    }
    else
    {
        if (mode == MB_FX_SOLID)
            rt_kprintf("[mbfx] phase: %u -> %u  (STATIC - EXPECTED in solid mode;\n"
                       "       solid colour is painted once by mb_actuator_apply, not animated)\n"
                       "       run 'mb_fx 1'..'mb_fx 7' to see animation, or 'mb_fx_stat probe'\n", p1, p2);
        else
            rt_kprintf("[mbfx] phase: %u -> %u  (FROZEN while fx=%s -> reflash() blocking or starved)\n",
                       p1, p2, mb_fx_name[mode]);
    }

    if (probe)
    {
        g_hold[REG_LED_FX]     = saved_mode;
        g_hold[REG_LED_FX_ARG] = saved_arg;
        if (saved_mode == MB_FX_SOLID) mb_actuator_apply();
        rt_kprintf("[mbfx] probe done, restored fx=%u\n", saved_mode);
    }
#else
    (void)argv;
    rt_kprintf("[mbfx] ONBOARD_LED_MATRIX=OFF -> fx code compiled out, nothing will animate\n");
#endif
}
MSH_CMD_EXPORT(mb_fx_stat, mb_fx_stat [probe] : diagnose led fx thread state);

/* ------------------ RS485 透传网关 msh 命令（第 7 步） ------------------ */
#ifdef BSP_USING_RS485
/* mb_rs485 <addr> <func> <reg> [qty|value]
 *   func=3 读：qty 为寄存器个数
 *   func=6 写单：value 为要写的值
 * 例：mb_rs485 1 3 0x10 2      -> 读从机1 的 0x10 起 2 个寄存器（温度/湿度）
 *     mb_rs485 1 6 0x20 100    -> 往从机1 的 0x20 写 100
 * 内部自动组帧 + 算 CRC + 触发网关 + 等结果并打印应答，省去手工填寄存器。 */
static void mb_rs485(int argc, char **argv)
{
    rt_uint16_t addr, func, reg, a3, i, len, st;
    rt_uint8_t  frame[16];

    if (argc < 4)
    {
        rt_kprintf("usage: mb_rs485 <addr> <func> <reg> [qty|value]\n");
        rt_kprintf("  e.g. mb_rs485 1 3 0x10 2   (read 2 regs from slave 1)\n");
        rt_kprintf("       mb_rs485 1 6 0x20 100 (write 100 to reg 0x20)\n");
        return;
    }

    addr = (rt_uint16_t)strtoul(argv[1], RT_NULL, 0);
    func = (rt_uint16_t)strtoul(argv[2], RT_NULL, 0);
    reg  = (rt_uint16_t)strtoul(argv[3], RT_NULL, 0);
    a3   = (argc >= 5) ? (rt_uint16_t)strtoul(argv[4], RT_NULL, 0) : 1;

    if (addr < 1 || addr > 247) { rt_kprintf("bad addr %u (1..247)\n", addr); return; }
    if (func != 3 && func != 6 && func != 16)
    { rt_kprintf("func must be 3/6/16\n"); return; }

    /* 组帧（Modbus RTU：地址 功能码 地址H 地址L 数据... CRC低 CRC高） */
    frame[0] = (rt_uint8_t)addr;
    frame[1] = (rt_uint8_t)func;
    frame[2] = (rt_uint8_t)(reg >> 8);
    frame[3] = (rt_uint8_t)(reg & 0xFF);
    if (func == 3)
    {
        if (a3 < 1 || a3 > 125) { rt_kprintf("qty must be 1..125\n"); return; }
        frame[4] = 0;
        frame[5] = (rt_uint8_t)a3;
        len = 6;
    }
    else if (func == 6)
    {
        frame[4] = (rt_uint8_t)(a3 >> 8);
        frame[5] = (rt_uint8_t)(a3 & 0xFF);
        len = 6;
    }
    else /* 16 */
    {
        frame[4] = 0;
        frame[5] = (rt_uint8_t)a3;
        frame[6] = (rt_uint8_t)(a3 * 2);
        frame[7] = 0; frame[8] = 0;   /* 占位：具体值由调用方预填，这里仅示意 */
        len = 9;
    }

    /* 写入请求区：每个寄存器高低字节拆开放，FC16 由主机填实际值 */
    g_hold[REG_RS485_DATA + 0] = frame[0];
    g_hold[REG_RS485_DATA + 1] = frame[1];
    g_hold[REG_RS485_DATA + 2] = frame[2];
    g_hold[REG_RS485_DATA + 3] = frame[3];
    g_hold[REG_RS485_DATA + 4] = frame[4];
    g_hold[REG_RS485_DATA + 5] = frame[5];
    if (len > 6)
    {
        g_hold[REG_RS485_DATA + 6] = frame[6];
        g_hold[REG_RS485_DATA + 7] = frame[7];
        g_hold[REG_RS485_DATA + 8] = frame[8];
    }

    g_hold[REG_RS485_ADDR] = addr;
    g_hold[REG_RS485_FUNC] = func;
    g_hold[REG_RS485_QTY]  = a3;
    g_hold[REG_RS485_OUTLEN] = len;      /* 网关会按此长度取字节并补 CRC */
    g_hold[REG_RS485_ST]    = RS485_ST_IDLE;
    g_hold[REG_RS485_CTRL]  = 1;         /* 触发 */

    /* 等网关跑完（最长 ~1s） */
    for (i = 0; i < 200; i++)
    {
        if (g_hold[REG_RS485_ST] != RS485_ST_BUSY && g_hold[REG_RS485_ST] != RS485_ST_IDLE)
            break;
        rt_thread_mdelay(5);
    }

    st = g_hold[REG_RS485_ST];
    {
        static const char *sn[] = {"idle", "busy", "ok", "timeout", "crc-err", "toolong"};
        rt_kprintf("[rs485] %s  tx=%d rx=%d\n",
                   (st <= 5) ? sn[st] : "?", g_hold[REG_RS485_OUTLEN], g_hold[REG_RS485_INLEN]);
    }

    if (st == RS485_ST_OK)
    {
        rt_uint16_t n = g_hold[REG_RS485_INLEN];
        rt_kprintf("[rs485] rsp:");
        for (i = 0; i < n; i++)
            rt_kprintf(" %02X", (rt_uint8_t)(g_hold[REG_RS485_DATA + i] & 0xFFu));
        rt_kprintf("\n");
        /* FC03 应答：第 3 字节是字节数，之后每 2 字节一个寄存器 */
        if (func == 3 && n >= 3)
        {
            rt_uint8_t nb = (rt_uint8_t)(g_hold[REG_RS485_DATA + 2] & 0xFF);
            rt_kprintf("[rs485] decoded:");
            for (i = 0; (i * 2 + 1) < nb && (rt_uint16_t)(4 + i * 2) < RS485_BUF_MAX; i++)
            {
                rt_uint16_t v = (rt_uint16_t)((g_hold[REG_RS485_DATA + 3 + i * 2] & 0xFF) << 8 |
                                              (g_hold[REG_RS485_DATA + 4 + i * 2] & 0xFF));
                rt_kprintf(" [%u]=%u", reg + i, v);
            }
            rt_kprintf("\n");
        }
    }
}
MSH_CMD_EXPORT(mb_rs485, mb_rs485 <addr> <func> <reg> [qty|val] : RS485 passthrough);

/* mb_rs485_lb [0 | 1 | crc]
 *   0    关自测，走真实 RS485（接现场设备时必须关）
 *   1    开自测，用本地合成的假从站应答（不接任何设备也能验软件链路）
 *   crc  开自测 + 故意打坏 CRC，用来确认「错误分支」真的会报错，而不是永远 ok
 *
 * 典型验收序列（现场没设备时）：
 *   mb_rs485_lb 1      -> mb_rs485 1 3 0x10 2   期望 ok，解码出 [16]=256 [17]=257
 *   mb_rs485_lb crc    -> mb_rs485 1 3 0x10 2   期望 crc-err
 *   mb_rs485_lb 0      -> mb_rs485 1 3 0x10 2   空总线上期望 timeout（验的是真实链路已发出） */
static void mb_rs485_lb(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("[rs485] loopback=%d badcrc=%d  (usage: mb_rs485_lb [0|1|crc])\n",
                   (int)g_rs485_loopback, (int)g_rs485_lb_badcrc);
        return;
    }

    if (strcmp(argv[1], "crc") == 0)
    {
        g_rs485_loopback = 1;
        g_rs485_lb_badcrc = 1;
    }
    else
    {
        g_rs485_loopback = (rt_uint8_t)(strtoul(argv[1], RT_NULL, 0) ? 1 : 0);
        g_rs485_lb_badcrc = 0;
    }

    rt_kprintf("[rs485] loopback=%d badcrc=%d\n",
               (int)g_rs485_loopback, (int)g_rs485_lb_badcrc);
}
MSH_CMD_EXPORT(mb_rs485_lb, mb_rs485_lb [0|1|crc] : RS485 loopback self-test);
#endif /* BSP_USING_RS485 */

