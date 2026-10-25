/*
 * 云端上传：星火一号把采集值经 RW007 上报到云端
 *
 * 链路： mb_slave 采集(500ms) -> g_hold[] -> mb_snapshot_get()
 *        -> 本线程每 5s 取一份摘要 -> 环形缓存 -> HTTP POST /api/v1/ingest
 *
 * 设计要点：
 *   1. **断网补传**：采样无条件入环形缓存，只有收到云端 200 才出队。
 *      断网期间最多缓存 CLOUD_RING_SIZE 条（默认 32 条 ≈ 2.5 分钟），
 *      恢复后按 FIFO 分批补回，所以云端看到的时间序列里会有一条「迟到的补传段」。
 *   2. **不阻塞采集**：网络是慢 I/O，全在本线程（低优先级）里做；
 *      socket 带收发超时，AP 断线时 connect 不会把线程挂死。
 *   3. **失败可观测**：cloud_stat 打印成功/失败/丢弃计数与最后一次 HTTP 状态码，
 *      现场排障先看它，不用抓包。
 *
 * 依赖：BSP_USING_RW007_WLAN（WiFi）、RT_USING_SAL + lwIP（socket）。
 * 没勾 RW007 时本文件整段不编译。
 */

#include <rtthread.h>

#ifdef BSP_USING_RW007_WLAN

#if !defined(RT_USING_SAL)
#error "云端上传需要 SAL：RT-Thread Settings -> RT-Thread Components -> Network -> Socket abstraction layer（同时开启 lwIP）"
#endif

#include <rtdevice.h>
#include <wlan_mgnt.h>      /* rt_wlan_connect / rt_wlan_is_ready */
#include <sys/socket.h>
#include <sys/time.h>       /* struct timeval（setsockopt 收发超时） */
#include <netdb.h>          /* gethostbyname */
#include <arpa/inet.h>      /* htons */
#include <stdio.h>
#include <stdlib.h>         /* atoi */
#include <string.h>

#include "cloud_config.h"
#include "mb_slave_data.h"

#define DBG_TAG "cloud"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* ---------------- 环形缓存（断网期间的数据） ---------------- */
static mb_snapshot_t g_ring[CLOUD_RING_SIZE];
static mb_d12x_snap_t g_ring_d12x[CLOUD_RING_SIZE];   /* 与 g_ring 同节拍的 D12x 中转数据 */
static rt_uint16_t   g_head;          /* 下一个写入位置 */
static rt_uint16_t   g_count;         /* 尚未被云端确认的条数 */

static char          g_txbuf[CLOUD_TXBUF_SIZE];   /* 组包用，放全局避免撑爆线程栈 */

static struct
{
    rt_uint32_t sent_ok;              /* 云端已确认的采样条数 */
    rt_uint32_t sent_fail;            /* 发送失败次数（按批计） */
    rt_uint32_t dropped;              /* 缓存满被丢掉的旧采样 */
    int         last_code;            /* 最后一次 HTTP 状态码，0 = 还没成功过 */
    rt_uint8_t  linked;               /* 当前是否在线 */
} g_stat;

/* ---------------- 环形缓存操作 ---------------- */
/* 两个 ring 同节拍写入当前 g_head 位置；g_head/g_count 的推进由主循环统一做 */
static void ring_set(rt_uint16_t idx, const mb_snapshot_t *s)
{
    g_ring[idx] = *s;
}
static void ring_set_d12x(rt_uint16_t idx, const mb_d12x_snap_t *s)
{
    g_ring_d12x[idx] = *s;
}

/* 取第 i 条待发送数据（0 = 最旧），用于按 FIFO 补传 */
static mb_snapshot_t *ring_peek(rt_uint16_t i)
{
    rt_uint16_t start = (rt_uint16_t)((g_head + CLOUD_RING_SIZE - g_count) % CLOUD_RING_SIZE);
    return &g_ring[(start + i) % CLOUD_RING_SIZE];
}

static mb_d12x_snap_t *ring_peek_d12x(rt_uint16_t i)
{
    rt_uint16_t start = (rt_uint16_t)((g_head + CLOUD_RING_SIZE - g_count) % CLOUD_RING_SIZE);
    return &g_ring_d12x[(start + i) % CLOUD_RING_SIZE];
}

static void ring_drop(rt_uint16_t n)
{
    g_count = (rt_uint16_t)(g_count - n);
}

/* ---------------- JSON 组包 ---------------- */
/*
 * 单条（云端按 0.1 定点还原，字段扁平化以匹配 server METRICS 的 field 名）：
 *   {"up":123,"hb":456,"err":0,"temp":275,"humi":388,"acc_x":-7260,"gyro_x":446}
 * 注：只上报实际有数据的指标（温度/湿度/加速度X/角速度X），删去 lux/ps 及 Y/Z 轴。
 *     acc_x/gyro_x 必须用扁平字段名，否则 server 读到 0 -> overview 显示「无数据」。
 * 批量：{"device":"spark1","samples":[{...},{...}]}
 */
static int build_batch(rt_uint16_t n)
{
    int len = 0;
    int left = CLOUD_TXBUF_SIZE;
    rt_uint16_t i;

    len = snprintf(g_txbuf, left, "{\"device\":\"%s\",\"samples\":[", CLOUD_DEVICE_ID);
    if (len < 0 || len >= left) return -1;

    for (i = 0; i < n; i++)
    {
        mb_snapshot_t *s = ring_peek(i);
        int w = snprintf(g_txbuf + len, left - len,
                         "%s{\"up\":%u,\"hb\":%u,\"err\":%u,\"temp\":%d,\"humi\":%d,"
                         "\"acc_x\":%d,\"gyro_x\":%d}",
                         (i == 0) ? "" : ",",
                         (unsigned)s->uptime, (unsigned)s->heartbeat, (unsigned)s->err,
                         (int)s->temp, (int)s->humi,
                         (int)s->acc_x, (int)s->gyro_x);
        if (w < 0 || w >= left - len) return -1;
        len += w;
    }

    {
        int w = snprintf(g_txbuf + len, left - len, "]}");
        if (w < 0 || w >= left - len) return -1;
        len += w;
    }
    return len;
}

/* D12x 中转数据批量：device="d12x"，字段对应云端 METRICS 的 d12x_* 条目 */
static int build_batch_d12x(rt_uint16_t n)
{
    int len = 0;
    int left = CLOUD_TXBUF_SIZE;
    rt_uint16_t i;

    len = snprintf(g_txbuf, left, "{\"device\":\"%s\",\"samples\":[", CLOUD_DEVICE_ID_D12X);
    if (len < 0 || len >= left) return -1;

    for (i = 0; i < n; i++)
    {
        mb_d12x_snap_t *s = ring_peek_d12x(i);
        int w;
        /* 电量 0.1%：未接电池/电源供电时 D12x 直接报 100%(=1000)，恒有值 */
        w = snprintf(g_txbuf + len, left - len,
                     "%s{\"up\":%u,\"temp\":%d,\"bat\":%u,\"cpu\":%u,\"mem\":%u}",
                     (i == 0) ? "" : ",",
                     (unsigned)s->uptime, (int)s->temp,
                     (unsigned)s->bat, (unsigned)s->cpu, (unsigned)s->mem);
        if (w < 0 || w >= left - len) return -1;
        len += w;
    }

    {
        int w = snprintf(g_txbuf + len, left - len, "]}");
        if (w < 0 || w >= left - len) return -1;
        len += w;
    }
    return len;
}

/* ---------------- HTTP POST（SAL 阻塞 socket） ---------------- */
static int http_post(const char *body, int body_len)
{
    struct hostent    *he;
    struct sockaddr_in addr;
    struct timeval     tv;
    char               hdr[256];
    int                s;
    int                head_len;
    int                ret;
    int                code = -1;

    he = gethostbyname(CLOUD_HOST);
    if (he == RT_NULL || he->h_addr_list[0] == RT_NULL)
    {
        LOG_E("dns fail: %s", CLOUD_HOST);
        return -1;
    }

    s = socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0)
    {
        LOG_E("socket fail");
        return -2;
    }

    tv.tv_sec  = CLOUD_SOCK_TIMEOUT_S;
    tv.tv_usec = 0;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(CLOUD_PORT);
    memcpy(&addr.sin_addr, he->h_addr_list[0], sizeof(addr.sin_addr));

    if (connect(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        LOG_E("connect fail: %s:%d", CLOUD_HOST, CLOUD_PORT);
        closesocket(s);
        return -3;
    }

    head_len = snprintf(hdr, sizeof(hdr),
                        "POST %s HTTP/1.1\r\n"
                        "Host: %s:%d\r\n"
                        "Content-Type: application/json\r\n"
                        "Content-Length: %d\r\n"
                        "Connection: close\r\n\r\n",
                        CLOUD_PATH, CLOUD_HOST, CLOUD_PORT, body_len);
    if (head_len < 0 || head_len >= (int)sizeof(hdr))
    {
        closesocket(s);
        return -4;
    }

    if (send(s, hdr, head_len, 0) != head_len)
    {
        LOG_E("send head fail");
        closesocket(s);
        return -5;
    }

    /* 大包可能分片，循环发完 */
    {
        int off = 0;
        while (off < body_len)
        {
            ret = send(s, body + off, body_len - off, 0);
            if (ret <= 0)
            {
                LOG_E("send body fail (%d)", ret);
                closesocket(s);
                return -6;
            }
            off += ret;
        }
    }

    /* 只读状态行，body 不关心 */
    ret = recv(s, hdr, sizeof(hdr) - 1, 0);
    if (ret > 0)
    {
        char *p;
        hdr[ret] = '\0';
        p = strstr(hdr, "HTTP/1.1 ");
        if (p == RT_NULL) p = strstr(hdr, "HTTP/1.0 ");
        if (p != RT_NULL) code = atoi(p + 9);
    }
    else
    {
        LOG_E("recv fail (%d)", ret);
    }

    closesocket(s);
    return (code == 200) ? 0 : -7;
}

/* ---------------- WiFi 连接 ---------------- */
static void ensure_wifi(void)
{
    int i;

    if (!rt_wlan_is_connected())
    {
        LOG_I("connecting to %s ...", CLOUD_WIFI_SSID);
        rt_wlan_connect(CLOUD_WIFI_SSID, CLOUD_WIFI_PWD);
    }

    /* rt_wlan_is_ready() = 已连接且拿到 IP */
    for (i = 0; i < CLOUD_WIFI_WAIT_SEC * 2; i++)
    {
        if (rt_wlan_is_ready())
        {
            return;
        }
        rt_thread_mdelay(500);
    }
    LOG_W("wifi not ready after %ds", CLOUD_WIFI_WAIT_SEC);
}

/* ---------------- 上传线程 ---------------- */
static void cloud_upload_entry(void *param)
{
    (void)param;

    rt_thread_mdelay(3000);   /* 等 wifi_spi_device_init() 把设备注册上来 */

    while (1)
    {
        mb_snapshot_t  snap;
        mb_d12x_snap_t snap_d12x;
        rt_uint16_t    idx;

        if (!rt_wlan_is_ready())
        {
            g_stat.linked = 0;
            ensure_wifi();
        }

        /* 采样：spark1 与 d12x 同节拍各写一条（D12x 经 Modbus 把本板数据放进中转区），
           再统一推进环形缓存。断网期间两条队列的数据都留着，恢复后按 FIFO 补传 */
        mb_snapshot_get(&snap);
        mb_d12x_get(&snap_d12x);
        idx = g_head;
        ring_set(idx, &snap);
        ring_set_d12x(idx, &snap_d12x);
        g_head = (rt_uint16_t)((g_head + 1) % CLOUD_RING_SIZE);
        if (g_count < CLOUD_RING_SIZE)
        {
            g_count++;
        }
        else
        {
            g_stat.dropped++;
        }

        if (rt_wlan_is_ready())
        {
            rt_uint16_t n = g_count;

            g_stat.linked = 1;
            if (n == 0)
            {
                /* 没积压就发最新一条 */
                n = 1;
            }
            else if (n > CLOUD_BATCH_MAX)
            {
                n = CLOUD_BATCH_MAX;
            }

            {
                /* 两个设备各发一批，必须都成功才出队（避免单设备重复补传）。
                 * 注意：build_batch / build_batch_d12x 共用 g_txbuf，所以必须先发完
                 * spark1 再构建 d12x，否则后一次 build 会覆盖前一次要发的报文。 */
                int l1 = build_batch(n);
                if (l1 > 0 && http_post(g_txbuf, l1) == 0)
                {
                    int l2 = build_batch_d12x(n);
                    if (l2 > 0 && http_post(g_txbuf, l2) == 0)
                    {
                        ring_drop(n);
                        g_stat.sent_ok += n;
                        g_stat.last_code = 200;
                        LOG_D("upload spark1+d12x %u sample(s), cached %u", (unsigned)n, (unsigned)g_count);
                    }
                    else
                    {
                        g_stat.sent_fail++;
                        LOG_W("upload failed, cached %u", (unsigned)g_count);
                    }
                }
                else
                {
                    g_stat.sent_fail++;
                    LOG_W("upload failed, cached %u", (unsigned)g_count);
                }
            }
        }
        else
        {
            g_stat.linked = 0;
        }

        rt_thread_mdelay(CLOUD_PERIOD_MS);
    }
}

int cloud_upload_init(void)
{
    rt_thread_t tid;

    tid = rt_thread_create("cloud", cloud_upload_entry, RT_NULL,
                           CLOUD_THREAD_STACK, CLOUD_THREAD_PRIO, CLOUD_THREAD_TICK);
    if (tid == RT_NULL)
    {
        LOG_E("create cloud thread failed");
        return -RT_ERROR;
    }
    rt_thread_startup(tid);
    rt_kprintf("[cloud] upload task start: %s:%d%s, period %dms\n",
               CLOUD_HOST, CLOUD_PORT, CLOUD_PATH, CLOUD_PERIOD_MS);
    return RT_EOK;
}
INIT_APP_EXPORT(cloud_upload_init);

/* ---------------- 诊断命令 ---------------- */
static void cloud_stat(int argc, char **argv)
{
    rt_kprintf("[cloud] server  : %s:%d%s\n", CLOUD_HOST, CLOUD_PORT, CLOUD_PATH);
    rt_kprintf("[cloud] wifi    : %s, ready=%d\n",
               CLOUD_WIFI_SSID, rt_wlan_is_ready());
    rt_kprintf("[cloud] sent    : %u ok, %u batch fail, last http=%d\n",
               (unsigned)g_stat.sent_ok, (unsigned)g_stat.sent_fail, g_stat.last_code);
    rt_kprintf("[cloud] cached  : %u (max %u), dropped %u\n",
               (unsigned)g_count, CLOUD_RING_SIZE, (unsigned)g_stat.dropped);
}
MSH_CMD_EXPORT(cloud_stat, cloud upload status);

#endif /* BSP_USING_RW007_WLAN */
