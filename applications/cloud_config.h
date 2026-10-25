/*
 * 云端上传配置（改这里就能换 WiFi 和服务器，不用动逻辑代码）
 *
 * 运行前必看：
 *   1. WIFI_SSID / WIFI_PWD 填你家的 2.4G 网络（RW007 不支持 5G）
 *   2. CLOUD_HOST / CLOUD_PORT 填运行 tools/cloud_server/server.py 那台机器的
 *      局域网 IP（连同一个 nova12 热点时就是 192.168.43.100），不要用 localhost —— 那是板子自己
 *   3. 上传周期 5 秒 = 云端「摘要数据」的采样间隔，与小程序趋势页一致
 */

#ifndef __CLOUD_CONFIG_H__
#define __CLOUD_CONFIG_H__

/* ---------------- WiFi ---------------- */
#define CLOUD_WIFI_SSID        "nova12"
#define CLOUD_WIFI_PWD         "yyy123456"
#define CLOUD_WIFI_WAIT_SEC    30        /* 开机后最多等这么久拿到 IP，超时本轮放弃 */

/* ---------------- 云端服务 ---------------- */
/* 注意：服务器必须和 RW007 在同一个 WiFi 网络里（同一网段）。
 * 本工程用手机热点 nova12 当 AP，它给设备分的网段是 192.168.43.x，
 * 所以服务器 PC 也要连 nova12，并设静态 IP 192.168.43.100（同网段）。
 * 若服务器在别的网络，STM32 路由表里没有去那个网段的下一跳，connect 必失败。 */
#define CLOUD_HOST             "192.168.43.100"
#define CLOUD_PORT             8000
#define CLOUD_PATH             "/api/v1/ingest"
#define CLOUD_DEVICE_ID        "spark1"  /* 从站在云端的标识，与小程序设备列表对应 */
#define CLOUD_DEVICE_ID_D12X   "d12x"    /* D12x 主控终端（经本从站中转上报，device 标 d12x） */

/* ---------------- 上传行为 ---------------- */
#define CLOUD_PERIOD_MS        5000      /* 采样/上传周期 */
#define CLOUD_BATCH_MAX        12        /* 断网补传时一次最多带几条（别太大，lwIP 缓冲吃不消） */
#define CLOUD_RING_SIZE        32        /* 断网期间缓存的采样条数（32 × 5s ≈ 2.5 分钟） */
#define CLOUD_SOCK_TIMEOUT_S   5         /* socket 收发超时，卡住时别把线程挂死 */
#define CLOUD_TXBUF_SIZE       2300      /* 发送缓冲：CLOUD_BATCH_MAX 条约 1.9KB，留余量 */

/* ---------------- 线程 ---------------- */
#define CLOUD_THREAD_STACK     2048
#define CLOUD_THREAD_PRIO      22
#define CLOUD_THREAD_TICK      10

#endif /* __CLOUD_CONFIG_H__ */
