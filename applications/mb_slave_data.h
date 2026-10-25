/*
 * 星火一号（从机）数据快照 —— 云端上传模块与 Modbus 从站之间的唯一接口
 *
 * 背景：g_hold[] 是 mb_slave.c 的 static 数组，外部拿不到；
 *      云端上传线程又必须读最新采集值。与其把数组改成全局（会让所有
 *      写寄存器的地方失去封装），不如只暴露一个只读快照函数：
 *      上传侧拿副本，不参与寄存器读写，天然无并发问题（uint16 单次读原子）。
 *
 * 数值语义与寄存器地图一致：
 *   temp/humi/lux 为 0.1 单位的定点值（275 = 27.5℃），
 *   六轴为裸 int16（ICM20608 原始输出）。
 */

#ifndef __MB_SLAVE_DATA_H__
#define __MB_SLAVE_DATA_H__

#include <rtthread.h>

typedef struct mb_snapshot
{
    rt_uint16_t heartbeat;    /* 0x0000 每 500ms 自增，云端据此判活     */
    rt_uint16_t uptime;       /* 0x0001 运行秒数                        */
    rt_uint16_t err;          /* 0x0002 b0 AHT10 / b1 AP3216C / b2 ICM  */
    rt_int16_t  temp;         /* 0x0010 0.1 ℃                           */
    rt_int16_t  humi;         /* 0x0011 0.1 %                           */
    rt_uint16_t lux;          /* 0x0012 0.1 lux                         */
    rt_uint16_t ps;           /* 0x0013 接近值                          */
    rt_int16_t  acc_x;        /* 0x0020                                 */
    rt_int16_t  acc_y;        /* 0x0021                                 */
    rt_int16_t  acc_z;        /* 0x0022                                 */
    rt_int16_t  gyro_x;       /* 0x0023                                 */
    rt_int16_t  gyro_y;       /* 0x0024                                 */
    rt_int16_t  gyro_z;       /* 0x0025                                 */
} mb_snapshot_t;

/* 取一份采集值快照；snap 为空指针时直接返回 */
void        mb_snapshot_get(mb_snapshot_t *snap);

/* 按寄存器地址读单个保持寄存器，越界返回 0（云端上传侧用不到，诊断用） */
rt_uint16_t mb_hold_get(rt_uint16_t addr);

/* D12x 主控终端「经从站中转」上报的本板数据快照
 * （数值语义与 REG_D12X_* 寄存器一致，单位见各字段注释） */
typedef struct mb_d12x_snap
{
    rt_int16_t  temp;       /* 0x0400 SoC 结温 0.1 ℃          */
    rt_uint16_t bat;        /* 0x0401 电量 0.1 %              */
    rt_uint16_t cpu;        /* 0x0402 CPU 占用 0.1 %          */
    rt_uint16_t mem;        /* 0x0403 剩余内存 KB             */
    rt_uint16_t uptime;     /* 0x0404 D12x 运行秒数           */
} mb_d12x_snap_t;

/* 取一份 D12x 中转数据快照；snap 为空指针时直接返回 */
void mb_d12x_get(mb_d12x_snap_t *snap);

#endif /* __MB_SLAVE_DATA_H__ */
