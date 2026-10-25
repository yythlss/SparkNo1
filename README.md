# 便携式工业调试仪 · 从机（星火一号 / STM32F407）

> **RT-Thread 嵌入式电子设计大赛 2026 参赛作品 · 配套从机**
> 基于 RT-Thread 的 Modbus 从站 / 传感器节点 / RS485 网关。
>
> 本仓库为作品配套节点，自研代码位于 [`applications/`](applications/)。

---

## 1. 角色定位

本板是「便携式工业调试仪」系统的**配套从机节点**（主控为另一仓库的 ArtInChip D12x）。
它让手持调试仪在「不接真实工业设备」时也能完整验证整条 Modbus/RS485 链路，并提供：

- **传感器采集**：AHT10（温湿度）、AP3216C（光照）、ICM20608（六轴），500 ms 周期；
- **执行器**：RGB 灯、蜂鸣器、板载 WS2812 灯阵（受 Modbus 寄存器遥控）；
- **Modbus-RTU 从站**：监听 USART3，支持 FC03 / FC06 / FC16，标准 CRC16 应答；
- **RS485 网关**：监听透传寄存器，自管 DE 方向收发，把第三方 RS485 设备应答回填；
- **云端上报**：经板载 RW007（WiFi）把 5 s 摘要 HTTP POST 到云端服务，供小程序查看。

> 配套主控仓库：<https://github.com/yythlss/d12x>

---

## 2. 硬件平台

| 项 | 规格 |
|---|---|
| MCU | STM32F407（星火一号开发板） |
| 操作系统 | RT-Thread（内核 / 设备框架 / 信号量 / 邮箱 / FinSH） |
| 通信 | USART3（Modbus-RTU 从站，115200/8N1）、RS485（半双工，DE 方向自管） |
| WiFi | 板载 RW007（SPI2），用于云端上报 |
| 传感器 | AHT10、AP3216C、ICM20608（软件 I2C 直读，未注册为 sensor 框架 device） |
| 执行器 | RGB / 蜂鸣器 / WS2812 灯阵 |

**与主控（D12x）接线**（TTL 直连，115200/8N1，**双方均为 3.3V TTL，严禁 5V 模块**）：

| 星火一号 | D12x（主控 uart1） | 说明 |
|---|---|---|
| PB10 (TX) | PA3 (RX) | 从机发 → 主控收 |
| PB11 (RX) | PA2 (TX) | 从机收 ← 主控发 |
| GND | GND | 共地 |

---

## 3. 自研代码目录

自研代码在 [`applications/`](applications/)（**7 个文件**）：

| 文件 | 职责 |
|---|---|
| `mb_slave.c` | Modbus-RTU 从站核心：采集线程 + 协议线程 + 执行器/灯效线程 + RS485 网关线程 |
| `mb_slave_data.h` | 寄存器地图（保持寄存器数组、心跳、运行秒数、错误位定义） |
| `cloud_upload.c` | 周期上报本机传感器与中转 D12x 遥测（环形缓存 + HTTP POST） |
| `cloud_config.h` | **云端配置集中在此**：WiFi SSID/密码、HTTP 端点、device id |
| `rw007_port.c` | 板载 RW007（SPI2）WiFi 移植 |
| `slave_check.c` | 脱机自测命令（`sensor_check` / `mb_dump` / `mb_do` / `mb_fx` / `mb_rs485_lb` 等） |
| `SConscript` | 构建脚本，登记上述源文件 |

> 改云端配置（SSID、URL、device id）**只动 `cloud_config.h`**，无需改动逻辑代码。

---

## 4. 线程模型（从机）

| 线程 | 优先级 | 职责 |
|---|---|---|
| 采集线程 | — | 500 ms 读取三传感器，写入保持寄存器并维护心跳/运行秒数/错误位 |
| 协议线程 | — | 监听 USART3，解析 Modbus-RTU 请求，执行 FC03/06/16，CRC16 应答 |
| 执行器线程 | — | 依据寄存器驱动 RGB / 蜂鸣器 |
| 灯效引擎线程 | — | 20 ms 节拍，8 种动画（受寄存器遥控） |
| RS485 网关线程 | — | 监听透传寄存器、自管 DE 收发、回填应答 |
| 云端上传线程 | — | 5 s 摘要 HTTP POST（RW007） |

---

## 5. 快速开始

### 5.1 构建（RT-Thread Studio，推荐）

1. 用 RT-Thread Studio 打开本工程（`project.uvprojx` 或 Studio 工程）；
2. 若 `rt-thread` / `libraries` 为符号链接缺失，先以管理员运行 `mklinks.bat`；
3. 编译、连接开发板、下载。

### 5.2 构建（scons 命令行）

```bat
:: 确保 rt-thread / libraries 已就位（见上）
scons
```

### 5.3 运行与 console

USB-Serial 接 uart1（115200/8N1）作为 console，上电后可用 FinSH 命令：

```shell
msh >sensor_check      # 三传感器读数
msh >mb_dump           # 寄存器心跳/传感器值
msh >mb_do 0x000F 500  # 执行器：RGB+蜂鸣器，占空比 500
msh >list_thread       # 查看线程
```

---

## 6. 脱机自测（不连主控即可验证）

> 从机独立 5V 供电，USB-Serial 接 uart1 作 console。

| 命令 | 预期 | 说明 |
|---|---|---|
| `list_device` | uart1/uart3/uart6、pwm2/pwm3 注册 | 验证 UART3 与真实 PWM 已编入固件 |
| `sensor_check` | AHT10/AP3216C/ICM20608 读数 | 温湿度/光照/六轴 |
| `mb_dump` | 心跳递增、err=0 | 寄存器地图自检 |
| `mb_do 0x000F 500` | RGB 三色亮、蜂鸣响 | bit3(0x08)=蜂鸣 |
| `mb_fx 2 3` + `mb_fx_stat` | 灯效线程活跃 | 动画引擎 |
| `mb_rs485_lb 1` → `mb_rs485 1 3 0x10 2` | 应答 `01 03 04 01 00 01 01 3B 9F` | RS485 回环 |
| `mb_rs485_lb 0` → `mb_rs485 1 3 0x10 2` | timeout（空总线，证明确实发帧） | 真实总线 |

> 双板联调前用 `mb_rs485_lb 1` 切回回环；联调时 `mb_rs485_lb 0` 切真实总线。
> `list_device` 看不到 aht10 等传感器设备属正常——三传感器走软件 I2C 直读。

---

## 7. 开源与许可

- 本作品自研代码（`applications/` 下上述文件）以 Apache License 2.0 开源，各源文件带 SPDX 头；
- RT-Thread 内核及星火一号 BSP/库遵循其各自开源协议。

---

欢迎在 Issues 中交流，也欢迎在 RT-Thread 官方论坛的参赛帖下拍砖。
