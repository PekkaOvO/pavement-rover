# GD32 Image Bridge — 系统设计文档

## 概述

GD32开发板通过USB-TTL向i.MX6ULL Linux开发板发送JPEG图片和摄像机角度数据；Linux板运行GPS路径控制程序（gnss_path_control），将载体状态与图片数据通过TCP发送到上位机CarView2实时显示和存储。

## 系统架构

```
GD32 ──USB-TTL@921600──► i.MX6ULL ──TCP:8765──► CarView2(PC)
                                ▲
                        GPS状态 (Unix域Socket)
                                │
                        gnss_path_control
```

### 组件

| 组件 | 位置 | 语言 | 说明 |
|------|------|------|------|
| `gd32_comm` (驱动) | `Drivers/Linux_Drivers/gd32_comm/` | C | 内核misc字符设备驱动，系统状态/控制接口 |
| `gd32_bridge` (守护进程) | `C_APP/pwm/KF-GINS-main/gd32_bridge/` | C++ | 收USB-TTL→帧同步→收GPS状态→TCP发送 |
| `gnss_path_control` (修改) | `KF-GINS-main/src/path_control/` | C++ | 新增Unix域socket推送GPS状态 |
| `CarView2` (改造) | `QtProjects/CarView2/` | C++/Qt6 | 新增TCP收数、图片显示、文件存储 |

---

## 1. USB-TTL通信协议 (GD32 → i.MX6ULL)

### 物理层
- 接口：USB-TTL (CH340/CP210x)
- 波特率：921600 bps, 8N1
- 设备节点：`/dev/ttyUSB0`

### 数据帧格式

```
| MAGIC(2B) | TYPE(1B) | LEN(4B) | PAYLOAD(NB) | CRC16(2B) |
```

| 字段 | 大小 | 说明 |
|------|------|------|
| MAGIC | 2B | `0xAA 0x55` — 帧同步头 |
| TYPE | 1B | `0x01`=图+角度 , `0x02`=仅角度 |
| LEN | 4B | 大端uint32, PAYLOAD长度 |
| PAYLOAD | NB | TYPE=0x01: JPEG数据(末尾4B附摄像机角度float)；TYPE=0x02: 4B float角度 |
| CRC16 | 2B | CRC16-CCITT (MAGIC~PAYLOAD) |

### 注意事项
- 图片帧预计~128KB，@921600约1.4秒传完
- 角度帧仅4B，可随时插入
- 接收方使用MAGIC进行字节流帧同步

---

## 2. 内核驱动 `gd32_comm`

### 功能
- misc字符设备 `/dev/gd32_comm`
- 提供系统运行状态查询（收包计数、错误统计、运行时长）
- 提供IOCTL接口：查询/重置
- 提供poll/select事件通知

### 文件列表
```
gd32_comm/
├── Makefile
└── gd32_comm.c
```

### 设备接口
- `open()` — 打开设备
- `read()` — 读取状态字符串
- `ioctl()` — `GD32_IOC_GET_STATUS` 查询内部计数

---

## 3. 桥接守护进程 `gd32_bridge`

### 功能
- 打开 `/dev/ttyUSB0` 设置TIOCEXCL独占锁
- 线程A（UART读取）：读原始字节 → 帧同步 → CRC校验 → 完整帧压入RingBuffer
- 线程B（GPS接收）：连接Unix域socket，接收最新GPS状态（原子变量缓存）
- 线程C（TCP发送）：从RingBuffer取帧 + 最新GPS状态 → 打包二进制协议 → TCP发送

### 二进制TCP协议 (Linux板 → 上位机)

```
| MAGIC(2B) | VERSION(1B) | TYPE(1B) | SEQ(4B) | TS_MS(8B) |
| LAT(8B) | LON(8B) | COURSE(4B) | SPEED(4B) | HEIGHT(4B) |
| CAM_ANGLE(4B) | IMAGE_LEN(4B) | [IMAGE_DATA(IMAGE_LEN)] | CRC16(2B) |
```

- 固定头部50字节，后接可选JPEG数据
- 全部多字节字段使用大端序（网络字节序）
- TYPE=0x01时有图片，TYPE=0x02时无图片（IMAGE_LEN=0）
- CRC16校验VERSION到IMAGE_DATA尾

### 文件列表
```
gd32_bridge/
├── CMakeLists.txt
├── main.cpp          — 入口、线程管理
├── uart_reader.h/.cpp   — USB-TTL帧同步+CRC
├── gps_receiver.h/.cpp  — Unix域socket接收GPS
├── tcp_sender.h/.cpp    — TCP打包发送
├── protocol.h           — 协议结构体定义
└── ring_buffer.h        — 线程安全环形队列
```

---

## 4. GPS状态 共享方案

### 方式
- gnss_path_control启动时创建一个Unix域socket（路径 `/tmp/gd32_gps.sock`）
- 每次`printStatus()`调用时，同时将`GnssPosition`和`RobotState`通过socket发布
- bridge守护进程连接同一socket，接收最新状态

### 修改内容
- `gnss_path_control_main.cpp`: 新增 `GpsPublisher` 类，在主循环中推送状态
- 新增文件 `gps_publisher.h/.cpp`

---

## 5. CarView2 改造

### 新增组件

| 文件 | 说明 |
|------|------|
| `comms/tcpreceiver.h/.cpp` | TCP客户端，接收二进制协议数据 |
| `ui/imagedisplaywidget.h/.cpp` | 图片显示Dock Widget |
| `storage/rollingfilestorage.h/.cpp` | 状态日志滚动文件存储 |
| `storage/imagebuffer.h/.cpp` | 保存最近20张图片到文件 |

### 功能需求

**TCP接收**：
- 主动连接Linux板（可配置IP:Port）
- 解析二进制协议，分发给其他组件
- 断线自动重连

**图片显示**：
- 主界面右侧新增Dock，显示最新图片
- 显示拍照时间戳、摄像机角度

**滚动文件存储**：
- 状态信息写入CSV文件（默认上限50MB/个）
- 始终保留最新2个文件：
  - 写 `state_0.csv` 至满 → 新建 `state_1.csv`
  - `state_1.csv` 满 → 删 `state_0.csv`，建 `state_2.csv`
  - `state_2.csv` 满 → 删 `state_1.csv`，建 `state_3.csv`
  - 依次类推，始终只有最近2个文件

**图片文件缓冲**：
- 每收到一张带图片的包，将JPEG保存到 `images/` 目录
- 文件命名 `img_{seq}_{timestamp}.jpg`
- 目录内保持最近20张，超出则删除最旧的
- 异常记录时引用对应的图片ID

**异常记录**：
- 程序运行日志保存到 `error.log`
- 异常或错误发生时，保留现场图片（计入20张限额，但加锁防止被删除）
- 保存异常记录到 `anomalies.log`

---

## 6. 性能考虑

- 所有I/O使用非阻塞模式 + select/poll
- 线程间通信使用无锁环形队列
- TCP发送线程使用独立socket，不受UART读取阻塞
- JPEG数据零拷贝传递（指针传递而非复制）
- GPS状态通过原子变量缓存，无锁读取

## 7. 错误处理

- USB-TTL断开：bridge守护进程自动重试打开，每2秒重试
- TCP断开：自动重连，指数退避（1s, 2s, 4s, max 30s）
- CRC校验失败：丢弃帧，增加错误计数，继续帧同步
- GPS无数据：bridge发送状态包时标记GPS无效字段
