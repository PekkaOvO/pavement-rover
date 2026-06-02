# 道路裂缝检测停车逻辑 — 设计与实现

## 目录

1. [系统整体架构](#1-系统整体架构)
2. [GD32 端 — 裂缝检测与数据发送](#2-gd32-端--裂缝检测与数据发送)
3. [USB CDC 协议详解](#3-usb-cdc-协议详解)
4. [gd32_bridge — 桥接程序原理](#4-gd32_bridge--桥接程序原理)
5. [裂缝判定逻辑](#5-裂缝判定逻辑)
6. [命令通道 — 从 gd32_bridge 到 gnss_path_control](#6-命令通道--从-gd32_bridge-到-gnss_path_control)
7. [gnss_path_control — 路径控制、停车响应与原地旋转](#7-gnss_path_control--路径控制停车响应与原地旋转)
8. [TB6612 电机驱动 — 刹车原理](#8-tb6612-电机驱动--刹车原理)
9. [代码变更清单](#9-代码变更清单按实施顺序)
10. [完整数据流程图](#10-完整数据流程图)
11. [后续步骤](#11-后续步骤)

---

## 1. 系统整体架构

```
┌──────────────────────────────────────────────────────────────────────┐
│                        i.MX6ULL (Linux SBC)                         │
│                                                                     │
│  ┌──────────┐    USB CDC     ┌──────────────┐   TCP:8766  ┌────────┐│
│  │   GD32   │ ──────────────→│ gd32_bridge  │ ──────────→│CarView2││
│  │ (MCU/AI) │    /dev/ttyACM0│  (桥接进程)   │            │ (PC)   ││
│  └──────────┘                └──────┬───────┘            └────────┘│
│       │                             │                              │
│       │ 裂缝检测结果                  │ Unix Socket                   │
│       │ (DET_V2)                     │ /tmp/gd32_vehicle_cmd.sock    │
│       ▼                             ▼                              │
│  ┌──────────┐                ┌──────────────┐                      │
│  │   AI     │                │gnss_path_    │  /dev/tb6612  ┌────┐ │
│  │  模型    │                │control       │ ───────────→ │电机│ │
│  │  推理    │                │(路径控制进程)  │              └────┘ │
│  └──────────┘                └──────────────┘                      │
│                                       ↑                            │
│                                   stdin                            │
│                                   (GPS NMEA)                       │
└──────────────────────────────────────────────────────────────────────┘
```

### 两个独立进程

| 进程 | 程序文件 | 功能 |
|---|---|---|
| **gd32_bridge** | `gd32_bridge/` | 读取 GD32 的 USB CDC 数据，组图、判断裂缝，转发给 CarView2 和 gnss_path_control |
| **gnss_path_control** | `src/path_control/gnss_path_control_main.cpp` | 读取 GPS 定位，执行纯追踪算法，通过 TB6612 驱动电机沿路径行驶 |

### 当前通信现状

| 通道 | 方向 | 传输内容 | 已实现？ |
|---|---|---|---|
| USB CDC | GD32 → gd32_bridge | 图像分片 + DET_V2 检测结果 | ✅ |
| TCP | gd32_bridge → CarView2 | RGB565 图像 + GPS + 检测元数据 | ✅ |
| Unix Socket (GPS) | gnss_path_control → gd32_bridge | GPS 经纬度/航向/速度 | ✅ |
| **Unix Socket (命令)** | **gd32_bridge → gnss_path_control** | **停车/调整/恢复命令** | ❌ **需要新增** |

---

## 2. GD32 端 — 裂缝检测与数据发送

### 2.1 检测流程

```
摄像头帧 (RGB565)
     │
     ▼
GD32 AI 推理引擎
     │
     ├── 未检测到裂缝 ──→ 只发 type=0x01 图像分片包
     │
     └── 检测到裂缝 ──→ 发 type=0x01 图像分片包
                         + type=0x02 DET_V2 检测结果包
                         （两者用 frame_id 关联）
```

### 2.2 发送时序

同一帧的图像和检测结果分开发送：

```
时间轴 →

GD32 → i.MX6ULL:

[type=0x01, frame_id=100, chunk_id=0]  ← 图像第0片
[type=0x01, frame_id=100, chunk_id=1]  ← 图像第1片
...
[type=0x01, frame_id=100, chunk_id=N]  ← 图像最后一片
[type=0x02, frame_id=100]             ← 检测结果（若有裂缝）
                                       ↑ 与图像共用 frame_id 关联

[type=0x01, frame_id=101, chunk_id=0]  ← 下一帧图像
...
```

注意：**检测结果包可能在图像包之前或之后到达**（取决于 GD32 内部调度），gd32_bridge 统一按 `frame_id` 做关联，不依赖顺序。

---

## 3. USB CDC 协议详解

### 3.1 统一包头 (16 字节)

文件：[gd32_bridge/protocol.h:63-78]

```cpp
struct UsbPacketHeader {    // 16 字节
    uint16_t magic;         // 0xAA55 — 帧同步魔数
    uint8_t  type;          // 0x01=图像分片, 0x02=检测事件
    uint8_t  event;         // type=0x02 时有效:
                            //   0x00=NONE(无裂缝)
                            //   0x01=STOP(检测到裂缝→停车)
                            //   0x02=X_DELTA(云台角度偏差)
                            //   0x03=FLOW_END(裂缝处理完成)
    uint16_t width;         // 图像宽度（像素）
    uint16_t height;        // 图像高度（像素）
    uint16_t packet_id;     // 当前分片序号（从0开始）
    uint16_t packet_total;  // 总分片数
    uint16_t payload_len;   // 本片数据长度（字节）
    uint16_t reserved;      // 保留
};
```

**与旧协议的关键区别：**

| 字段 | 旧协议 (V1) | 新协议 (V2.2.0+) |
|------|------------|-----------------|
| 包头大小 | 20 字节 | **16 字节** |
| frame_id | 有（4 字节） | **移除**，不再需要帧关联 |
| event | 无 | **新增**，事件驱动 |
| chunk_id/chunk_total | 有 | **改为** packet_id/packet_total |
| format | 有 | **改为** event 字段 |

### 3.2 事件驱动机制

GD32 不再通过 `frame_id` 关联图像和检测结果，而是通过 **事件** 驱动：

```
USB CDC 数据流（时间轴 →）

[type=0x01, packet_id=0]  ← 图像分片（无需关联检测结果）
[type=0x01, packet_id=1]
...
[type=0x01, packet_id=N]
   ↓ 单帧图像组装完成，直接推入 FrameQueue

[type=0x02, event=STOP(0x01)]  ← 检测到裂缝 → 发布停车命令
[type=0x02, event=STOP(0x01)]  ← 持续检测持续发送
[type=0x02, event=NONE(0x00)]  ← 当前帧无裂缝
[type=0x02, event=X_DELTA(0x02)] ← 云台角度偏差值
[type=0x02, event=FLOW_END(0x03)] ← 裂缝处理完成
```

### 3.3 type=0x01 — 图像分片包

- 完整 RGB565 图像被拆成多个 packet 传输
- 接收端按 `packet_id` / `packet_total` 重组
- 最后一片到达后检查完整性（`width * height * 2` 字节）
- 图像分片 **不携带检测信息**，仅用于 CarView2 画面预览

### 3.4 type=0x02 — 检测事件包

检测事件包带有 28 字节的 `UsbDetObject` 结构体（跟随在 16 字节包头之后）：

文件：[gd32_bridge/protocol.h:92-108]

```cpp
struct UsbDetObject {             // 28 字节
    uint16_t cls_index;            // 类别索引: 0=裂缝, 1=其他
    uint16_t object_id;            // 目标跟踪ID（持续跟踪同一裂缝）
    uint16_t conf_q10000;          // 置信度 × 10000（例: 9500 = 95%）
    uint16_t reserved;
    char name[16];                 // 类别名称字符串（"crack"）
    float x_angle_delta_deg;       // 裂缝偏离画面中心的角度(°)
                                   // 负值=偏左, 正值=偏右
};
```

**事件与处理逻辑对照：**

| 事件 | 触发条件 | gd32_bridge 响应 |
|------|---------|-----------------|
| STOP (0x01) | `cls_index==0 && conf_q10000>5000` | `publishStop()` → 小车立即停车 |
| X_DELTA (0x02) | 云台追踪裂缝时的角度偏差 | `publishAdjust(angle)` → 小车原地旋转 |
| NONE (0x00) | 无裂缝或置信度不足 | 清除异常状态，继续正常巡检 |
| FLOW_END (0x03) | 裂缝处理全流程完成 | `publishResume()` → 小车恢复行驶 |

> **注意：** STOP 和 X_DELTA 事件会携带 UsbDetObject，其中 `x_angle_delta_deg` 为云台 X 舵机当前角度减去初始角度（GIMBAL_X_INIT_ANGLE = 80°）。正值表示裂缝偏右，负值表示偏左。

---

## 4. gd32_bridge — 桥接程序原理

### 4.1 模块结构

文件：[gd32_bridge/main.cpp](gd32_bridge/main.cpp)

```
gd32_bridge 进程
┌─────────────────────────────────────────────────────┐
│ UsbCdcReader  ←─── /dev/ttyACM0 (USB CDC)           │
│   ↓ 组好的帧 (FrameQueue)                           │
│ TcpSender → TCP → CarView2                          │
│                                                     │
│ GpsReceiver  ←─── /tmp/gd32_gps.sock (从路径控制)    │
│   ↓ GPS 状态注入 TCP 包头                            │
└─────────────────────────────────────────────────────┘
```

### 4.2 UsbCdcReader 工作流程（事件驱动）

文件：[gd32_bridge/usb_cdc_reader.cpp](gd32_bridge/usb_cdc_reader.cpp)

```cpp
readLoop() {
    while (running) {
        1. 打开 /dev/ttyACM0 (O_RDWR | O_NOCTTY | O_NONBLOCK)
        2. 设置 raw 模式 (cfmakeraw)
        3. 读取原始字节流到缓冲区
        4. 在缓冲区中扫描 0xAA55 魔数 → 定位包头

        if (找到完整包) {
            switch (type) {
            case 0x01 (IMAGE):
                按 packet_id 组装图像
                if (组图完成) {
                    // 直接推入帧队列，不再查询检测结果
                    pushRgb565Frame(width, height, rgb565_data);
                }
                break;

            case 0x02 (DET_EVENT):
                解析 UsbDetObject（跟在 16B 包头后）
                switch (event) {
                case STOP (0x01):
                    cmd_pub_.publishStop();
                    anomaly_pending = true;
                    break;

                case X_DELTA (0x02):
                    cmd_pub_.publishAdjust(x_angle_delta_deg);
                    anomaly_pending = false;
                    break;

                case NONE (0x00):
                    anomaly_pending = false;
                    break;

                case FLOW_END (0x03):
                    cmd_pub_.publishResume();
                    break;
                }
                break;
            }
        }
    }
}
```

### 4.3 图像和事件的关联机制

新协议不再需要 frame_id 字典关联：

```
旧协议 (V1):                     新协议 (V2.2.0+):
                                
图像分片 [frame_id=N]            图像分片 [无需关联]
检测结果 [frame_id=N]            检测事件 [STOP]
         ↓                                ↓
  detections_[N] 查字典          直接处理事件
         ↓                                ↓
  决定帧类型 (普通/异常)          发布命令到车辆

 ★ 优点：                            ★ 优点：
 - 图像和检测可分离到达               - 无需字典缓存
 - 可处理乱序                         - 事件即时响应
 ★ 缺点：                            - 无内存泄漏风险
 - 需要管理字典(内存泄漏风险)          ★ 缺点：
 - 帧关联延迟                         - 事件和图像各自独立
```

> **注意：** `anomaly_pending` 标志用于 TCP 发送线程决定帧类型：
> - `anomaly_pending=true` → 帧类型标记为 `QUEUE_FRAME_RGB565_ANOM`（异常帧）
> - `anomaly_pending=false` → 帧类型标记为 `QUEUE_FRAME_RGB565`（普通帧）
>
> 此标志确保了 CarView2 能正确区分异常帧（进入异常列表）和普通帧（仅预览）。

---

## 5. 裂缝判定逻辑

### 5.1 事件驱动判定（gd32_bridge 侧）

裂缝判定完全由 GD32 的事件驱动，gd32_bridge **不再自行解析裂缝特征**：

```
USB CDC 数据流
     │
     ▼
UsbCdcReader::readLoop()
     │
     ├─ [type=0x02, event=STOP]
     │   │  GD32 已判定: cls_index==0 && conf_q10000>5000
     │   │  无需 gd32_bridge 二次判定
     │   ▼
     │   cmd_pub_.publishStop()  →  Unix DGRAM socket
     │
     ├─ [type=0x02, event=X_DELTA]
     │   │  GD32 云台追踪角度偏差
     │   ▼
     │   cmd_pub_.publishAdjust(x_angle_delta_deg)  →  Unix DGRAM socket
     │
     ├─ [type=0x02, event=NONE]
     │   │  当前帧无裂缝
     │   ▼
     │   anomaly_pending = false
     │
     └─ [type=0x02, event=FLOW_END]
         │  裂缝处理全流程完成
         ▼
         cmd_pub_.publishResume()  →  Unix DGRAM socket
```

### 5.2 GD32 端的裂缝判定流程

```
摄像头帧 (RGB565)
     │
     ▼
GD32 AI 推理引擎
     │
     ├── cls_index != 0 或 conf_q10000 ≤ 5000
     │   → 发送 [type=0x02, event=NONE(0x00)]
     │
     └── cls_index == 0 && conf_q10000 > 5000
         → 进入裂缝处理流程:
           1. 发送 [type=0x02, event=STOP(0x01)]
           2. 云台开始追踪裂缝 (PID 控制)
           3. 云台稳定后, 发送 [type=0x02, event=X_DELTA(0x02)]
           4. 拍照/测量
           5. 发送 [type=0x02, event=FLOW_END(0x03)]
           6. 恢复巡检
```

### 5.3 GD32 云台追踪状态机（参考）

| 状态 | 动作 | 说明 |
|------|------|------|
| SEARCH_TARGET | 连续 5 帧检测到裂缝才确认 | 防误触发 |
| WAIT_BEFORE_GIMBAL | 等待 5 秒 | 给车辆足够时间刹停 |
| GIMBAL_CENTERING | PID 控制云台使裂缝居中 | `x_angle_delta_deg → 0` |
| KEEP_TRACKING | 保持跟踪，发送 X_DELTA | 通知车辆调整方向 |
| FLOW_END | 处理完成 | 发送 RESUME |

---

## 6. 命令通道 — 从 gd32_bridge 到 gnss_path_control

### 6.1 设计原则

- **简单的协议**：使用 Unix Domain Socket (DGRAM)，每次发送一个固定结构体
- **非阻塞通信**：发送端不确认对方是否收到，接收端定期查询
- **最小消息**：只传递必要的指令数据

### 6.2 消息协议定义

文件 `gd32_bridge/vehicle_cmd.h`（[查看源码](gd32_bridge/vehicle_cmd.h)）：

```cpp
#pragma once
#include <cstdint>

// Unix socket 路径（与 GpsReceiver 的 socket 置于同一目录）
constexpr char VEHICLE_CMD_SOCK_PATH[] = "/tmp/gd32_vehicle_cmd.sock";

// 命令类型
enum VehicleCmdType : uint8_t {
    CMD_NONE   = 0,
    CMD_STOP   = 1,  // 立即停车（裂缝检测到）
    CMD_RESUME = 2,  // 恢复行驶（裂缝处理完成）
    CMD_ADJUST = 3,  // 调整航向（根据角度差）
};

// 消息结构（总共 9 字节）
#pragma pack(push, 1)
struct VehicleCmdMessage {
    VehicleCmdType cmd;          // 命令类型 (1B)
    float          angle_delta_deg;  // 偏转角 (4B)，仅 CMD_STOP/CMD_ADJUST 时有效
    uint32_t       timestamp_ms;     // 时间戳 (4B)，便于接收端判断命令新鲜度
};
#pragma pack(pop)
```

### 6.3 发布端 — VehicleCmdPublisher

文件 [gd32_bridge/vehicle_cmd_publisher.h](gd32_bridge/vehicle_cmd_publisher.h)（[源码](gd32_bridge/vehicle_cmd_publisher.cpp)）：

```cpp
class VehicleCmdPublisher {
public:
    VehicleCmdPublisher();
    ~VehicleCmdPublisher();

    bool start();       // 创建 socket
    void stop();        // 关闭 socket
    void publish(const VehicleCmdMessage &msg);

private:
    int fd_ = -1;
    struct sockaddr_un dest_addr_;
};
```

实现要点：

```cpp
bool VehicleCmdPublisher::start() {
    fd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
    // DGRAM 不需要 bind — 只管发

    // 设置目标地址
    memset(&dest_addr_, 0, sizeof(dest_addr_));
    dest_addr_.sun_family = AF_UNIX;
    strncpy(dest_addr_.sun_path, VEHICLE_CMD_SOCK_PATH,
            sizeof(dest_addr_.sun_path) - 1);

    return fd_ >= 0;
}

void VehicleCmdPublisher::publishStop(float angle_delta) {
    VehicleCmdMessage msg;
    msg.cmd = CMD_STOP;
    msg.angle_delta_deg = angle_delta;
    msg.timestamp_ms = getTimestampMs();

    sendto(fd_, &msg, sizeof(msg), 0,
           (struct sockaddr*)&dest_addr_, sizeof(dest_addr_));
    // DGRAM 不计发送失败 — 接收端可能尚未启动
}
```

### 6.4 接收端 — VehicleCmdListener

文件 [src/path_control/vehicle_cmd_listener.h](src/path_control/vehicle_cmd_listener.h)（[源码](src/path_control/vehicle_cmd_listener.cpp)）：

```cpp
class VehicleCmdListener {
public:
    VehicleCmdListener();
    ~VehicleCmdListener();

    bool start();       // 创建 + bind socket
    void stop();
    bool tryRecv(VehicleCmdMessage &msg);  // 非阻塞接收

private:
    int fd_ = -1;
};
```

实现要点：

```cpp
bool VehicleCmdListener::start() {
    fd_ = socket(AF_UNIX, SOCK_DGRAM, 0);

    // 先删除可能残留的 socket 文件
    unlink(VEHICLE_CMD_SOCK_PATH);

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, VEHICLE_CMD_SOCK_PATH,
            sizeof(addr.sun_path) - 1);

    bind(fd_, (struct sockaddr*)&addr, sizeof(addr));

    // 设置非阻塞
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

    return true;
}

bool VehicleCmdListener::tryRecv(VehicleCmdMessage &msg) {
    // MSG_DONTWAIT: 无数据时立即返回
    ssize_t n = recv(fd_, &msg, sizeof(msg), MSG_DONTWAIT);
    return n == sizeof(msg);
}
```

### 6.5 命令发布时机（已实现）

在 [gd32_bridge/usb_cdc_reader.cpp](gd32_bridge/usb_cdc_reader.cpp) 的 readLoop 中：

```cpp
if (hdr.type == 0x02) {
    // 解析 UsbDetObject（位于包头之后 28 字节）
    UsbDetObject det;
    memcpy(&det, payload, sizeof(det));

    switch (static_cast<UsbDetEvent>(hdr.event)) {
    case UsbDetEvent::STOP:
        cmd_pub_.publishStop();
        anomaly_pending_ = true;
        std::cerr << "DET_EVENT: STOP (crack detected)" << std::endl;
        break;

    case UsbDetEvent::X_DELTA:
        cmd_pub_.publishAdjust(det.x_angle_delta_deg);
        anomaly_pending_ = false;
        std::cerr << "DET_EVENT: X_DELTA angle="
                  << det.x_angle_delta_deg << "°" << std::endl;
        break;

    case UsbDetEvent::NONE:
        anomaly_pending_ = false;
        break;

    case UsbDetEvent::FLOW_END:
        cmd_pub_.publishResume();
        std::cerr << "DET_EVENT: FLOW_END (resume)" << std::endl;
        break;
    }
}
```

---

## 7. gnss_path_control — 路径控制、停车响应与原地旋转

### 7.1 当前控制循环

文件：[src/path_control/gnss_path_control_main.cpp](src/path_control/gnss_path_control_main.cpp)

```cpp
while (!g_stop_requested && std::getline(std::cin, line)) {
    // 1. 查询车辆命令（非阻塞）
    // 2. 解析 NMEA GPS 位置
    // 3. Crack-stop 判断
    // 4. 更新路径控制器 (pure pursuit)
    // 5. 计算左右轮速度
    // 6. driver.set(left, right, error)  ← 驱动电机
}
```

主循环每行 GPS 数据迭代一次（1-10 Hz），每次迭代先查询命令 socket。

### 7.2 完整实现（已实现）

```cpp
// gnss_path_control_main.cpp

// 全局状态
static std::sig_atomic_t g_stop_requested = 0;
static bool g_crack_stop = false;   // 裂缝停车标志

int main(int argc, char **argv) {
    // ... 加载配置、打开驱动 ...

    // ★★★ 启动车辆命令监听 ★★★
    path_control::VehicleCmdListener cmd_listener;
    cmd_listener.start();

    while (!g_stop_requested && std::getline(std::cin, line)) {

        // =============================================================
        // 1. 查询车辆命令（非阻塞，可同时接收多条）
        // =============================================================
        {
            path_control::VehicleCmdMessage cmd;
            while (cmd_listener.tryRecv(cmd)) {
                switch (cmd.cmd) {
                case VehicleCmdType::STOP:
                    g_crack_stop = true;
                    driver.stop(error);
                    break;

                case VehicleCmdType::ADJUST:
                    g_crack_stop = true;
                    doInPlaceRotation(driver, cmd.angle_delta_deg,
                                      config.runtime.dry_run);
                    break;

                case VehicleCmdType::RESUME:
                    g_crack_stop = false;
                    break;
                }
            }
        }

        // =============================================================
        // 2. 如果处于裂缝停车状态
        // =============================================================
        if (g_crack_stop) {
            // 继续读取 GPS 数据并发布（CarView2 地图持续更新）
            // 但不驱动电机
            GnssPosition fix;
            if (parseGnssPositionLine(line, fix)) {
                gps_pub.publish(fix, state, fix.time);
            }
            driver.stop(error);  // 保持刹车
            continue;            // 跳过路径控制和电机输出
        }

        // =============================================================
        // 3. 正常 GNSS 路径控制
        // =============================================================
        GnssPosition fix;
        if (!parseGnssPositionLine(line, fix))
            continue;

        // GNSS-only 航向推算（两GPS点间方位角）
        controller.update(fix, state, out, error);

        // 发布 GPS → Unix socket → gd32_bridge → CarView2
        gps_pub.publish(fix, state, fix.time);

        // 驱动电机
        if (!out.goal_reached) {
            driver.set(out.left_percent, out.right_percent, error);
        } else {
            driver.stop(error);
        }
    }
}
```

### 7.3 三个命令的详细行为

| 命令 | 触发条件 | gnss_path_control 行为 |
|------|---------|----------------------|
| **STOP** | GD32 检测到裂缝 | 1. `g_crack_stop = true`<br>2. `driver.stop()` 刹车<br>3. 继续解析并发布 GPS（地图更新）<br>4. 不驱动电机 |
| **ADJUST** | GD32 云台追踪到裂缝角度 | 1. `g_crack_stop = true`<br>2. 执行 `doInPlaceRotation(driver, angle)`<br>3. 原地旋转到指定角度<br>4. 保持停车状态 |
| **RESUME** | GD32 裂缝处理完成 | 1. `g_crack_stop = false`<br>2. 恢复正常路径跟踪和电机驱动 |

### 7.4 原地旋转 — doInPlaceRotation()

#### 原理

利用差速轮 + 舵机实现纯原地旋转（不产生前后位移）：

```
           舵机居中 (servo 0°)
                │
         ┌──────┴──────┐
         │   前轮(舵机) │
         │   方向摆正   │
         └──────┬──────┘
                │
   左轮 ←─┼──────┼──────┼──→ 右轮
   (-P)    │    车体    │    (+P)
           └───────────┘
   左轮向后，右轮向前 → 逆时针旋转（正角度）
   左轮向前，右轮向后 → 顺时针旋转（负角度）
```

关键点：
- **舵机居中**（`servo(0°)`）：确保前轮摆正，不产生转向分力
- **差速轮反转**：左右轮等速反向旋转，产生纯力矩

#### 计算公式

```
θ = P × K × t

其中:
  θ = 目标旋转角度 (°)
  P = 轮速百分比 (ROTATION_PERCENT, 默认 30%)
  K = 校准系数 (CALIB_K, 需实地标定)
  t = 旋转时间 (s)

→ t = θ / (P × K)
```

#### 校准方法

```
1. 设置 ROTATION_PERCENT = 30
2. 执行 set(+30, -30) 持续 5 秒
3. 测量车身实际旋转角度（实测值）
4. K = 实测角度 / (30 × 5)

示例: 旋转了 240° → K = 240 / (30 × 5) = 1.6
```

#### 代码实现

```cpp
// 校准参数
constexpr int ROTATION_PERCENT = 30;   // 轮速百分比
constexpr double CALIB_K = 1.6;        // 校准系数（需标定）

void doInPlaceRotation(Tb6612Driver &driver,
                       float angle_deg, bool dry_run) {
    // 限幅 ±180°
    if (angle_deg < -180.0f) angle_deg = -180.0f;
    if (angle_deg > 180.0f)  angle_deg = 180.0f;

    const double abs_angle = std::abs(angle_deg);
    if (abs_angle < 0.5) return;  // 角度太小，跳过

    // 计算旋转时间
    const double duration_s = abs_angle / (ROTATION_PERCENT * CALIB_K);

    // 符号决定方向: 正→右转, 负→左转
    const int motor_val = (angle_deg > 0) ? ROTATION_PERCENT : -ROTATION_PERCENT;

    // Step 1: 舵机居中
    driver.servo(0.0f, error);

    // Step 2: 差速轮反转: 左轮=-val, 右轮=+val
    driver.set(-motor_val, motor_val, error);

    // Step 3: 等待
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(duration_s * 1000.0)));

    // Step 4: 停止
    driver.stop(error);
}
```

---

## 8. TB6612 电机驱动 — 刹车原理

文件：[src/path_control/path_controller.cpp:912-1004]

### 8.1 驱动接口

```cpp
class Tb6612Driver {
    bool open(std::string &error);      // 打开 /dev/tb6612 设备
    void close();                       // 关闭设备
    bool stop(std::string &error);      // 刹车 → 写 "stop\n"
    bool set(int left, int right, ...); // 调速 → 写 "set L R\n"
    bool servo(float angle_deg, ...);   // 舵机 → 写 "servo angle\n"
                                        // angle: -90~90° (0°=居中)
};
```

### 8.2 刹车命令路径

```
gnss_path_control 调用 driver.stop()
    ↓
Tb6612Driver::stop()  →  writeCommand("stop\n")
    ↓
写字符串 "stop\n" 到 /dev/tb6612 设备
    ↓
TB6612 驱动板接收 → 两路 PWM 占空比设为 0
    ↓
电机停转 → 小车刹停
```

### 8.3 恢复行驶命令路径

正常控制循环中 `driver.set(left_percent, right_percent, error)` 写入：

```
"set 45 50\n"  ← "set 左轮百分比 右轮百分比\n"
```

TB6612 驱动板收到后恢复 PWM 输出 → 电机重新转动。

---

## 9. 代码变更清单（按实施顺序）

### 9.1 新增文件 ✅ 全部完成

| 文件 | 说明 | 状态 |
|---|---|---|
| `gd32_bridge/vehicle_cmd.h` | 命令协议定义（枚举、结构体、socket路径） | ✅ |
| `gd32_bridge/vehicle_cmd_publisher.h` | 发布端类（gd32_bridge 侧） | ✅ |
| `gd32_bridge/vehicle_cmd_publisher.cpp` | 发布端实现 | ✅ |
| `src/path_control/vehicle_cmd_listener.h` | 接收端类（gnss_path_control 侧） | ✅ |
| `src/path_control/vehicle_cmd_listener.cpp` | 接收端实现 | ✅ |

### 9.2 修改文件 ✅ 全部完成

| 文件 | 修改内容 | 状态 |
|---|---|---|
| `gd32_bridge/protocol.h` | 替换 20B CdcPacketHeader → 16B UsbPacketHeader，新增事件枚举，移除 frame_id/chunk 相关字段 | ✅ |
| `gd32_bridge/usb_cdc_reader.h` | 构造函数增加 `VehicleCmdPublisher &` 参数，移除 detections_ 字典 | ✅ |
| `gd32_bridge/usb_cdc_reader.cpp` | 事件驱动 readLoop：STOP→publishStop, X_DELTA→publishAdjust, FLOW_END→publishResume | ✅ |
| `gd32_bridge/main.cpp` | 创建 VehicleCmdPublisher，传入 UsbCdcReader | ✅ |
| `gd32_bridge/CMakeLists.txt` | 添加 vehicle_cmd_publisher.cpp | ✅ |
| `src/path_control/path_controller.h` | Tb6612Driver 增加 `servo(float angle)` 方法 | ✅ |
| `src/path_control/path_controller.cpp` | 实现 servo 方法（写 "servo angle\n" 到设备） | ✅ |
| `src/path_control/gnss_path_control_main.cpp` | 完整重写：VehicleCmdListener 集成、裂缝停车状态机、原地旋转 doInPlaceRotation() | ✅ |
| `CMakeLists.txt`（主目录） | KF-GINS-GnssPathControl 目标添加 vehicle_cmd_listener.cpp | ✅ |
| `config/kf-gins.yaml` | track_width 从 0.26 改为 0.16 | ✅ |

### 9.3 待完成项

| 项 | 说明 | 优先级 |
|---|---|---|
| 标定 `CALIB_K` | 实地测试：set(+30,-30) 5s → 测角度 → 计算 K | **高（使用前必须）** |
| 标定 `ROTATION_PERCENT` | 根据小车重量、地面摩擦力调整 | 中 |
| 参数 YAML 化 | 将 ROTATION_PERCENT/CALIB_K 从硬编码改为配置文件参数 | 低 |
| 角度限幅调整 | 当前 ±180°，如实测旋转不足可调整 | 低 |

---

## 10. 完整数据流程图

```
时间线从左到右 →

GD32 (MCU)
  │
  │ 摄像头采集到一帧图像
  │ AI 模型推理
  │
  ├─[无裂缝]────────────────────────────────────────────────
  │  │
  │  │ USB CDC: [type=0x01, packet_id/N] RGB565 分片
  │  │ USB CDC: [type=0x02, event=NONE]
  │  ▼
  │
  ├─[有裂缝]────────────────────────────────────────────────
  │  │
  │  │ USB CDC: [type=0x01, packet_id/N] RGB565 分片 (图像)
  │  │ USB CDC: [type=0x02, event=STOP, UsbDetObject]
  │  │    ↓ 停车，云台开始追踪裂缝
  │  │ USB CDC: [type=0x02, event=X_DELTA, angle=...]
  │  │    ↓ 裂缝居中，角度偏差值
  │  │ USB CDC: [type=0x02, event=FLOW_END]
  │  │    ↓ 处理完成
  │  ▼
  │
  ▼
┌─────────────────────────────────────────────────────────────┐
│ gd32_bridge                                                 │
│                                                             │
│ UsbCdcReader::readLoop()                                    │
│                                                             │
│ ① 扫描魔数 0xAA55 → 解析 16B UsbPacketHeader               │
│ ② 判断 type:                                                │
│    • 0x01 → 按 packet_id/packet_total 组装 RGB565 图像       │
│      完成后 push QUEUE_FRAME_RGB565 (always)                │
│      (图像本身不再区分普通/异常，异常帧由 TcpSender 标记)     │
│                                                             │
│    • 0x02 → 解析 event 字段:                                 │
│      - STOP(0x01)  → cmd_pub_.publishStop()                 │
│                       anomaly_pending_ = true                │
│      - X_DELTA(0x02)→ cmd_pub_.publishAdjust(angle_deg)     │
│                       anomaly_pending_ = false               │
│      - NONE(0x00)   → anomaly_pending_ = false               │
│      - FLOW_END(0x03)→ cmd_pub_.publishResume()              │
│                                                             │
│ ③ TcpSender 从队列取帧                                      │
│    • anomaly_pending=true  → TYPE_RGB565_ANOM → 异常记录    │
│    • anomaly_pending=false → TYPE_RGB565 → 普通预览         │
│                                                             │
│ ④ VehicleCmdPublisher (Unix DGRAM socket)                   │
│    • publishStop()   → /tmp/gd32_vehicle_cmd.sock           │
│    • publishAdjust() → /tmp/gd32_vehicle_cmd.sock           │
│    • publishResume() → /tmp/gd32_vehicle_cmd.sock           │
└─────────────────────────────────┬───────────────────────────┘
                                  │
                                  │ Unix DGRAM 9B 消息
                                  │ { cmd(1B), angle_delta(4B), ts(4B) }
                                  │
        ┌─────────────────────────┼─────────────────────────┐
        ▼                         ▼                         ▼
   CMD_STOP                 CMD_ADJUST                CMD_RESUME
        │                         │                         │
        │ driver.stop()           │ doInPlaceRotation()     │ g_crack_stop = false
        │ g_crack_stop = true     │  servo(0°)              │
        │                         │  set(-P, +P)            │
        ▼                         │  sleep(t)               ▼
   ┌──────────┐                   │  driver.stop()     ┌──────────┐
   │  刹停    │                   ▼                    │  恢复行驶 │
   │ GPS继续  │              ┌──────────┐              │  set(L,R) │
   │ 地图更新  │              │  原地旋转  │              └──────────┘
   └──────────┘              └──────────┘

                                  │
                                  │ "stop\n" / "set L R\n" / "servo angle\n"
                                  │ 写入 /dev/tb6612 字符设备
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────┐
│ TB6612 电机驱动模块                                          │
│                                                             │
│ 收到 "stop\n":                                               │
│   → PWM_A = 0, PWM_B = 0                                    │
│   → 电机停转 → 小车刹停                                     │
│                                                             │
│ 收到 "set 45 50\n":                                          │
│   → PWM_A = 45%, IN1/IN2 → 左轮转速                         │
│   → PWM_B = 50%, IN3/IN4 → 右轮转速                         │
│   → 小车行驶                                                 │
│                                                             │
│ 收到 "servo 0\n":                                            │
│   → 舵机 PWM 占空比 → 前轮居中                               │
│                                                             │
│ 收到 "servo -30\n":                                          │
│   → 舵机 PWM 占空比 → 前轮左转 30°                          │
└─────────────────────────────────────────────────────────────┘
```

---

## 11. 后续步骤

### 11.1 已完成的功能

| 功能 | 状态 | 说明 |
|------|------|------|
| USB CDC 协议更新（16B 包头、事件驱动） | ✅ | protocol.h, usb_cdc_reader |
| 命令通道（Unix DGRAM socket） | ✅ | vehicle_cmd.h, publisher, listener |
| gd32_bridge 事件处理（STOP/X_DELTA/FLOW_END） | ✅ | usb_cdc_reader.cpp |
| gnss_path_control 停车状态机 | ✅ | STOP 刹车 + GPS 继续发布 |
| 原地旋转（差速轮 + 舵机） | ✅ | doInPlaceRotation() |
| GNSS-only 航向推算 | ✅ | 两 GPS 点计算方位角 |
| TB6612 servo 驱动 | ✅ | path_controller.cpp |
| 轮距修正（0.26 → 0.16m） | ✅ | config + 代码 |
| CMakeLists.txt 更新 | ✅ | 主目录 + gd32_bridge |

### 11.2 待完成的标定工作

#### 1. 原地旋转标定（必须）

```bash
# 步骤 1: 在平地上标定 CALIB_K
# 执行原地旋转 5 秒（P=30%）
# 测量实际旋转角度
# 计算: K = 实测角度 / (30 * 5)

# 步骤 2: 验证
# 发送 ADJUST 90° 命令，测量实际旋转角度
# 如果不足 90°，增大 CALIB_K
# 如果超过 90°，减小 CALIB_K

# 步骤 3: 细调 ROTATION_PERCENT
# 如果 30% 扭矩不足（原地打滑或不转），增大到 40-50%
# 重新标定 K 值
```

#### 2. 裂缝检测 → STOP 响应延迟测试

```
测试方法:
  1. 在 GD32 摄像头前放置模拟裂缝
  2. 测量从 GD32 检测 → USB CDC → gd32_bridge → Unix socket
     → gnss_path_control → driver.stop() 的完整延迟
  3. 目标: < 500ms

  影响延迟的因素:
  - GD32 AI 推理时间（通常在 100-300ms）
  - USB CDC 传输延迟（921600 bps，最大 5ms/帧）
  - Unix socket 传输延迟（< 1ms）
  - GPS 主循环阻塞等待（0-1000ms，取决于 GPS 频率）
```

### 11.3 后续优化方向

| 优先级 | 优化项 | 说明 |
|--------|--------|------|
| **高** | 参数 YAML 化 | 将 ROTATION_PERCENT、CALIB_K、角度限幅放入配置文件 |
| **中** | 使用 poll() 方案 | 替代 std::getline 阻塞，实现更实时命令响应 |
| **中** | ADJUST 后自动 RESUME | 当前 ADJUST 后需 GD32 发送 FLOW_END 才恢复 |
| **低** | 激光测距精确定位 | 裂缝前精确停车，需要新增传感器模块 |
| **低** | 多裂缝连续处理 | 队列管理，处理完一个裂缝后继续下一个 |
