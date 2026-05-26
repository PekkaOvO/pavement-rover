# 系统部署与使用说明

## 系统架构

```
                          开发板 (i.MX6ULL)
                        ┌──────────────────────────────┐
GD32 ──USB-TTL@921600──►│ gd32_bridge                  │──TCP:8766──► CarView2 (PC)
                        │  ├─ UART读取线程              │              ├─ 地图显示
GNSS (USB GPS) ────────►│  ├─ GPS接收线程 (Unix socket) │              ├─ 图像预览
          │             │  └─ TCP发送线程               │              ├─ CSV滚动记录
          ▼             │                              │              └─ 图片缓冲
 gnss_path_control      └──────────────────────────────┘
       │                          ▲
       └── Unix socket ───────────┘
       (gps_publisher)
```

## 一、编译环境准备

### 1.1 PC (x86) 编译环境

```bash
# 安装基础工具
sudo apt-get install cmake build-essential gdb

# 安装 Qt6 (用于 CarView2)
sudo apt-get install qt6-base-dev qt6-websockets-dev libqt6sql6-sqlite

# 安装 ARM 交叉编译工具链 (用于编译开发板程序)
sudo apt-get install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf
```

### 1.2 项目文件结构

```
/home/jayce/linux/IMX6ULL/
├── Drivers/Linux_Drivers/gd32_comm/    # 内核驱动
├── C_APP/pwm/KF-GINS-main/             # 主项目目录
│   ├── gd32_bridge/                    # 桥接守护进程
│   ├── src/path_control/               # 路径规划算法
│   ├── cmake/arm-toolchain.cmake        # ARM交叉编译配置
│   └── bin/                            # 编译输出

/home/jayce/QtProjects/CarView2/        # 上位机程序
```

---

## 二、编译（PC 上执行）

### 2.1 编译 x86 程序（KF-GINS + 路径规划，在PC上运行）

用于在PC上处理离线数据、仿真测试：

```bash
# 进入项目目录
cd /home/jayce/linux/IMX6ULL/C_APP/pwm/KF-GINS-main

# 配置（x86原生编译）
mkdir -p build && cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# 编译产物在 bin/ 目录：
#   KF-GINS                    - GNSS/INS组合导航解算
#   KF-GINS-PathControl        - 原始路径规划（KF-GINS输出 → 电机控制）
#   KF-GINS-GnssPathControl    - GNSS-only路径规划（纯GPS → 电机控制）
#   gd32_bridge                - GD32桥接守护进程（ARM架构，需交叉编译）
```

> **注意：** `gd32_bridge` 此时也会被 x86 编译器编译，但只用于在 PC 端测试语法。开发板部署需交叉编译（见2.3）。

### 2.2 编译 CarView2（上位机PC）

```bash
cd /home/jayce/QtProjects/CarView2
mkdir -p build && cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### 2.3 交叉编译 ARM 程序（开发板部署）

**编译 gd32_bridge：**

```bash
cd /home/jayce/linux/IMX6ULL/C_APP/pwm/KF-GINS-main
mkdir -p build-arm && cd build-arm
cmake ../ -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-toolchain.cmake
make -j$(nproc) gd32_bridge

# 产物：bin/gd32_bridge (ARM架构)
```

**编译 KF-GINS-GnssPathControl（含GPS发布功能）：**

```bash
# 同样在 build-arm 目录下
cd /home/jayce/linux/IMX6ULL/C_APP/pwm/KF-GINS-main/build-arm
cmake ../ -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-toolchain.cmake
make -j$(nproc) KF-GINS-GnssPathControl

# 产物：bin/KF-GINS-GnssPathControl (ARM架构)
```

### 2.4 编译内核模块 gd32_comm（可选）

```bash
cd /home/jayce/linux/IMX6ULL/Drivers/Linux_Drivers/gd32_comm
make

# 产物：gd32_comm.ko (ARM内核模块)
```

---

## 三、开发板部署

将交叉编译得到的文件拷贝到 i.MX6ULL 开发板：

```bash
# 假设开发板IP为 192.168.1.10
scp bin/gd32_bridge root@192.168.1.10:/usr/local/bin/
scp bin/KF-GINS-GnssPathControl root@192.168.1.10:/usr/local/bin/
scp ../Drivers/Linux_Drivers/gd32_comm/gd32_comm.ko root@192.168.1.10:/root/
scp ../dataset/*.yaml root@192.168.1.10:/root/  # 配置文件
```

---

## 四、使用场景

### 场景A：查看GPS输出

在开发板上连接 USB-GPS 模块后，查看 GPS 原始 NMEA 数据：

```bash
# USB-GPS 默认波特率 115200
stty -F /dev/ttyUSB0 115200 raw -echo && cat /dev/ttyUSB0
```

如果 GPS 连接在别的串口，请替换 `/dev/ttyUSB0` 为实际设备名。

### 场景B：离线数据处理（在PC上运行）

用数据集跑 KF-GINS 组合导航解算：

```bash
cd /home/jayce/linux/IMX6ULL/C_APP/pwm/KF-GINS-main
./bin/KF-GINS ./dataset/kf-gins.yaml
```

### 场景C：原始路径规划（KF-GINS + PathControl, PC上仿真）

```bash
cd /home/jayce/linux/IMX6ULL/C_APP/pwm/KF-GINS-main

# 试运行（--dry-run，不控制电机，只打印输出）
./bin/KF-GINS ./dataset/kf-gins.yaml | ./bin/KF-GINS-PathControl ./kf-gins.yaml --dry-run

# 实际运行（连接TB6612电机驱动，控制小车运动）
./bin/KF-GINS ./dataset/kf-gins.yaml | ./bin/KF-GINS-PathControl ./kf-gins.yaml
```

> 管道前：KF-GINS 解析 IMU+GNSS 数据 → stdout 输出导航结果
> 管道后：PathControl 读取导航结果 → 计算控制量 → 输出到电机

### 场景D：GNSS-only 路径规划（在开发板上运行）

纯GPS位置控制（不使用IMU），GPS数据通过 stdin 输入：

```bash
# 方式1：从GPS接收机实时读取
cat /dev/ttyUSB0 | ./KF-GINS-GnssPathControl ./config.yaml

# 方式2：从文件读取（回放测试）
cat recorded_gnss.txt | ./KF-GINS-GnssPathControl ./config.yaml

# 试运行模式：
cat /dev/ttyUSB0 | ./KF-GINS-GnssPathControl ./config.yaml --dry-run
```

支持的输入格式（自动识别）：
- `lat lon` — 经纬度
- `time lat lon [height]` — 带时间戳
- `GNSS,time,lat,lon,height,course_deg,speed_kph` — CSV格式
- `$GNGGA,...` + `$GNVTG,...` — 原始NMEA语句

额外命令行参数：
- `--dry-run`：试运行，不控制电机
- `--dev /dev/tb6612`：指定电机驱动设备
- `--print-every N`：每N个样本打印一次状态
- `--max-speed X`：最大速度限制 (m/s)

### 场景E：GD32图像桥接（完整系统）

**前提：**
- GD32 开发板通过 USB-TTL 连接到 i.MX6ULL 开发板
- GD32 以 921600 bps 发送 JPEG 图片和摄像机角度
- USB-GPS 连接到开发板提供定位数据

**步骤1：在 PC 上启动 CarView2 上位机**

```bash
cd /home/jayce/QtProjects/CarView2/build
./CarView2
```

启动后界面：
- 左上方：在线地图（显示车辆位置）
- 左下方：图像预览 Dock（显示GD32拍摄的图片）
- 右上方：异常抓拍画面
- 右下方：异常记录列表

CarView2 会自动在以下端口监听：
- `8765` — WebSocket（原有的异常记录功能）
- `8766` — TCP（gd32_bridge 传送的图片+状态数据）

同时会在当前目录自动创建：
- `state/` 目录 — 滚动 CSV 状态文件（50MB/个，保留最新2个）
- `images/` 目录 — 图片环形缓冲（最近20张）

**步骤2：在开发板启动 gd32_bridge**

```bash
# 语法：gd32_bridge <CarView2的IP> [端口]
/usr/local/bin/gd32_bridge 192.168.1.100 8766
```

gd32_bridge 启动后自动：
1. 打开 `/dev/ttyUSB0` @ 921600，设置 TIOCEXCL 独占锁
2. 创建 Unix socket `/tmp/gd32_gps.sock` 等待 GPS 数据
3. 连接 CarView2（IP:192.168.1.100，端口8766）
4. 如果 TCP 断线，自动指数退避重连（1s → 2s → 4s → ... → 30s）

**步骤3：启动 GNSS 路径规划（含GPS发布）**

```bash
# GPS数据通过stdin输入，程序自动向/tmp/gd32_gps.sock推送位置
cat /dev/ttyUSB0 | /usr/local/bin/KF-GINS-GnssPathControl ./config.yaml
```

**步骤4：连接 GD32 开发板**

将 GD32 的 USB-TTL 插入 Linux 开发板的 USB 口，设备节点 `/dev/ttyUSB0` 出现后，gd32_bridge 会自动检测并打开。

**完整启动顺序：**

```bash
# === 在 PC 上 ===
# 终端 1: 启动 CarView2
./CarView2

# === 在开发板上 ===
# 终端 1: 启动桥接守护进程
gd32_bridge 192.168.1.100 8766

# 终端 2: 启动 GPS 路径规划
cat /dev/ttyGPS | KF-GINS-GnssPathControl ./config.yaml
```

---

## 五、数据流说明

```
GD32拍摄图片 ──► USB-TTL(921600) ──► gd32_bridge UART线程
                                          │
                                    帧同步+CRC校验
                                          │
                                    压入 FrameQueue
                                          │
GPS 定位数据 ──► gnss_path_control ──► Unix socket ──► gd32_bridge GPS线程
                                          │
                                    原子变量缓存最新状态
                                          │
gd32_bridge TCP发送线程 ← 读取Queue + GPS状态
          │
    打包二进制TCP协议:
    | MAGIC | VERSION | TYPE | SEQ | TS_MS |
    | LAT | LON | COURSE | SPEED | HEIGHT |
    | CAM_ANGLE | IMAGE_LEN | [JPEG] | CRC16 |
          │
          ▼
    CarView2 (端口8766)
     ├─ 地图更新位置
     ├─ 图像预览Dock显示图片
     ├─ CSV文件滚动存储
     └─ images/目录缓冲最近20张
```

### 时间同步机制

- GD32 每帧图片携带 Unix 时间戳
- GPS 位置也带有 GNSS 时间戳
- gd32_bridge 将图片时间戳和 GPS 时间戳打包在同一 TCP 帧中
- CarView2 按时间戳对齐显示

### 故障恢复

| 故障 | 恢复机制 |
|------|----------|
| USB-TTL 断开 | 每2秒重试打开设备 |
| TCP 断线 | 指数退避重连（1s→30s） |
| CRC 校验失败 | 丢弃帧，继续同步 |
| GPS 无数据 | 发送无图片状态帧（TYPE=0x02） |

---

## 六、编译命令速查表

| 组件 | 目录 | 命令 | 目标平台 |
|------|------|------|----------|
| KF-GINS (x86) | `KF-GINS-main/` | `cmake .. && make -j$(nproc)` | PC |
| KF-GINS-PathControl (x86) | `KF-GINS-main/` | 同上（一起编译） | PC |
| KF-GINS-GnssPathControl (ARM) | `KF-GINS-main/build-arm/` | `cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-toolchain.cmake && make -j$(nproc) KF-GINS-GnssPathControl` | 开发板 |
| gd32_bridge (ARM) | `KF-GINS-main/build-arm/` | `cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-toolchain.cmake && make -j$(nproc) gd32_bridge` | 开发板 |
| gd32_comm.ko | `Drivers/Linux_Drivers/gd32_comm/` | `make` | 开发板 |
| CarView2 | `QtProjects/CarView2/` | `cmake .. && make -j$(nproc)` | PC |
