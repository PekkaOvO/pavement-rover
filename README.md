# 系统部署与使用说明

## 系统架构

```
                          开发板 (i.MX6ULL)
                        ┌──────────────────────────────────┐
GD32 ───USB CDC (ACM)──►│ gd32_bridge                      │──TCP:8766──► CarView2
                        │  ├─ USB CDC读取+事件处理          │
GNSS ──stdin──────────► │  ├─ GPS接收 (Unix socket)        │
          │             │  └─ 车辆命令发布 (Unix DGRAM)     │
          ▼             └──────────┬───────────┬───────────┘
 gnss_path_control                 │           ▲
   ├─ GPS发布 ────Unix socket──────┘           │
   ├─ 命令接收 ◄───/tmp/gd32_vehicle_cmd.sock──┘
   ├─ 裂缝停车状态机
   ├─ 原地旋转 (差速轮+舵机, t=θ/(P×K))
   └─ TB6612电机驱动 (/dev/tb6612)
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

> ARM 编译需要两个输出目录，不要混淆：
> - `build/` — x86 原生编译（PC 上跑离线处理）
> - `build_imx6ull/` — ARM 交叉编译（开发板上跑）

```bash
cd /home/jayce/linux/IMX6ULL/C_APP/pwm/KF-GINS-main
mkdir -p build_imx6ull && cd build_imx6ull

cmake ../ -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-toolchain.cmake
make -j$(nproc)

# 产物在 bin/ 目录：
#   gd32_bridge                - GD32桥接守护进程 (ARM)
#   KF-GINS-GnssPathControl    - GNSS-only路径规划 (ARM)
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
# 假设开发板IP为 172.20.10.12，在工作目录 KF-GINS-main 下执行
scp bin/gd32_bridge bin/KF-GINS-GnssPathControl start_vehicle.sh \
    ../Drivers/Linux_Drivers/gd32_comm/gd32_comm.ko \
    ../dataset/*.yaml \
    root@172.20.10.12:/home/root/

# 开发板上设置脚本可执行
ssh root@172.20.10.12 "chmod +x /home/root/start_vehicle.sh"
```

### 3.1 当前图像桥接调试用命令（固定 IP）

当前调试环境：

- 上位机/Windows IP：`172.20.10.3`
- i.MX6ULL 开发板 IP：`172.20.10.12`
- 开发板部署目录：`/home/root/`
- CarView2 TCP 端口：`8766`

#### 3.1.1 编译 CarView2（WSL/PC 上执行）

```bash
cd /home/jayce/QtProjects/CarView2
cmake --build build --target TcpReceiverTest CarView2 -j2
ctest --test-dir build --output-on-failure -R TcpReceiverTest
```

运行 CarView2：

```bash
cd /home/jayce/QtProjects/CarView2
./build/CarView2
```

#### 3.1.2 编译开发板程序（WSL/PC 上执行）

`gd32_bridge` 目录内有两个常用程序：

- `gd32_bridge`：真实桥接程序，读取 GD32 USB CDC 图像和 DET_V2 检测标签，再通过 TCP 发给 CarView2。
- `tcp_image_test_sender`：测试发送器，不依赖 GD32，可直接发送普通/异常/混合图像给 CarView2。

```bash
cd /home/jayce/linux/IMX6ULL/C_APP/pwm/KF-GINS-main/gd32_bridge

cmake -S . -B build-arm \
  -DCMAKE_CXX_COMPILER=arm-linux-gnueabihf-g++ \
  -DCMAKE_C_COMPILER=arm-linux-gnueabihf-gcc

cmake --build build-arm --target gd32_bridge tcp_image_test_sender -j2
```

确认编译产物是 ARM 程序：

```bash
file build-arm/gd32_bridge build-arm/tcp_image_test_sender
```

期望看到 `ELF 32-bit LSB executable, ARM, EABI5`。

#### 3.1.3 拷贝到开发板（WSL/PC 上执行）

```bash
cd /home/jayce/linux/IMX6ULL/C_APP/pwm/KF-GINS-main/gd32_bridge

scp build-arm/gd32_bridge build-arm/tcp_image_test_sender \
  root@172.20.10.12:/home/root/

ssh root@172.20.10.12 \
  "chmod +x /home/root/gd32_bridge /home/root/tcp_image_test_sender && ls -l /home/root/gd32_bridge /home/root/tcp_image_test_sender"
```

#### 3.1.4 运行测试发送器（开发板上执行）

先启动新版 CarView2，再在开发板运行：

```bash
cd /home/root
./tcp_image_test_sender 172.20.10.3 8766 30 200 mixed
```

参数含义：

- `172.20.10.3`：上位机/Windows IP。
- `8766`：CarView2 TCP 接收端口。
- `30`：发送 30 帧。
- `200`：每 200 ms 发送一帧。
- `mixed`：普通图和异常图混合发送。

测试模式：

```bash
# 只发普通图：只应出现在图像预览
./tcp_image_test_sender 172.20.10.3 8766 30 200 normal

# 全部发异常图：应出现在图像预览，并进入异常抓拍和异常列表
./tcp_image_test_sender 172.20.10.3 8766 20 200 anomaly

# 普通/异常混合：所有图进预览，异常图额外进异常抓拍和异常列表
./tcp_image_test_sender 172.20.10.3 8766 30 200 mixed
```

发送端日志示例：

```text
sent frame seq=0 type=anomaly bytes=131154
sent frame seq=1 type=normal bytes=131126
sent frame seq=2 type=normal bytes=131126
sent frame seq=3 type=anomaly bytes=131154
```

#### 3.1.5 运行真实桥接程序（开发板上执行）

确认 GD32 通过 USB CDC 接到开发板，设备通常为 `/dev/ttyACM0`。先启动新版 CarView2，再运行：

```bash
cd /home/root
./gd32_bridge 172.20.10.3 8766
```

期望日志：

```text
gd32_bridge starting:
  server:  172.20.10.3:8766
  cdc:     /dev/ttyACM0 (USB CDC ACM)
  gps:     /tmp/gd32_gps.sock
usb_cdc: opened /dev/ttyACM0
tcp_sender: connected
stats: frames=...
```

真实桥接逻辑：

- GD32 发送 `type=0x01 format=0x01` 的 RGB565 图像包。
- GD32 发送 `type=0x02 format=0x02` 的 DET_V2 检测包。
- `gd32_bridge` 按 `frame_id` 合并图像和检测结果。
- 无 DET_V2 目标：CarView2 只更新图像预览。
- 有 DET_V2 目标：CarView2 更新图像预览，同时生成异常记录，进入异常抓拍和异常信息列表。

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

### 场景F：裂缝处理全流程测试（test_control）

用模拟数据测试"正常行驶 → STOP → 原地旋转 → RESUME"完整流程：

```bash
# 在开发板上运行
# 终端 1: 启动 CarView2 (PC上)

# 终端 2: 编译 test_control (ARM)
cd /home/jayce/linux/IMX6ULL/C_APP/pwm/KF-GINS-main
cmake --build build_imx6ull --target KF-GINS-TestControl -j2

# 终端 2: 运行测试（管道到 gnss_path_control）
sudo ./bin/KF-GINS-TestControl <start_lat> <start_lon> 0.5 \
  | sudo ./bin/KF-GINS-GnssPathControl ./config.yaml

# 示例（lat=30.0, lon=120.0, 向北0.5m/步 @ 5Hz）
sudo ./bin/KF-GINS-TestControl 30.0 120.0 0.5 \
  | sudo ./bin/KF-GINS-GnssPathControl ./config.yaml
```

**测试流程：**

```text
终端输出（test_control 侧）:
  === Test Control for GNSS Path Control ===

  1. 小车自动开始向前行驶 (GPS 数据持续输出)

  2. 输入: stop
     → 发送 CMD_STOP → 小车刹车停止
     → GPS 继续输出（CarView2 地图保持更新）

  3. 输入: adjust 90
     → 发送 CMD_ADJUST angle=90°
     → 小车原地旋转 90° (servo 0°, set(-30%,+30%), sleep(t), stop)

  4. 输入: resume
     → 发送 CMD_RESUME → 小车恢复路径跟踪

  5. 输入: quit → 退出
```

**支持的交互命令：**

| 命令 | 作用 |
|------|------|
| `stop` | 发送停车命令 |
| `adjust <deg>` | 发送原地旋转命令（角度正=右转，负=左转） |
| `resume` | 发送恢复行驶命令 |
| `origin <lat> <lon>` | 重置 GPS 模拟位置 |
| `heading <deg>` | 设置 GPS 行进方向（0=北，90=东） |
| `speed <m/s>` | 设置每步行进距离 |
| `pause / cont` | 暂停/继续 GPS 输出 |
| `status` | 显示当前状态 |
| `quit` | 退出 |

**前提：**
- GD32 开发板通过 USB-TTL 连接到 i.MX6ULL 开发板（`/dev/ttyUSB0`）
- GD32 以 921600 bps 发送 JPEG 图片和摄像机角度
- USB-GPS 连接到开发板（通常是 `/dev/ttyUSB1`）
- 电机驱动板（TB6612）连接到开发板
- 网络已连接（开发板和 PC 在同一局域网）

一键启动：

```bash
# === PC端：一条命令启动上位机 ===
cd /home/jayce/QtProjects/CarView2/build
./CarView2
```

```bash
# === 开发板端：一条命令启动全部 ===
# 复制到开发板后执行:
chmod +x start_vehicle.sh
./start_vehicle.sh 192.168.1.100 /dev/ttyUSB1 ./config.yaml
# 参数: ./start_vehicle.sh <上位机IP> [GPS设备] [配置文件]
# 默认:                     192.168.1.100  /dev/ttyUSB1 ./config.yaml
```

`start_vehicle.sh` 脚本会自动按顺序完成以下工作：
1. 启动 `gd32_bridge` → 连接 CarView2、等待 USB-TTL 和 GPS 数据
2. 等待 GPS 设备就绪 → 启动 `KF-GINS-GnssPathControl` 并自动推送位置到 bridge
3. GPS 断开或程序退出后自动清理后台进程

**接线说明：**

| 设备 | 开发板接口 | 设备节点 |
|------|-----------|---------|
| GD32 | USB-TTL (CH340) | `/dev/ttyUSB0` |
| USB-GPS | USB 口 | `/dev/ttyUSB1` |
| 电机驱动 | GPIO (参考 tb6612 驱动) | `/dev/tb6612` |
| 网络 | 以太网或 WiFi | — |

**详细启动步骤（如果一键脚本有问题，按以下分步执行）：**

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

**完整启动顺序（一键版）：**

```bash
# === PC端 ===
./CarView2

# === 开发板端（一条命令） ===
./start_vehicle.sh 192.168.1.100
```

**完整启动顺序（分步版）：**

```bash
# === 在 PC 上 ===
# 终端 1: 启动 CarView2
./CarView2

# === 在开发板上 ===
# 终端 1: 启动桥接守护进程（后台运行）
gd32_bridge 192.168.1.100 8766 &

# 终端 2: 启动 GPS 路径规划
cat /dev/ttyGPS | KF-GINS-GnssPathControl ./config.yaml
```

---

## 五、数据流说明

### 5.1 图像+检测数据流

```
GD32 ──USB CDC──► gd32_bridge
  │
  ├─ type=0x01 (图像分片)
  │   16B header + RGB565 chunk data
  │   按 packet_id/packet_total 重组完整图像
  │
  ├─ type=0x02 (检测结果+事件)
  │   16B header + 28B UsbDetObject
  │   event=STOP(0x01)    → 检测到裂缝，发布停车命令
  │   event=X_DELTA(0x02) → 云台角度偏差，发布调整命令
  │   event=NONE(0x00)    → 无裂缝，清除异常状态
  │   event=FLOW_END(0x03)→ 裂缝处理完成，发布恢复命令
  │
  ↓ 帧队列 (FrameQueue)
  │
gd32_bridge TCP发送线程 ──► CarView2 (TCP:8766)
  ├─ 地图更新 (GPS位置)
  ├─ 图像预览 (RGB565)
  ├─ 异常抓拍记录
  └─ CSV滚动存储
```

### 5.2 车辆控制命令流

```
gd32_bridge (检测到裂缝事件)
       │
       │ VehicleCmdPublisher
       │ Unix DGRAM socket: /tmp/gd32_vehicle_cmd.sock
       │ 消息: { cmd(1B), angle_delta_deg(4B), timestamp_ms(4B) }
       ▼
gnss_path_control
  ├─ 收到 STOP  → driver.stop()，保持刹车，继续发布GPS
  ├─ 收到 ADJUST → doInPlaceRotation(driver, angle)
  │                  公式: t = |θ| / (P × K)
  │                  流程: servo 0° → set(-P, +P) → sleep(t) → stop
  └─ 收到 RESUME → 恢复路径跟踪
       │
       │ driver.set(left%, right%) 或 driver.stop()
       ▼
  TB6612电机驱动 (/dev/tb6612)
```

### 5.3 GPS数据流

```
USB-GPS (NMEA) ──stdin──► gnss_path_control
                                │
                           解析经纬度/航向/速度
                                │
                           路径跟踪 (pure pursuit)
                                │
                           ├─ 控制电机 (TB6612)
                           └─ 发布GPS → Unix socket → gd32_bridge → TCP → CarView2
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
| KF-GINS-GnssPathControl (ARM) | `KF-GINS-main/build_imx6ull/` | `cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-toolchain.cmake && make -j$(nproc) KF-GINS-GnssPathControl` | 开发板 |
| gd32_bridge (ARM) | `KF-GINS-main/build_imx6ull/` | `cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-toolchain.cmake && make -j$(nproc) gd32_bridge` | 开发板 |
| gd32_bridge standalone (ARM) | `KF-GINS-main/gd32_bridge/` | `cmake -S . -B build-arm -DCMAKE_CXX_COMPILER=arm-linux-gnueabihf-g++ && cmake --build build-arm --target gd32_bridge -j2` | 开发板 |
| tcp_image_test_sender (ARM) | `KF-GINS-main/gd32_bridge/` | `cmake -S . -B build-arm -DCMAKE_CXX_COMPILER=arm-linux-gnueabihf-g++ && cmake --build build-arm --target tcp_image_test_sender -j2` | 开发板 |
| gd32_comm.ko | `Drivers/Linux_Drivers/gd32_comm/` | `make` | 开发板 |
| CarView2 | `QtProjects/CarView2/` | `cmake .. && make -j$(nproc)` | PC |
