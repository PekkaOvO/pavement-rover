# 系统部署与使用说明

## 系统架构

```
                          开发板 (i.MX6ULL)
                        ┌──────────────────────────────────┐
GD32 ───USB CDC (ACM)──►│ gd32_bridge                      │──TCP:8766──► CarView2
                        │  ├─ USB CDC读取+事件处理          │
GNSS ──stdin──────────► │  ├─ GPS接收 (Unix socket)        │
                        │  └─ 车辆命令发布 (Unix DGRAM)     │
                        └──────────┬───────────────────────┘
                                   │
                    ┌──────────────┴──────────────┐
                    │ 控制程序 (二选一)             │
                    ├──────────────────────────────┤
                    │ gnss_path_control            │
                    │  ├─ GPS发布 → Unix socket    │
                    │  ├─ 命令接收 ← Unix DGRAM    │
                    │  ├─ 裂缝停车状态机             │
                    │  └─ TB6612电机驱动            │
                    │                              │
                    │ obstacle_avoid_demo           │
                    │  ├─ 7阶段状态机                │
                    │  ├─ 命令接收 ← Unix DGRAM     │
                    │  └─ TB6612电机驱动            │
                    └──────────────┬───────────────┘
                                   │
                                   ▼
                          TB6612电机驱动
                           (/dev/tb6612)
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
#   KF-GINS-ObstacleAvoidDemo  - 障碍物绕行演示 (ARM)
#   KF-GINS-TestControl        - 裂缝流程测试 (ARM)

# 如只想编译特定目标：
cmake --build . --target KF-GINS-ObstacleAvoidDemo -j$(nproc)
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
scp bin/gd32_bridge bin/KF-GINS-GnssPathControl bin/KF-GINS-ObstacleAvoidDemo \
    start_vehicle.sh \
    ../Drivers/Linux_Drivers/gd32_comm/gd32_comm.ko \
    ../dataset/*.yaml config/obstacle_avoid_demo.yaml \
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

### 场景E：障碍物绕行演示（Obstacle Avoid Demo）

独立演示程序，不依赖 GPS/IMU，只需 GD32 发送停车/旋转/恢复命令即可驱动完整绕行流程：

1. Phase 1: 直行前进 → 等待 GD32 CMD_STOP
2. Phase 2: 原地旋转（角度来自 GD32 CMD_ADJUST）
3. Phase 3: 继续直行 → 等待 GD32 CMD_STOP（障碍物检测）
4. Phase 4: 等待 GD32 CMD_RESUME（流程结束标志）→ 后退（舵机左转，后轮差速偏右）
5. Phase 5: 前进绕行（舵机右转，后轮左快右慢）
6. Phase 6: 轮回正，直行
7. Phase 7: 舵机左转 + 差速左转 → 回正直行
8. Final: 继续前进直到 Ctrl+C

所有阶段的时间和速度由独立 YAML 配置（修改 obstacle_avoid_demo.yaml，不影响 kf-gins.yaml）：

```bash
# 在开发板上运行 --dry-run 试运行
cd /home/root
./KF-GINS-ObstacleAvoidDemo ./obstacle_avoid_demo.yaml --dry-run

# 确认无误后去掉 --dry-run 实际运行
./KF-GINS-ObstacleAvoidDemo ./obstacle_avoid_demo.yaml
```

程序监听 `/tmp/gd32_vehicle_cmd.sock` 接收 GD32 命令：
| 命令 | 事件 | 作用 |
|------|------|------|
| CMD_STOP(1) | USB_DET_EVENT_STOP(0x01) | 检测到裂缝，停车 |
| CMD_ADJUST(2) | USB_DET_EVENT_X_DELTA(0x02) | 云台居中，发送旋转角度 |
| CMD_RESUME(3) | USB_DET_EVENT_FLOW_END(0x03) | 裂缝流程结束，继续行驶 |

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
  ┌────┴────────────────────────────┐
  │ 两种独立消费者，二选一运行        │
  └────┬────────────────────────────┘
       │
       ├── gnss_path_control (基于GPS路径跟踪)
       │    ├─ 收到 STOP  → driver.stop()，继续发布GPS
       │    ├─ 收到 ADJUST → doInPlaceRotation(angle)
       │    ├─ 收到 RESUME → 恢复路径跟踪
       │    └─ 正常行驶   → pure pursuit 计算目标速度 → driver.set()
       │
       └── obstacle_avoid_demo (独立状态机，无需GPS)
            ├─ Phase 1:  收到 STOP → 停止 → 等 ADJUST
            ├─ Phase 2:  收到 ADJUST → doInPlaceRotation(angle)
            ├─ Phase 3:  直行 → 等第2次 STOP
            ├─ Phase 4:  等 RESUME → 后退(差速偏右)
            ├─ Phase 5:  前进绕行(右转)
            ├─ Phase 6:  直行
            ├─ Phase 7:  左转回正
            └─ Final:    持续直行
       │
       │ driver.set(left%, right%) / driver.stop() / driver.servo(angle)
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
| KF-GINS-ObstacleAvoidDemo (ARM) | `KF-GINS-main/build_imx6ull/` | `cmake --build . --target KF-GINS-ObstacleAvoidDemo -j$(nproc)` | 开发板 |
| CarView2 | `QtProjects/CarView2/` | `cmake .. && make -j$(nproc)` | PC |

---

## 七、总体控制流程

### 7.1 系统总览

整个系统由**三个并行子系统**组成，在开发板和 PC 之间协同工作：

```
┌────────────────────────────────────────────────────────────────────┐
│ PC (CarView2 上位机)                                                │
│  ┌─────────────────┐  ┌─────────────────┐  ┌──────────────────┐   │
│  │ 在线地图显示     │  │ 图像预览        │  │ 异常记录列表     │   │
│  │ (接收GPS经纬度)  │  │ (显示RGB565图像) │  │ (记录裂缝事件)   │   │
│  └────────┬────────┘  └────────┬────────┘  └────────┬─────────┘   │
│           │                    │                     │              │
│           └────────┬───────────┴──────────┬──────────┘              │
│                    │  TCP 连接 (端口 8766)  │                        │
└────────────────────┼──────────────────────┼────────────────────────┘
                     │                      │
┌────────────────────┼──────────────────────┼────────────────────────┐
│ 开发板 i.MX6ULL    │                      │                        │
│         ┌──────────▼──────────────────────▼──────────┐             │
│         │              gd32_bridge                    │             │
│         │  ┌────────────────────┐  ┌───────────────┐  │             │
│         │  │ USB CDC 读取线程   │  │ TCP 发送线程  │  │             │
│         │  │  - 解析图像分片    │  │  - 图像帧     │  │             │
│         │  │  - 解析DET事件     │  │  - GPS位置    │  │             │
│         │  │  - 发布车辆命令    │  │  - 异常事件   │  │             │
│         │  └────────┬───────────┘  └───────────────┘  │             │
│         │           │                                  │             │
│         │   Unix DGRAM: /tmp/gd32_vehicle_cmd.sock     │             │
│         └───────────┼──────────────────────────────────┘             │
│                     │                                               │
│           ┌─────────▼─────────┐      ┌─────────────────────────┐   │
│           │ 控制程序 (二选一)  │      │ USB-GPS → NMEA stdin    │   │
│           │                   │      │                         │   │
│           │ ① GnssPathControl │      │  cat /dev/ttyUSB0       │   │
│           │   (GPS路径跟踪)    │──────► 经纬度/航向/速度        │   │
│           │                   │      │                         │   │
│           │ ② ObstacleAvoid   │      │ → pure pursuit 计算     │   │
│           │   (障碍物绕行)     │      │ → driver.set(L,R)      │   │
│           └─────────┬─────────┘      │ → GPS → Unix socket     │   │
│                     │                │    → gd32_bridge → TCP  │   │
│                     │                │    → CarView2 地图      │   │
│                     ▼                └─────────────────────────┘   │
│            ┌────────────────┐                                      │
│            │  TB6612 电机   │                                      │
│            │  /dev/tb6612   │                                      │
│            │  set(L,R)      │                                      │
│            │  servo(angle)  │                                      │
│            │  stop()        │                                      │
│            └────────────────┘                                      │
└────────────────────────────────────────────────────────────────────┘
```

### 7.2 硬件接线与设备节点

| 设备 | 开发板接口 | 设备节点 | 说明 |
|------|-----------|---------|------|
| GD32 摄像头 | USB-TTL (CH340) | `/dev/ttyACM0` | USB CDC 图像+事件，921600 bps |
| USB-GPS | USB 口 | `/dev/ttyUSB0` (典型) | NMEA 协议，115200 bps |
| TB6612 电机驱动 | GPIO | `/dev/tb6612` | 字符设备，set/stop/servo 命令 |
| 网络 | 以太网/WiFi | — | 开发板 ↔ PC 同一网段 |

### 7.3 需要在开发板部署的文件

从 PC 拷贝到开发板 `/home/root/`：

```bash
# 编译产物 (ARM)
bin/gd32_bridge                # GD32 桥接守护进程（必需）
bin/KF-GINS-ObstacleAvoidDemo  # 障碍物绕行演示（如需）
bin/KF-GINS-GnssPathControl    # GPS 路径规划（如需）
bin/KF-GINS-TestControl        # 裂缝流程测试（如需）

# 配置文件
config/obstacle_avoid_demo.yaml  # 绕行演示配置
config/kf-gins.yaml (或 config.yaml)  # 路径规划配置

# 脚本
start_vehicle.sh               # 一键启动脚本
```

### 7.4 两种控制模式说明

系统支持两种独立的控制程序，**每次只能运行其中一个**：

| 特性 | GnssPathControl (GPS路径跟踪) | ObstacleAvoidDemo (障碍物绕行) |
|------|-------------------------------|-------------------------------|
| 依赖 GPS | ✅ 必需 | ❌ 不需要 |
| 依赖 GD32 | ✅ 接收 STOP/ADJUST/RESUME | ✅ 接收 STOP/ADJUST/RESUME |
| 驱动方式 | pure pursuit 动态计算目标速度 | 固定速度+系数，按阶段切换 |
| 适用场景 | 正常巡逻，沿路径行驶 | 演示障碍物绕行全流程 |
| 配置来源 | kf-gins.yaml | obstacle_avoid_demo.yaml |
| 启动方式 | 管道输入 NMEA | 独立运行 |

### 7.5 完整启动流程

#### 分步启动（推荐调试用）

```bash
# ========== PC 端 (Windows / WSL) ==========
# 终端1: 启动上位机
cd /home/jayce/QtProjects/CarView2/build
./CarView2
# CarView2 启动后监听 TCP:8766 (图像/GPS/状态) + WebSocket:8765 (异常记录)

# ========== 开发板端 (i.MX6ULL, root@172.20.10.12) ==========
# 终端2: 启动 GD32 桥接守护进程（后台运行）
cd /home/root
./gd32_bridge 172.20.10.3 8766 &
# 输出示例:
#   gd32_bridge starting: server=172.20.10.3:8766
#   usb_cdc: opened /dev/ttyACM0
#   stats: frames=...

# 终端3: 启动控制程序（二选一）

# 方式A: GPS 路径跟踪 (需要 USB-GPS)
cat /dev/ttyUSB0 | ./KF-GINS-GnssPathControl ./config.yaml --dry-run
# 去掉 --dry-run 实际运行:
# cat /dev/ttyUSB0 | ./KF-GINS-GnssPathControl ./config.yaml

# 方式B: 障碍物绕行演示 (不需要 GPS)
./KF-GINS-ObstacleAvoidDemo ./obstacle_avoid_demo.yaml --dry-run
# 去掉 --dry-run 实际运行:
# ./KF-GINS-ObstacleAvoidDemo ./obstacle_avoid_demo.yaml
```

#### 一键启动（生产用）

```bash
# PC端
./CarView2

# 开发板端: 自动启动 gd32_bridge + GnssPathControl
./start_vehicle.sh 172.20.10.3 /dev/ttyUSB0 ./config.yaml
```

### 7.6 各子系统数据流详解

#### (a) 图像传输流 (GD32 → CarView2)

```
GD32 OV5640 摄像头
  → RGB565 图像 (256×256)
  → USB CDC 分包 (16B header + payload)
  → gd32_bridge USB 读取线程
      ├─ 按 packet_id/packet_total 重组完整图像
      └─ 放入帧队列 (FrameQueue)
  → gd32_bridge TCP 发送线程
  → CarView2 TCP:8766 接收
      ├─ 图像预览 Dock (左下方)
      └─ 异常抓拍 Dock (右上方，仅 type=0x02 anomaly)
```

#### (b) GPS/姿态数据流 (USB-GPS → CarView2)

```
USB-GPS 模块
  → NMEA 语句 ($GNGGA, $GNVTG, $GPRMC...)
  → stdin 管道输入 gnss_path_control
  → 解析经纬度/航向/速度/高度
  → pure pursuit 路径跟踪
      ├─ 计算目标速度 → driver.set(L, R)
      └─ 发布 GPS 位置 → Unix socket /tmp/gd32_gps.sock
          → gd32_bridge GPS 接收线程
          → TCP 发送 → CarView2
              ├─ 地图更新 (左上角)
              └─ CSV 状态记录
```

#### (c) 小车控制流 (以 ObstacleAvoidDemo 为例)

```
程序启动 → 加载 YAML 配置 → 打开 /dev/tb6612
         → 启动 VehicleCmdListener (监听 Unix socket)
         → 进入 7 阶段状态机:

  Phase 1 直行
    servo(0°) → set(40%, 40%) → 持续前进
    → 收到 GD32 CMD_STOP(t=0x01) → stop()
    → 收到 GD32 CMD_ADJUST(t=0x02, angle=θ) → 保存旋转角度
    └────────── Phase 2 完成 ──────────

  Phase 2 原地旋转
    servo(0°) → set(-15%, +15%)
    → t = |θ| / (P × K) 秒后 stop()
    └────────── Phase 2 完成 ──────────

  Phase 3 继续直行
    servo(0°) → set(40%, 40%) → 持续前进
    → 收到 GD32 CMD_STOP  → stop()
    └────────── Phase 3 完成 ──────────

  Phase 4 等待 → 后退
    → 收到 GD32 CMD_RESUME(t=0x03, 流程结束)
    servo(-30°) → set(-30%, -50%) → 持续 1.5s
    └────────── Phase 4 完成 ──────────

  Phase 5 前进绕行 (右转)
    servo(30°) → set(60%, 30%) → 持续 2s
    └────────── Phase 5 完成 ──────────

  Phase 6 直行
    servo(0°) → set(40%, 40%) → 持续 3s
    └────────── Phase 6 完成 ──────────

  Phase 7 左转回正
    servo(-30°) → set(30%, 60%) → 持续 1.5s
    └────────── Phase 7 完成 ──────────

  Final 持续直行直到 Ctrl+C
    servo(0°) → set(40%, 40%)
```

### 7.7 上位机 CarView2 界面说明

CarView2 启动后在四个 Dock 区域显示信息：

| 区域 | 位置 | 显示内容 | 数据来源 |
|------|------|----------|----------|
| 在线地图 | 左上方 | 车辆实时位置 (经纬度标注) | gd32_bridge TCP → GPS 经纬度 |
| 图像预览 | 左下方 | GD32 摄像头实时画面 | gd32_bridge TCP → RGB565 图像 |
| 异常抓拍 | 右上方 | 检测到裂缝时的抓拍图片 | gd32_bridge TCP → type=0x02 anomaly |
| 异常列表 | 右下方 | 裂缝发生时间、位置、图片记录 | gd32_bridge TCP + 本地 SQLite |

自动生成的文件：
- `state/` 目录 — 滚动 CSV 状态文件（50MB/个，保留最新2个）
- `images/` 目录 — 图片环形缓冲（最近20张）

---

> **最后更新：2026-06-07**
>
> 以下为仅使用 **ObstacleAvoidDemo**（障碍物绕行演示）的最小启动方案，不涉及 GPS 路径跟踪。

## 八、ObstacleAvoidDemo 最小启动方案

### 8.1 网络拓扑

```
开发板 i.MX6ULL          Windows                  WSL (Ubuntu)
172.20.10.12              172.20.10.3             172.31.41.250
     │                        │                        │
     │──── TCP:8766 ─────────►│── portproxy:8766 ─────►│ CarView2
     │  (gd32_bridge 发图像)  │   (Windows 转发到 WSL)  │
     │                        │                        │
     │──── GD32 USB CDC ──┐   │                        │
     │                    │   │                        │
     │◄─── /dev/tb6612 ───┘   │                        │
     │  (ObstacleAvoidDemo)   │                        │
```

### 8.2 需要拷贝到开发板的文件

```bash
# 从 PC (KF-GINS-main 目录) 拷贝到开发板 /home/root/
scp bin/gd32_bridge               root@172.20.10.12:/home/root/
scp bin/KF-GINS-ObstacleAvoidDemo  root@172.20.10.12:/home/root/
scp config/obstacle_avoid_demo.yaml root@172.20.10.12:/home/root/
```

### 8.3 端口转发（仅 CarView2 在 WSL 中运行时需要）

WSL 默认 NAT 模式，Windows 和开发板在同一网段（172.20.10.x），但 WSL 在独立子网（172.31.41.x），开发板无法直接访问 WSL。

**在 Windows PowerShell（管理员）中执行：**

```powershell
# 添加端口转发：Windows:8766 → WSL:8766
netsh interface portproxy add v4tov4 listenport=8766 listenaddress=0.0.0.0 connectport=8766 connectaddress=172.31.41.250

# 防火墙放行 8766 端口（TCP 入站）
netsh advfirewall firewall add rule name="CarView2-8766" dir=in action=allow protocol=TCP localport=8766

# 查看已配置的转发规则
netsh interface portproxy show all
```

> **注意：** 如果 CarView2 直接在 Windows 上运行（非 WSL），则不需要端口转发，只需防火墙放行 8766 端口即可。

### 8.4 启动顺序

#### 步骤 1：PC 端启动 CarView2

```bash
# WSL 中启动
cd /home/jayce/QtProjects/CarView2/build
./CarView2

# （确保 Windows 防火墙已放行 8766 端口，见 8.3）
```

#### 步骤 2：开发板启动 gd32_bridge（图像传输）

```bash
ssh root@172.20.10.12
cd /home/root

# 后台运行 gd32_bridge，图像发到 Windows IP
./gd32_bridge 172.20.10.3 8766 &
# 输出示例：
#   gd32_bridge starting: server=172.20.10.3:8766
#   usb_cdc: opened /dev/ttyACM0
#   stats: frames=...
```

#### 步骤 3：开发板启动 ObstacleAvoidDemo

```bash
# 先 --dry-run 试运行，确认动作序列
./KF-GINS-ObstacleAvoidDemo ./obstacle_avoid_demo.yaml --dry-run

# 确认无误后实际运行
./KF-GINS-ObstacleAvoidDemo ./obstacle_avoid_demo.yaml
```

### 8.5 完整流程示例

```
PC (WSL) 端:                              开发板端:

Terminal 1:                                Terminal 2:
$ ./CarView2                               $ ssh root@172.20.10.12
  (等待 TCP:8766 连接...)                   $ ./gd32_bridge 172.20.10.3 8766 &
                                           $ ./KF-GINS-ObstacleAvoidDemo ./obstacle_avoid_demo.yaml

CarView2 显示:                             程序输出:
  ┌──────────┬──────────┐                    === Obstacle Avoidance Demo ===
  │ 地图      │ 异常抓拍  │                    Device: /dev/tb6612
  │(无GPS时   │          │                    Forward: L=40% R=19%
  │ 无显示)   │          │                    [Phase 1] Driving forward...
  ├──────────┼──────────┤  ← GD32 检测到裂缝 → CMD_STOP received
  │ 图像预览  │ 异常列表  │                    [Phase 2] Rotating...
  │(摄像头    │          │  ← GD32 云台居中  → CMD_ADJUST angle=45°
  │ 实时画面) │          │                    [Phase 3] Forward again...
  └──────────┴──────────┘  ← GD32 障碍物    → CMD_STOP received
                                              [Phase 4] Waiting for CMD_RESUME...
                           ← GD32 流程结束   → CMD_RESUME received → backup...
                                              [Phase 5] Evading...
                                              [Phase 6] Straight...
                                              [Phase 7] Turn left...
                                              [Final] Continuing...
```

### 8.6 注意事项

| 问题 | 解决方法 |
|------|----------|
| CarView2 收不到图像 | 检查 Windows 防火墙是否放行 8766 端口；检查 WSL 端口转发是否配置正确 |
| 开发板连不上 Windows | `ping 172.20.10.3` 确认网络通；检查 Windows 网络类型（需为"专用网络"） |
| ObstacleAvoidDemo 启动报错 | 先用 `--dry-run` 试运行；检查 `/dev/tb6612` 是否存在 |
| gd32_bridge 打不开 /dev/ttyACM0 | 确认 GD32 USB 已连接；检查 `ls /dev/ttyA*` |
| WSL IP 变化 | `ip addr show eth0` 查看新 IP，更新 portproxy：`netsh interface portproxy set v4tov4 ...` |
