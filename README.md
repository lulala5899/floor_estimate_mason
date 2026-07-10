# floor_estimate_mason

基于 **ESP32-S3 + BMP390** 的气压测楼层方案（单板 self-relative 模式）。

电梯内无 Wi-Fi 信号，双板差分不可行，故本仓库专注于 **单板模式**：一块 ESP32 通过 USB 串口将气压数据发送到电脑，电脑端 ROS2 节点根据实地校准的基准气压换算海拔，实时输出楼层估计。

---

## 硬件清单

| 组件 | 说明 |
|------|------|
| ESP32-S3 开发板 | USB 串口设备一般为 `/dev/ttyACM0` |
| BMP390 气压传感器模块 | I2C 接口 |
| 杜邦线 | 母对母，4 条 |

---

## 接线

```
3V3  -> BMP390 VCC
GND  -> BMP390 GND
GPIO37 -> BMP390 SDA
GPIO38 -> BMP390 SCL
CS   -> 3V3（固定 I2C 模式）
SDO  -> GND（地址 0x76）或 3V3（地址 0x77）
```

> 如果只有一个 3V3/GND 引脚，可用杜邦线并联给 VCC/CS 和 GND/SDO。

---

## 快速开始

### 1. 安装依赖

**PlatformIO（固件编译）**

```bash
python3 -m pip install --user -U platformio
```

**ROS2 Humble**

确保已安装 ROS2 Humble，并 source 环境：

```bash
source /opt/ros/humble/setup.bash
```

### 2. 编译烧录 ESP32 固件

```bash
cd ESP32_Barometer-main
python3 -m platformio run -e barometer_node_s3 -t upload --upload-port /dev/ttyACM0
```

关键配置已预设：

| 项目 | 值 |
|------|-----|
| 编译环境 | `barometer_node_s3` |
| I2C 引脚 | SDA=37, SCL=38 |
| USB 模式 | CDC（日志走 `/dev/ttyACM0`）|
| 采样间隔 | **50ms（20Hz）** |
| 传感器 IIR 滤波 | **Coeff 1（轻滤波，低延迟）** |
| 时间同步 | **非阻塞式，上电即输出数据** |

### 3. 串口验证

```bash
python3 -m platformio device monitor -p /dev/ttyACM0 -b 115200
```

期望输出：

```
INFO: I2C init SDA=37 SCL=38
BAROD>timestamp,pressure_hpa,temp_c
BAROT>24:EC:4A:01:43:20
```

### 4. 编译 ROS2 包

```bash
cd ros_barometer-main
colcon build --symlink-install
source install/setup.bash
```

### 5. 启动节点

```bash
ros2 launch serial_to_ros2 baro_p_alti_launch.py
```

启动后控制台输出示例：

```
[INFO] Starting baseline calibration: collecting pressure for 10.0s
[INFO] Calibration sample 1: 1005.23 hPa
...
[INFO] Baseline calibration done: 1005.19 hPa (UG layer, 198 samples over 10.1s)
[INFO] First altitude: 0.023 m
```

---

## 系统工作原理

### 上电自动校准（核心新功能）

传统方法需要手动测量当地气压填入配置文件，本系统实现了**全自动校准**：

```
上电 → ESP32 开始输出 BAROD> 数据
  ↓
ROS2 节点进入校准阶段（默认 10 秒）
    → 每收到一帧数据就记录气压值
    → 只发布 /pressure 原始气压（不发布海拔）
    → 等待 10 秒收集足够样本
  ↓
10 秒结束 → 计算所有样本的均值 = UG 层基准气压
  ↓
发出 /baseline_pressure 话题（基准气压值，只发一次）
  ↓ 自动更新 default_local_pressure
  ↓
进入正常阶段：
  → 用 ISA 大气模型算海拔（基于 UG 基准气压）
  → 海拔 / 层高 = 楼层号 → 持续发 /floor_estimate（无防抖，实时）
  → 中值滤波(10 帧) + 连续 5 帧一致 → 发 /floor_state（防抖确认）
  → 同时发布 /pressure + /barometer + /z_motion
```

校准完成后，海拔计算以 UG 层为参考零点，因此：
- 在 UG 层时 `altitude ≈ 0m`
- 在 1F 时 `altitude ≈ 3m`（取决于 `floor_height` 参数）
- 在 B1（地下一层）时 `altitude ≈ -3m`

### 楼层防抖机制

气压噪声和气流扰动可能使原始海拔在小范围内波动，导致 `/floor_estimate` 频繁跳变。系统的**四层防抖**方案：

```
原始海拔 (20Hz)
  ↓ 存入环形缓冲区（默认 10 帧）
中值滤波 → 去中位数 → 候选楼层号
  ↓ 死区检测：越过楼层边界至少 0.3m 才考虑变化（抗气压漂移）
  ↓ 连续一致确认：必须同一个候选值持续 N 帧（默认 5 帧≈250ms）
  ↓ 运动门控：竖直速度回落到 0.15m/s 以下才提升给 /floor_state
最终确认 → 更新 /floor_state（TRANSIENT_LOCAL QoS，1Hz 心跳重发）
```

- `/floor_estimate` — **无防抖**，保留实时性，适合需要快速响应的场景
- `/floor_state` — 四项防抖全部通过后才更新。QoS 为 TRANSIENT_LOCAL depth=1，新订阅者连上即能获取最后一次楼层值，同时 1Hz 心跳无条件重发当前值

### 固件数据流

```
BMP390 传感器（50Hz ODR，IIR Coeff 1）
  │ 每 50ms 读取一次气压和温度
  ▼
ESP32 主循环
  │ 输出 BAROD>时间戳,气压hPa,温度°C
  │ 每秒发送一次 BAROT> 请求时间同步
  │ 收到 TS> 后更新本地时钟
  ▼
USB 串口（115200 bps）
```

### ROS2 节点数据流

```
USB 串口 → serial_reader_task（异步读取）
  │
  ▼
_process_serial_line → _serial_raw_to_pressure
  │
  ├─ 校准未完成: 收集样本，仅发 /pressure
  │                    10s 后算出基准气压
  │                    发 /baseline_pressure（仅一次）
  │
  └─ 校准完成: 算 ISA 海拔 → 发 /barometer（含 altitude）
                          → 中值滤波(10帧) → 死区检测(0.3m)
                          → 同候选连续确认(5帧) → 运动门控
                          → 发 /floor_state
                          → 实时算楼层 → 发 /floor_estimate（无防抖）
                          → 回读 /barometer
                              → 有限差分法算垂直速度/加速度
                              → 发 /z_motion
```

---

## ROS2 话题列表

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/pressure` | `sensor_msgs/FluidPressure` | ~20Hz | 原始气压（Pa），校准期间也发布 |
| `/barometer` | `barometer_interfaces/Barometer` | ~20Hz | 含海拔(altitude)、气压、温度，校准后才发布 |
| `/z_motion` | `barometer_interfaces/ZMotion` | ~20Hz | 垂直速度(vspeed)和加速度(vacc) |
| `/baseline_pressure` | `std_msgs/Float32` | 仅 1 次 | UG 层基准气压值(hPa)，校准完成时发布 |
| `/floor_estimate` | `std_msgs/Int32` | ~20Hz | 当前楼层号（0=UG, 1=1F, -1=B1），校准后才发布，无防抖 |
| `/floor_state` | `std_msgs/Int32` | 变化时 + 1Hz 心跳 | 四层防抖(中值滤波+死区+连续确认+运动门控)确认后的楼层号。QoS TRANSIENT_LOCAL，新订阅者立刻拿到最新值 |

### 话题消息结构

**Barometer.msg**
```
std_msgs/Header header
float32 altitude     # 海拔高度（米），相对于 UG 基准气压
float32 pressure     # 气压（帕斯卡）
float32 temperature  # 温度（摄氏度）
```

**ZMotion.msg**
```
std_msgs/Header header
float32 vspeed   # 垂直速度 (m/s)，正值为上升
float32 vacc     # 垂直加速度 (m/s²)
```

---

## 配置参数

配置文件：`ros_barometer-main/serial_to_ros2/config/esp32_serial_baro.yaml`

```yaml
esp32_serial_baro:
  ros__parameters:
    serial_port: /dev/ttyACM0          # ESP32 串口设备路径
    output_mode: self-relative         # 单板模式（固定）
    frequency: 5.0                     # 节点内频率（不影响实际数据率）
    default_local_pressure: 1004.0     # 校准前的临时参考气压（会被校准值覆盖）
    calibration_duration: 10.0         # 上电采集校准时长（秒）
    floor_height: 3.0                  # 每层楼高度（米）
    floor_debounce_buffer_size: 10     # 中值滤波窗口（帧数）
    floor_debounce_consecutive: 5      # 同候选连续确认帧数
    floor_debounce_deadband: 0.3       # 楼层边界死区（米）
    floor_state_heartbeat: 1.0         # /floor_state 心跳重发间隔（秒）
    motion_gate_speed_threshold: 0.15  # 运动门控速度阈值（m/s）
    motion_gate_window: 10             # 速度平均窗口（帧数）
```

### 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/dev/ttyACM0` | ESP32 串口设备路径，留空会自动扫描 |
| `calibration_duration` | `10.0` | 上电后采集多少秒气压作为 UG 基准 |
| `floor_height` | `3.0` | 每层楼高度（米），用于计算楼层号 |
| `default_local_pressure` | `1004.0` | 校准前的临时值，校准完成后会被自动覆盖 |
| `frequency` | `5.0` | 内部轮询频率（实际数据率由 ESP32 20Hz 主导） |
| `floor_debounce_buffer_size` | `10` | 中值滤波窗口大小（帧），越大越平滑但响应越慢 |
| `floor_debounce_consecutive` | `5` | 同一候选楼层需连续确认多少帧才锁定 |
| `floor_debounce_deadband` | `0.3` | 楼层边界死区（米），防止气压漂移导致误判 |
| `floor_state_heartbeat` | `1.0` | /floor_state 心跳重发间隔（秒），0 则禁用 |
| `motion_gate_speed_threshold` | `0.15` | 电梯静止判定：平均|速度|低于此值（m/s）才提交楼层 |
| `motion_gate_window` | `10` | 速度平均窗口（帧数），越大越不易被瞬时噪声误触发 |

---

## 命令行参数覆盖

可通过 `--ros-args -p` 临时覆盖参数：

```bash
# 校准时长改为 15 秒
ros2 run serial_to_ros2 esp32_serial_baro \
    --ros-args -p serial_port:=/dev/ttyACM0 \
    -p calibration_duration:=15.0

# 层高改为 3.5 米
ros2 run serial_to_ros2 esp32_serial_baro \
    --ros-args -p serial_port:=/dev/ttyACM0 \
    -p floor_height:=3.5
```

---

## 调试与查看数据

### 新开终端查看话题

```bash
source /opt/ros/humble/setup.bash
source ros_barometer-main/install/setup.bash

# 查看基准气压（校准完成后才会出现）
ros2 topic echo /baseline_pressure --once

# 实时监听楼层（快速原始估计）
ros2 topic echo /floor_estimate

# 监听防抖确认后的楼层（变化时更新）
ros2 topic echo /floor_state

# 查看完整数据
ros2 topic echo /barometer --once

# 查看所有活跃话题
ros2 topic list
```

### 预期输出

**校准阶段（前 10 秒）：**
```bash
$ ros2 topic echo /pressure --once
header:
  stamp:
    sec: 1775629956
    nanosec: 165000000
  frame_id: barometer_link
fluid_pressure: 100523.0    # 单位 Pa（= 1005.23 hPa）
variance: 0.0
```

**校准完成后：**
```bash
$ ros2 topic echo /barometer --once
header:
  stamp:
    sec: 1775629966
    nanosec: 65000000
  frame_id: barometer_link
altitude: 2.85              # 相对于 UG 层的高度（米）
pressure: 100491.0          # 单位 Pa
temperature: 28.95          # 单位 °C

$ ros2 topic echo /floor_estimate --once
data: 1                     # 当前在 1F（实时估计）

$ ros2 topic echo /floor_state --once
data: 1                     # 当前在 1F（防抖确认）

$ ros2 topic echo /baseline_pressure --once
data: 1005.19               # UG 层基准气压（hPa）
```

---

## 时间同步

ESP32 固件实现了自动时间同步协议：

- 上电后每秒发送一次 `BAROT>` 请求
- ROS2 节点收到后回复 `TS><unix_ms>` 时间戳
- ESP32 更新本地 RTC，后续 `BAROD>` 中的时间戳即为正确的 Unix 毫秒时间
- 每 2 小时重新同步一次，补偿时钟漂移

此过程**非阻塞**，上电立即输出数据，不影响采集。

---

## 常见问题

**Q: 串口设备不是 `/dev/ttyACM0`？**
A: 用 `ls /dev/ttyACM*` 或 `ls /dev/ttyUSB*` 查看实际设备名，修改 YAML 配置或启动时用 `-p serial_port:=/dev/ttyACM1`。

**Q: 校准 10 秒不够/太长？**
A: 修改 `calibration_duration` 参数。一般 10 秒足够（约 200 个样本），楼层环境越稳定时间越短。

**Q: 楼层号不对？**
A: 调整 `floor_height` 参数匹配实际楼层高度。标准层高通常 3m，商住楼可能有差异。

**Q: `/floor_estimate` 和 `/floor_state` 有什么区别？**
A: `/floor_estimate` 是每帧实时计算的原始楼层号（20Hz），可能因气压噪声轻微跳变；`/floor_state` 经中值滤波 + 连续 5 帧一致确认后才更新，适合 UI 展示和稳态逻辑。两者同时可用。

**Q: 高度变化响应慢？**
A: 当前固件已优化：采样 50ms，IIR 滤波系数 1。如需更快可在 `HP206C.cpp` 中关闭 IIR 滤波（`BMP3_IIR_FILTER_COEFF_0`），但噪声会略微增加。

**Q: `colcon build` 时报找不到 `barometer_interfaces`？**
A: 先编译消息包：
```bash
cd ros_barometer-main
colcon build --packages-select barometer_interfaces
source install/setup.bash
colcon build --symlink-install
```

**Q: ROS2 Humble 不是我的版本？**
A: 把命令中的 `humble` 替换为你的 ROS2 版本即可（如 `jazzy`、`rolling`）。

---

## 系统架构

```
┌─────────────────────────────────────────────────┐
│               ESP32-S3 + BMP390                 │
│                                                  │
│  BMP390 (50Hz ODR, IIR Coeff 1)                 │
│    ↓ 每 50ms 读取                                  │
│  collectAndSendData()                            │
│    ↓                                             │
│  Serial: BAROD>ts,pressure,temp                  │
│         BAROT>MAC  (每秒请求时间同步)               │
└──────────────────────┬──────────────────────────┘
                       │ USB Serial (115200)
                       ▼
┌─────────────────────────────────────────────────┐
│          ROS2 Node: esp32_serial_baro            │
│                                                  │
│  serial_reader_task                              │
│    ↓                                             │
│  _process_serial_line                            │
│    ↓                                             │
│  _serial_raw_to_pressure                         │
│    ├─ [校准阶段] 收集10s样本 → 计算均值            │
│    │              → 发 /baseline_pressure        │
│    │              → 设置 default_local_pressure  │
│    │                                             │
│    └─ [正常运行] ISA海拔换算 → /barometer         │
│                             → /floor_estimate    │
│                             → 中值滤波→死区检测→ │
│                               连续确认→运动门控  │
│                             → /floor_state       │
│                             → /z_motion          │
│                                                  │
│  Publishers:                                     │
│    /pressure          (FluidPressure)            │
│    /barometer         (Barometer)                │
│    /z_motion          (ZMotion)                  │
│    /baseline_pressure (Float32, 仅一次)           │
│    /floor_estimate    (Int32, 实时)              │
│    /floor_state       (Int32, 防抖, 1Hz心跳)      │
└─────────────────────────────────────────────────┘
```
