# floor_estimate_mason

基于 **ESP32-S3 + BMP390** 的气压测楼层方案（单板模式）。

电梯内无 Wi-Fi 信号，双板差分不可行，故本仓库专注于 **单板 self-relative 模式**：一块 ESP32 通过 USB 串口将气压数据发送到电脑，电脑端 ROS2 节点根据本地参考气压换算海拔，实时输出楼层估计。

---

## 硬件清单

| 组件 | 说明 |
|------|------|
| ESP32-S3 开发板 | USB 串口设备一般为 `/dev/ttyACM0` |
| BMP390 气压传感器模块 | I2C 接口 |
| 杜邦线 | 母对母，4 条 |

---

## 1. 接线

```
3V3  -> BMP390 VCC
GND  -> BMP390 GND
GPIO41 -> BMP390 SDA
GPIO42 -> BMP390 SCL
CS   -> 3V3（固定 I2C 模式）
SDO  -> GND（地址 0x76）或 3V3（地址 0x77）
```

> 如果只有一个 3V3/GND 引脚，可用杜邦线并联给 VCC/CS 和 GND/SDO。

---

## 2. 安装依赖

### 2.1 PlatformIO（固件编译）

```bash
python3 -m pip install --user -U platformio
```

### 2.2 ROS2 Humble

确保已安装 ROS2 Humble，并 source 环境：

```bash
source /opt/ros/humble/setup.bash
```

---

## 3. 固件编译与烧录

工程位置：`ESP32_Barometer-main/`

关键配置已在 `platformio.ini` 中预设：

- 环境：`barometer_node_s3`（单板串口输出）
- I2C 引脚：SDA=41, SCL=42
- USB CDC 模式已启用（Serial 日志走 `/dev/ttyACM0`）

检测串口设备：

```bash
ls /dev/ttyACM* 2>/dev/null
```

编译：

```bash
cd ESP32_Barometer-main
python3 -m platformio run -e barometer_node_s3
```

烧录：

```bash
python3 -m platformio run -e barometer_node_s3 -t upload --upload-port /dev/ttyACM0
```

---

## 4. 串口验证

```bash
python3 -m platformio device monitor -p /dev/ttyACM0 -b 115200
```

期望输出格式：

```
BAROD>timestamp,pressure_hpa,temp_c
BAROT><ESP32_MAC>
```

实测样例：

```
BAROD>...,1006.04,28.04
BAROT>24:EC:4A:01:43:20
```

---

## 5. 时间同步（可选但推荐）

向 ESP32 发送一次时间同步，使输出的时间戳为 Unix 毫秒时间：

```bash
python3 - <<'PY'
import serial, time
port = '/dev/ttyACM0'
now_ms = int(time.time() * 1000)
with serial.Serial(port, 115200, timeout=1) as s:
    s.write(f'TS>{now_ms}\n'.encode())
    s.flush()
    for _ in range(8):
        line = s.readline().decode(errors='ignore').strip()
        if line:
            print(line)
PY
```

期望看到 `DEBUG: Received time sync request: TS>...`，此后 `BAROD>` 行的时间戳变为正常的毫秒时间。

---

## 6. ROS2 节点：串口桥接

### 6.1 构建 ROS 工作区

```bash
source /opt/ros/humble/setup.bash
cd ros_barometer-main
colcon build --symlink-install
source install/setup.bash
```

### 6.2 启动串口桥接节点

单板模式使用 `output_mode:=self-relative`，并指定当地参考气压：

```bash
ros2 run serial_to_ros2 esp32_serial_baro \
    --ros-args \
    -p serial_port:=/dev/ttyACM0 \
    -p output_mode:=self-relative \
    -p default_local_pressure:=1005.0
```

> `default_local_pressure` 的值填你所在楼层实测的稳定气压（单位 hPa）。

### 6.3 查看话题数据

新终端：

```bash
source /opt/ros/humble/setup.bash
source ros_barometer-main/install/setup.bash
ros2 topic echo /barometer --once
```

期望输出：

```
header:
  stamp:
    sec: 1775629956
    nanosec: 165000000
  frame_id: barometer_link
altitude: 79.3833
pressure: 100564.0
temperature: 29.49
```

---

## 7. 确定参考气压

在**已知楼层**（如 1 楼）观察气压稳定值：

```bash
ros2 topic echo /barometer --once | grep pressure
```

例如读到 `1006.04 hPa`，则重启桥接节点时填入：

```bash
ros2 run serial_to_ros2 esp32_serial_baro \
    --ros-args \
    -p serial_port:=/dev/ttyACM0 \
    -p output_mode:=self-relative \
    -p default_local_pressure:=1006.04
```

此时话题中的 `altitude` 即为相对于参考楼层的海拔差（单位米）。

---

## 8. 实时楼层验证

启动桥接节点后，另开终端运行楼层预测脚本：

```bash
source /opt/ros/humble/setup.bash
source ros_barometer-main/install/setup.bash

python3 ESP32_Barometer-main/tools/realtime_floor_validation.py \
    --duration 120 \
    --live-interval 1 \
    --floor-height 3 \
    --floor-count 5
```

参数说明：

| 参数 | 含义 |
|------|------|
| `--duration` | 采集时长（秒） |
| `--live-interval` | 输出间隔（秒） |
| `--floor-height` | 每层楼高度（米） |
| `--floor-count` | 总楼层数 |

脚本会订阅 `/barometer` 话题，根据 `altitude` 实时推算当前楼层并输出结果。

---

## 系统架构（单板）

```
ESP32-S3 + BMP390
    │
    ├─ USB Serial (/dev/ttyACM0)
    │
    ▼
esp32_serial_baro (ROS2 node)
    │
    ├─ output_mode=self-relative
    ├─ default_local_pressure=<参考气压>
    │
    ▼
/barometer topic
    │
    ▼
realtime_floor_validation.py
    │
    ▼
楼层估计结果
```

---

## 常见问题

**Q: 串口设备不是 `/dev/ttyACM0`？**
A: 用 `ls /dev/ttyACM*` 或 `ls /dev/ttyUSB*` 查看实际设备名，在启动桥接节点时修改 `serial_port` 参数。

**Q: 参考气压怎么获取最准确？**
A: 将设备放在已知楼层静止 1-2 分钟，取 `ros2 topic echo /barometer --once` 中 `pressure` 的稳定值。

**Q: ROS2 Humble 不是我的版本？**
A: 把命令中的 `humble` 替换为你的 ROS2 版本名即可（如 `jazzy`、`rolling`）。
