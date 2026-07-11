# 楼层气压定位系统 —— 安装与部署指南

> 本指南分成两个场景：
> - **场景一：虚拟机仿真调试**——在电脑（或虚拟机）上把整套逻辑跑通、把参数调得差不多，
>   全程不需要碰机器人，节省来回搬机器人、坐电梯测试的时间；
> - **场景二：宇树 G1 真机部署**——把调好的代码搬到 G1 机载电脑上，装到机身上正式跑起来。
>
> **强烈建议先做完场景一，确认逻辑没问题、参数大致合理了，再进场景二**——真机调试成本
> 高得多（要实际抱着 G1 坐电梯），先在虚拟机里把能确定的问题都解决掉。

---

## 这套系统在做什么

机器人上装一颗**气压传感器（BMP390）**，接在一块 **ESP32-S3** 开发板上。电梯每上升或
下降一层楼，周围气压会发生一个很小但可测量的变化。ESP32 把气压数据通过 USB 连接线
发给运行 ROS2 的电脑（调试阶段是你的电脑/虚拟机，部署阶段是 G1 机载电脑），程序把气压
换算成"当前在第几层"，发布成一个 ROS2 话题 `/floor_state`，供其他模块订阅使用。

| 部分 | 作用 | 调试阶段跑在哪 | 部署阶段跑在哪 |
|---|---|---|---|
| `ESP32_Barometer-main` | 固件，读传感器、发数据 | ESP32 开发板 | ESP32 开发板（不变）|
| `ros_barometer-main` | 接收数据、算楼层、防抖 | 你的电脑/虚拟机 | G1 机载电脑 |

---

## 硬件清单（两个场景通用）

| 物品 | 说明 |
|---|---|
| ESP32-S3 开发板 | 带 USB-C 接口 |
| BMP390 气压传感器模块 | I2C 接口 |
| 杜邦线（母对母）| 至少 4 根 |
| USB 数据线 | 必须支持数据传输，不能是"只充电"的线 |

---

# 场景一：虚拟机仿真调试

目的：在不依赖 G1、不用反复坐电梯的情况下，先把 ROS2 节点能不能正常跑起来、防抖逻辑
是否符合预期确认清楚。ESP32 + BMP390 这一小套硬件可以先摆在桌上、拿在手上上下移动，
或者你自己拿着设备走楼梯来产生真实的气压变化。

## 1. 搭建虚拟机环境

ROS2 Humble 官方只正式支持 Ubuntu 22.04，如果你的电脑是 Windows/Mac，需要先装一个
Ubuntu 22.04 虚拟机：

1. 下载并安装 [VirtualBox](https://www.virtualbox.org/)（或 VMware）。
2. 下载 [Ubuntu 22.04 Desktop 镜像](https://ubuntu.com/download/desktop)，在虚拟机软件里
   新建一台虚拟机并装好系统（分配至少 4 核 CPU、8GB 内存、40GB 硬盘，ROS2 编译比较吃资源）。

### 关键一步：让虚拟机能"看到"ESP32（USB 直通）

这是虚拟机调试和真机部署**唯一的本质区别**——真机上 ESP32 是直接插在 G1 机载电脑的
物理 USB 口上，虚拟机里则需要手动把 USB 设备"直通"进去，否则虚拟机里的 Ubuntu 完全
看不到这个设备。以 VirtualBox 为例：

1. 关闭虚拟机，打开 VirtualBox 主界面，选中这台虚拟机 → 设置 → USB。
2. 勾选启用 USB 控制器，选择 **USB 3.0 (xHCI)**。
3. 点右侧的"+"图标，把 ESP32 开发板插到电脑上后，列表里会出现类似
   `Espressif ... USB JTAG/serial debug unit` 的设备，勾选它添加成过滤器。
4. 启动虚拟机，把 ESP32 插到宿主机 USB 口上，虚拟机里执行 `ls /dev/ttyACM*` 应该能看到
   `/dev/ttyACM0`。

> 如果启动虚拟机后看不到设备：先确认宿主机上装了 VirtualBox 的 **Extension Pack**
> （USB 3.0 直通需要它），Windows 上还需要在设备管理器里确认没有被其他驱动占用。

## 2. 接线

```
开发板 3V3   ──→ BMP390 VCC
开发板 GND   ──→ BMP390 GND
开发板 GPIO37 ──→ BMP390 SDA
开发板 GPIO38 ──→ BMP390 SCL
BMP390 CS    ──→ 接 3V3（固定为 I2C 模式）
BMP390 SDO   ──→ 接 GND（地址 0x76，固件默认按这个地址找）
```

## 3. 安装开发软件（虚拟机内）

打开虚拟机里的终端（Terminal）：

```bash
sudo apt update
sudo apt install -y git
```

安装 VS Code（<https://code.visualstudio.com/>，下载 `.deb` 双击安装），装好后在扩展商店
搜索 `PlatformIO IDE` 安装（图形界面烧录固件，不用记命令）。

安装 ROS2 Humble，按官方教程一步步做：
<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

装完后把这行加进 `~/.bashrc`，让每个新终端自动加载 ROS2 环境：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

安装 ROS2 节点依赖的 Python 库：

```bash
pip install --user pyserial pyserial-asyncio pyyaml
```

## 4. 下载代码

```bash
cd ~
git clone https://github.com/lulala5899/floor_estimate_mason.git
```

## 5. 烧录固件（图形界面）

1. VS Code 打开 `~/floor_estimate_mason/ESP32_Barometer-main` 文件夹。
2. 把 ESP32 插到电脑上（虚拟机场景记得按上面第 1 步做好 USB 直通）。
3. 点左侧蚂蚁头图标 🐜 → `barometer_node_s3 → General → Upload`，等提示 `SUCCESS`。
4. 同一个菜单里点 `Monitor` 打开串口监视器，应该能看到：
   ```
   INFO: I2C init SDA=37 SCL=38
   BAROD>1731234567890,1005.23,26.10
   ```
   看到 `BAROD>` 说明固件工作正常。**记得看完关掉监视器**，不然占用串口后面连不上。

## 6. 编译并运行 ROS2 节点

```bash
cd ~/floor_estimate_mason/ros_barometer-main
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch serial_to_ros2 baro_p_alti_launch.py
```

启动时**让设备保持静止**，等终端打印 `Baseline calibration done`，说明已经把当前位置
记为楼层 0。之后另开一个终端（记得也要 `source` 两次）：

```bash
ros2 topic echo /floor_state
```

拿着 ESP32+BMP390 走楼梯、坐电梯，或者干脆手举高低变化，观察这个数字变不变、变得
是否合理。

## 7. 楼层判断算法说明（了解这个才知道怎么调参）

`/floor_state` 现在用的是一个"稳定窗口"算法，思路很简单：

- 维护最近 `floor_stability_window`（默认 5）个 `/floor_estimate`（原始楼层号，每帧算一次，不做任何平滑）；
- 如果这个窗口里的值**全部一样** → 说明已经稳定停在某一层，直接采信这个值；
- 如果窗口里的值不完全一样（正处于跨楼层的过渡阶段）→ 保守地取窗口里的**最小值**，
  宁可判断滞后一点，也不提前宣布"到了新的一层"。

也就是说：**站定不动的时候，`/floor_state` 一定是个稳定不跳变的数；正在移动、还没站定的时候，
这个数可能会有短暂的跳动，这是设计上允许的（不影响最终停下来时的准确性）**。如果不确定
稳没稳，直接看 `/floor_estimate` 和 `/floor_state` 两个话题对比着看：前者是瞬时原始值，后者是
处理过的最终值。

同时还有一个 `mount_height_offset` 参数（默认 1.0 米）：因为传感器装在机器人身上，离地面有
一段高度，这个值会在算完海拔之后统一减掉，让基准点更贴近地面/机身实际所在的楼层，而不是
贴着传感器自己的安装高度，减少刚好卡在楼层分界线附近导致误判的概率。

## 8. 不方便反复坐电梯？用假数据脚本纯软件测试参数

如果只是想验证参数设置得合不合理，不需要每次都真的移动设备——可以写一个脚本，假装自己是
ESP32，往串口发送符合协议格式的假数据，专门测试边界情况（比如让气压在楼层分界线附近来回
抖动，看 `/floor_state` 会不会跟着乱跳）。

做法：用一对虚拟串口（`socat` 工具可以创建一对互通的虚拟串口）代替真实 ESP32，往其中一端
灌自己编的气压曲线（模拟"匀速上升两层楼后停住"之类的场景），ROS2 节点连接另一端，把
`serial_port` 参数指向这个虚拟串口即可。这样可以不依赖硬件、几秒钟内把一次电梯行程重放几
十遍，专门用来调 `floor_stability_window` 这个参数。

## 调试阶段大概率要改的参数

配置文件位置：`ros_barometer-main/serial_to_ros2/config/esp32_serial_baro.yaml`

| 参数 | 默认值 | 什么时候需要改 |
|---|---|---|
| `serial_port` | `/dev/ttyACM0` | 虚拟机里 USB 直通后设备名可能变成 `ttyACM1` 等，用 `ls /dev/ttyACM*` 确认 |
| `floor_height` | `3.0` | 用实际测试环境的层高替换（如果暂时没有真实电梯环境，先随便定一个值，后面在真机场景重新标定）|
| `mount_height_offset` | `1.0` | 按传感器实际安装离地/离机身参考点的高度改，越准，越不容易在楼层分界线附近误判 |
| `calibration_duration` | `10.0` | 调试阶段建议先调小（比如 3～5 秒），这样每次重启节点不用等 10 秒，反复测试更快；调试完成后再改回 10 秒左右保证真实精度 |
| `floor_stability_window` | `5` | 如果发现楼层反应太慢，调小；如果发现停稳了数值还在跳，调大——这是现在调试阶段最值得反复试的参数 |

可以不改配置文件，用命令行临时覆盖某个参数快速试验：

```bash
ros2 run serial_to_ros2 esp32_serial_baro --ros-args -p floor_stability_window:=8
```

---

# 场景二：部署到宇树 G1 真机

场景一里代码逻辑和参数已经基本调好，这一步只是**换个地方跑**——把同一套代码搬到
G1 机载电脑上，重新过一遍编译流程（不同电脑要重新编译，不能直接拷贝编译产物），
然后针对真实电梯环境做最后的参数微调。

## 1. G1 机载电脑基本信息

G1 一般有两个板载计算单元：**运控计算单元**（宇树运动控制专用，不对开发者开放）和
**开发计算单元**（PC2，留给你做二次开发）。常见默认信息如下（**不同批次/不同经销商
定制的机器可能不一样，务必先以你自己那台 G1 附带的文档为准**）：

| 项目 | 常见默认值 |
|---|---|
| 开发计算单元 IP | `192.168.123.164` |
| 用户名 | `unitree` |
| 密码 | `123` |

## 2. 连接到 G1 机载电脑

用网线把电脑和 G1 背部/身上的网口连起来，把你电脑的有线网卡设成同一网段的静态 IP
（比如 `192.168.123.200`，注意不能和 G1 已用的地址冲突）：

```bash
# 把 eth0 换成你实际的网卡名（用 ip a 查看）
sudo ip addr flush dev eth0
sudo ip addr add 192.168.123.200/24 dev eth0
sudo ip link set eth0 up
```

确认能连通：

```bash
ping 192.168.123.164
```

能 ping 通之后，用 SSH 连上去：

```bash
ssh unitree@192.168.123.164
```

（VS Code 也可以装 **Remote - SSH** 插件，直接图形界面远程编辑/调试 G1 上的代码，
比纯终端操作方便很多，推荐用这个方式而不是全程 SSH 命令行。）

## 3. 把代码传到 G1 上

两种方式二选一：

**方式 A：G1 机载电脑能联网** —— 直接在 G1 上执行和场景一相同的
`git clone https://github.com/lulala5899/floor_estimate_mason.git`。

**方式 B：G1 机载电脑没有外网** —— 从你自己电脑用 `scp` 把整个文件夹传过去：

```bash
scp -r ~/floor_estimate_mason unitree@192.168.123.164:~/
```

## 4. 确认 G1 上的 ROS2 环境

⚠️ **这是真机部署阶段最容易踩坑的地方**：不同 G1 出厂配置上预装的 ROS2 版本不一定
是 Humble（比如某些第三方集成商预装的是 Foxy/Ubuntu 20.04）。SSH 上去之后先确认：

```bash
ls /opt/ros/
```

看到哪个版本目录（`humble`、`foxy` 等），后面所有 `source /opt/ros/xxx/setup.bash` 命令
里的版本号都要换成实际看到的这个，不能照抄场景一虚拟机里的 `humble`。

## 5. 硬件安装到机身上的注意事项

- **气压计要做防风处理**：G1 身上有散热风扇、关节运动带来的局部气流，会直接干扰
  BMP390 的读数。建议给传感器进气孔盖一层薄海绵/无纺布做缓冲，而不是裸露安装。
- **USB 口选择要固定**：G1 上通常已经接了雷达、相机等好几个 USB 设备，ESP32 插的
  端口在系统重启后编号（`/dev/ttyACM0` / `ttyACM1`）可能会变。建议加一条 udev 规则，
  把 ESP32 按其硬件序列号固定成一个不会变的设备名（比如 `/dev/ttyACM_baro`），
  这样 `esp32_serial_baro.yaml` 里的 `serial_port` 只需要配一次，以后不用每次开机确认。
- **走线要固定**：USB 线要用扎带固定好，避免机器人运动时被拉扯松动。

## 6. 在 G1 上编译并运行（和场景一相同，路径和 ROS2 版本号自己替换）

```bash
cd ~/floor_estimate_mason/ros_barometer-main
source /opt/ros/<你的ROS2版本>/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch serial_to_ros2 baro_p_alti_launch.py
```

第一次跑的时候，让机器人先站在你希望作为"起始楼层"（比如大厅）静止不动，等日志打出
`Baseline calibration done` 再开始移动/坐电梯。

## 7. 让节点开机自动运行（不用每次都手动 SSH 上去跑）

给 `ros2 launch ...` 那条命令写一个 systemd 服务，这样 G1 开机（或机载电脑重启）后会
自动拉起这个节点：

```ini
# /etc/systemd/system/floor-estimate.service
[Unit]
Description=Floor estimate ROS2 node
After=network.target

[Service]
User=unitree
ExecStart=/bin/bash -c "source /opt/ros/<你的ROS2版本>/setup.bash && \
    source /home/unitree/floor_estimate_mason/ros_barometer-main/install/setup.bash && \
    ros2 launch serial_to_ros2 baro_p_alti_launch.py"
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now floor-estimate.service
# 查看运行状态/日志
sudo systemctl status floor-estimate.service
journalctl -u floor-estimate.service -f
```

> 如果 G1 本身有 Webserver/仪表盘来管理 ROS2 服务（部分集成商版本会提供），也可以
> 直接把这个 launch 文件加进它的服务列表里管理，效果和 systemd 是一样的，看你那台
> G1 具体带的是哪种管理方式。

## 8. 搬到真机之后，大概率还要重新调的参数

虚拟机里调好的参数不能直接照搬，主要因为这几点环境差异：

| 参数 | 为什么真机上通常要重调 |
|---|---|
| `floor_stability_window` | G1 自身的振动、风扇气流会引入虚拟机场景里没有的额外噪声，如果发现停稳之后数值还偶尔跳一下，适当调大这个窗口 |
| `mount_height_offset` | 换算成 G1 上气压计实际安装点离地面的高度，虚拟机调试阶段这个值可能只是随便设的 |
| `floor_height` | 用实际要部署的那栋楼的真实层高替换调试阶段的占位值 |
| `calibration_duration` | 调试阶段为了测试方便调小过的话，真机上建议调回 10 秒左右，保证基准气压足够准 |
| `serial_port` | 见上面"USB 口固定"那条，用 udev 规则配好的固定设备名 |

调参方式和场景一相同，改 `ros_barometer-main/serial_to_ros2/config/esp32_serial_baro.yaml`
这一个文件即可，改完重启 `floor-estimate.service`（或重新 `ros2 launch`）生效。

---

## 常见问题

**Q: 虚拟机里 `ls /dev/ttyACM*` 什么都没有**
A: 检查 VirtualBox 的 USB 直通是否配置成功（见场景一第 1 步），以及 Extension Pack
是否安装。

**Q: SSH 不上 G1（192.168.123.164）**
A: 先 `ping` 确认网络通不通；确认你电脑的静态 IP 和 G1 在同一网段但不是同一个地址；
确认用的是网线连接（无线一般默认是关闭的）。

**Q: G1 上 `colcon build` 报错找不到 `barometer_interfaces`**
A: 和虚拟机场景一样，先单独编译消息包：
```bash
colcon build --packages-select barometer_interfaces
source install/setup.bash
colcon build --symlink-install
```

**Q: 楼层号在真机上比虚拟机测试时更容易乱跳**
A: 大概率是风扇/振动噪声比预期大，按上面第 8 点先调大 `floor_stability_window`，
再检查气压计有没有做防风处理（进气孔加海绵/无纺布）。

**Q: `/floor_estimate` 和 `/floor_state` 该用哪个？**
A: 给其他模块用请用 `/floor_state`——机器人站定不动时这个值是稳定不跳变的，正在移动、
还没停稳的过程中可能会有短暂跳动，这是正常现象。`/floor_estimate` 是每一帧的瞬时原始值，
只用来调试对比，不建议直接给其他模块用。

**Q: 为什么之前设计的"运动门控"（判断电梯是否已经停止再锁定楼层）被去掉了？**
A: 那套逻辑依赖对原始高度逐帧差分算出的"瞬时速度"，气压传感器本身的噪声一放大就很不
稳定，导致"是否已停止"这个判断经常测不准，`/floor_state` 因此长期锁死在初始值不更新。
现在改用的"稳定窗口"算法不依赖这个不可靠的速度估计，只看最近几帧原始楼层号是否一致，
逻辑更简单也更可靠。
