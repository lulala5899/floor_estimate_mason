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

同时还有一个 `mount_height_offset` 参数，**默认应该保持 `0.0`**：这套系统是"自校准相对高度"，
开机那一刻的气压就被直接定义成海拔 0，跟传感器装在机器人身上多高、离地面多远完全没有关系——
机器人不管是 1 米高还是 3 米高，从一层坐到二层，气压升高的量都是同一个楼层高度，不会因为机身
高度不同而变化。

这个参数唯一的作用，是把"开机基准点=0"这个人为设定的原点往上/往下平移。**平移之后，上楼和
下楼两个方向的判定门槛会此消彼长**（比如设成 1.0，上楼要多爬 1 米才判定换层，下楼只要少降
1 米就会判定换层），两个门槛之和永远等于 `floor_height`（默认 3.0m）这个常数，不存在任何一个
偏移值能让两个方向同时变宽。所以除非你有意需要"上楼更保守、下楼更灵敏"这种非对称效果，否则
保持 `0.0` 是唯一能让上下楼判定同样稳（各留一半余量）的取值，**跟机器人实际身高无关，不需要
按机身高度去设**。

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
| `mount_height_offset` | `0.0` | 一般不需要动，保持默认能让上下楼判定门槛对称、都最不容易误判；只有你**故意**想要"上楼保守、下楼灵敏"（或反过来）这种非对称效果时才调它，不是按机身高度来设 |
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

## 2. 把电脑和 G1 连起来

这一步是让你的电脑和 G1 的机载电脑能够"对话"，用的是最基础的有线网络连接，不涉及无线网络。

### 2.1 物理连接

找一根**网线**，一头插进你电脑的网口（如果是笔记本没有网口，需要一个 USB 转网口的
转接器），另一头插进 G1 身上的网口（一般在背部或者身侧，具体位置看你那台机器的说明书，
如果找不到可以问一下卖给你机器的人或者厂家客服）。

### 2.2 什么是"网段"、为什么要设静态 IP

简单理解：两台设备要能互相通信，必须处于同一个"网段"里，可以类比成"同一个门牌号码段"。
G1 的机载电脑固定用的地址是 `192.168.123.164`，所以你的电脑也必须手动设置成
`192.168.123.` 开头的一个地址（但最后一段数字不能和 G1 一样，比如用 `.200`），
这样两边才能找到对方。默认情况下你的电脑网卡不会自动配成这个地址，需要手动设一下。

### 2.3 先确认你的网卡叫什么名字

在 Ubuntu 虚拟机（或者你直接用来连 G1 的那台电脑）终端里执行：

```bash
ip a
```

会列出这台电脑上所有的网络接口，找**你刚才插网线的那个接口**，名字一般类似
`eth0`、`enp0s3`、`enx...` 这种（前面几个字母是 `eth` 或 `en` 开头的基本都是有线网卡）。
记住这个名字，下面命令里的 `eth0` 都要换成你实际看到的这个名字。

> 如果你是在 VMware 虚拟机里操作，别忘了这根网线也要走一遍之前给 ESP32 做的那套
> USB/网络"连接给虚拟机"的流程——虚拟机的虚拟网卡设置里，网络连接方式建议选
> "桥接模式（Bridged）"，这样虚拟机才能和物理网线直接通信，选"NAT"模式可能连不上。

### 2.4 给网卡设置静态 IP

```bash
sudo ip addr flush dev eth0
sudo ip addr add 192.168.123.200/24 dev eth0
sudo ip link set eth0 up
```

这三行依次是：清空这张网卡原来乱七八糟的地址设置 → 给它指定一个新地址
`192.168.123.200` → 启用这张网卡。执行的时候如果要求输入密码，是要你输入当前
Ubuntu 用户的登录密码（虚拟机安装时你自己设的那个），不是 G1 的密码。

> 这个设置只在当前这次开机期间有效，虚拟机重启后需要重新执行一遍。如果你打算长期这样用，
> 可以搜"Ubuntu 静态 IP netplan 配置"了解怎么把这个设置固化下来，不是必须现在就做。

### 2.5 确认两边通了

```bash
ping 192.168.123.164
```

执行后应该看到类似下面这样持续刷新的输出（按 `Ctrl+C` 停止）：

```
64 bytes from 192.168.123.164: icmp_seq=1 ttl=64 time=0.523 ms
64 bytes from 192.168.123.164: icmp_seq=2 ttl=64 time=0.412 ms
```

看到这样持续有回应，说明网络通了，可以进行下一步。**如果一直显示
`Destination Host Unreachable` 或者没有任何回应**，按顺序排查：
- 网线有没有真的插紧、两头都插对了口；
- 上面 `ip a` 查到的网卡名有没有填对；
- G1 本身有没有开机（机载电脑通电需要一点时间启动，刚开机可能要等一两分钟）；
- 虚拟机的网络连接模式是不是"桥接模式"。

### 2.6 连接进 G1 的系统（SSH 是什么）

SSH 是一种"远程登录"方式，效果就是让你在自己电脑的终端里，操作的其实是 G1 机载
电脑里的系统，输入的命令都是在 G1 上执行，不是在你自己电脑上执行——这是接下来所有
操作的基础，务必先搞清楚这个概念，不然容易搞混"我现在敲的命令到底是在哪台电脑上跑"。

```bash
ssh unitree@192.168.123.164
```

第一次连接会提示一段类似"能否信任这台主机的指纹"的英文提示，输入 `yes` 回车。
接着会要求输入密码，输入 `123`（输入密码的时候屏幕上**不会显示任何字符**，这是
正常的安全设计，不是卡住了，正常打完直接回车即可）。

连接成功之后，你会发现终端提示符变了（通常会显示 `unitree@xxx:~$` 这样的样子，
和之前 `ros@ros-virtual-machine:~$` 不一样），这就表示你现在操作的是 G1 内部的系统了。

> **强烈建议**：装一下 VS Code 的 **Remote - SSH** 插件（在 VS Code 扩展商店搜索
> `Remote - SSH` 安装），装好之后可以直接在 VS Code 里输入 `192.168.123.164`
> 图形化连接进 G1，像操作自己电脑一样浏览、编辑 G1 里的文件，比纯命令行方便很多，
> 尤其是后面要改配置文件的时候。

## 3. 把代码传到 G1 上

这一步开始，**如果你是用纯终端 SSH 连接**，注意分清楚自己当前是在哪台电脑的终端里
（是你自己电脑，还是已经 SSH 进了 G1）——克隆代码这个操作，是要在 **G1 的系统里**执行的。

两种方式二选一：

**方式 A：G1 机载电脑能连外网**（先确认一下，在 SSH 进 G1 之后执行 `ping baidu.com`
试试，能通就是能联网）：

```bash
cd ~
git clone https://github.com/lulala5899/floor_estimate_mason.git
```

**方式 B：G1 机载电脑没有外网**（大多数情况是这样，机器人内网一般不直接连外网）——
从你**自己的电脑**（不是 G1 里）用 `scp` 命令把整个文件夹直接传过去：

```bash
scp -r ~/floor_estimate_mason unitree@192.168.123.164:~/
```

`scp` 这条命令是在你自己电脑的终端里执行的（不是 SSH 进去之后执行），效果是把你自己
电脑上 `~/floor_estimate_mason` 这整个文件夹，原样拷贝一份到 G1 的 `unitree` 用户家目录下。
执行时同样会要求输入密码 `123`。传完之后可以 SSH 进 G1 执行 `ls ~` 确认
`floor_estimate_mason` 这个文件夹已经在里面了。

## 4. 确认 G1 上的 ROS2 环境

⚠️ **这是真机部署阶段最容易踩坑的地方，务必先做这一步再往下**：不同 G1 出厂配置上
预装的 ROS2 版本不一定是 Humble（比如某些第三方集成商预装的是 Foxy，对应 Ubuntu 20.04），
虚拟机里用惯的 `source /opt/ros/humble/setup.bash` 这条命令，在 G1 上如果版本不对，
会直接报"文件不存在"。

SSH 进 G1 之后执行：

```bash
ls /opt/ros/
```

这条命令会列出这台电脑上装了哪个/哪些 ROS2 版本，输出可能是 `humble`，也可能是
`foxy`、`galactic` 之类别的名字。**记住这个名字**，后面所有命令里凡是写
`/opt/ros/<你的ROS2版本>/setup.bash` 的地方，都要把 `<你的ROS2版本>` 替换成这里实际
看到的名字（比如看到的是 `foxy`，后面就都写 `/opt/ros/foxy/setup.bash`）。

如果这个目录是空的、或者压根没有 `/opt/ros` 这个目录，说明 G1 这台机载电脑上还没装
ROS2，需要先按官方教程装一遍（参考场景一第 3 步里给的那个官方安装教程链接，把里面
的 `humble` 换成 G1 系统对应的 ROS2 版本），这种情况建议先找一下你那台 G1 随附的开发
文档确认清楚，不同集成商配置差异比较大。

## 5. 把硬件装到机身上之前，检查这几点

这一步是纯硬件操作，先不急着接线，看完注意事项再动手：

- **气压计要做防风处理**：G1 身上有散热风扇，走路/关节转动时也会带来局部气流，这些
  气流会直接吹到 BMP390 的进气孔上，干扰读数。装之前找一小块薄海绵或者无纺布（口罩
  内层那种材质就行），轻轻盖在传感器的进气孔上面，不要完全堵死，只是起缓冲作用。
- **别把传感器紧贴在风扇排风口或者关节缝隙旁边**：尽量找机身上相对独立、气流干扰小
  的位置安装，比如躯干内部靠上的空间。
- **USB 线要用扎带固定**，从 ESP32 到 G1 机身接口沿途至少固定两三个点，避免机器人走动
  时线被来回拉扯导致接触不良（虚拟机调试阶段遇到的 I2C 断线问题，在真机上如果线材固定
  不好会更容易复现）。
- **确认 ESP32 插的是哪个 USB 口**：G1 身上通常还接了雷达、摄像头等其他 USB 设备，装的
  时候记一下你把 ESP32 插在了哪个物理接口上，方便后面万一要重新拔插时能找回同一个口。

硬件装好、确认接线牢固之后，再继续下一步。

## 6. 在 G1 上编译并运行

这一步操作和场景一基本一样，**唯一区别是要把 ROS2 版本号换成你在第 4 步查到的实际
版本**（下面命令里 `<你的ROS2版本>` 这几个字都要替换掉，不能照抄）。确认自己是通过
SSH（或 VS Code Remote-SSH）连在 G1 上再执行：

```bash
cd ~/floor_estimate_mason/ros_barometer-main
source /opt/ros/<你的ROS2版本>/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch serial_to_ros2 baro_p_alti_launch.py
```

和虚拟机阶段一样，第一次跑的时候**让机器人先站在你希望作为"起始楼层"的地方（比如
大厅）静止不动**，等终端打出 `Baseline calibration done` 之后再开始移动/坐电梯，
这一步决定了后面所有楼层号都是"相对这个起始位置"计算的。

跑起来之后，同样可以照场景一的方法，另开一个终端（同样要先 SSH 连进 G1，再执行两遍
`source`）去 `ros2 topic echo /floor_state` 观察输出，确认真机上也能正常工作。

## 7. 让节点开机自动运行（不用每次都手动连上去敲命令）

前面每次都要手动 SSH 进去、敲一遍编译运行的命令，比较麻烦。这一步是配置一个
**systemd 服务**——你可以把它理解成"开机自动启动的后台任务"，配置好之后，以后
G1（或者说它的机载电脑）每次开机，这个楼层检测程序会自动跑起来，不需要你再手动连上去操作。

### 7.1 创建服务配置文件

SSH 连进 G1，执行：

```bash
sudo nano /etc/systemd/system/floor-estimate.service
```

会打开一个空白的文本编辑器（`nano` 是一个简单的命令行文本编辑器），把下面的内容
**完整粘贴进去**（记得把里面两处 `<你的ROS2版本>` 换成第 4 步查到的实际版本名）：

```ini
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

粘贴完之后，按 `Ctrl+O` 保存（会在底部提示确认文件名，直接回车确认），再按
`Ctrl+X` 退出编辑器。

### 7.2 启用这个服务

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now floor-estimate.service
```

第一条是让系统重新读取一遍服务配置，第二条是"设置成开机自动启动"并且"立刻启动一次"。

### 7.3 确认它真的在正常运行

```bash
sudo systemctl status floor-estimate.service
```

正常的话，输出里应该能看到一行绿色的 `active (running)` 字样。如果看到的是红色的
`failed` 或者 `inactive`，说明启动失败了，用下面这条命令看具体的运行日志、找报错原因
（这条命令会持续刷新最新日志，按 `Ctrl+C` 退出）：

```bash
journalctl -u floor-estimate.service -f
```

常见的失败原因是 `<你的ROS2版本>` 或者路径没替换对，仔细检查一下第 7.1 步粘贴的内容。

### 7.4 以后怎么管理这个服务

```bash
sudo systemctl stop floor-estimate.service      # 手动停止
sudo systemctl start floor-estimate.service     # 手动启动
sudo systemctl restart floor-estimate.service   # 改完配置文件（比如yaml参数）后，重启让改动生效
sudo systemctl disable floor-estimate.service   # 取消开机自启（如果以后不想要了）
```

> 如果 G1 本身带有 Web 管理界面/仪表盘来管理机载程序（部分集成商版本会提供），也可以
> 直接把这个 launch 命令加进它的服务列表里管理，效果和这里配置 systemd 是一样的，
> 看你那台 G1 具体带的是哪种管理方式，两者选一种即可，不用重复配置。

## 验收清单（做完这几条，说明真机部署成功了）

- [ ] `ssh unitree@192.168.123.164` 能正常连上
- [ ] `~/floor_estimate_mason` 文件夹已经在 G1 上
- [ ] `colcon build --symlink-install` 编译没有报错
- [ ] 手动执行 `ros2 launch ...` 能看到 `Baseline calibration done`
- [ ] 另开终端 `ros2 topic echo /floor_state` 能看到数值，且机器人坐电梯移动后数值会变化
- [ ] `sudo systemctl status floor-estimate.service` 显示 `active (running)`
- [ ] 重启一次 G1 机载电脑（`sudo reboot`），不用手动操作，服务自动又跑起来了

## 8. 搬到真机之后，大概率还要重新调的参数

虚拟机里调好的参数不能直接照搬，主要因为这几点环境差异：

| 参数 | 为什么真机上通常要重调 |
|---|---|
| `floor_stability_window` | G1 自身的振动、风扇气流会引入虚拟机场景里没有的额外噪声，如果发现停稳之后数值还偶尔跳一下，适当调大这个窗口 |
| `mount_height_offset` | 一般不需要因为换到真机就重调——这个参数跟机身/传感器实际安装高度无关，保持虚拟机阶段验证过的 `0.0` 即可（除非你调试阶段临时改动过又忘了改回来，去 yaml 里确认一下） |
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
