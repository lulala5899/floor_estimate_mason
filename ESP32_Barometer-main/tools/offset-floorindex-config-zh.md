# 论文 2601.02184 配置文档（中文）

## 1. 文档目的

本配置文档用于说明当前项目中“差分气压楼层估计”这部分的固定配置与运行约定，覆盖：

- 偏置校准（Eq.(4)、Eq.(8)、Eq.(9)）
- 运行时 offset 的写法与含义
- 差分高度 \(\Delta h\) 的计算与验收
- 楼层索引（C. Floor Indexing）配置
- 周期重标定建议

## 2. 当前生效配置

### 2.1 传感器 MAC 与偏置参数

当前写入 YAML 的配置如下（运行时采用 `corrected = raw + offset`）：

- 记号约定：
  - mobile 传感器记为 \(m\)，base 传感器记为 \(b\)
  - 压力 offset 记为 \(o^{(p)}\)，温度 offset 记为 \(o^{(T)}\)
- 本次参数：
  - \(o_{m}^{(p)} = -0.0466 \ \mathrm{hPa}\), \(o_{m}^{(T)} = -0.8014 \ ^\circ\mathrm{C}\)
  - \(o_{b}^{(p)} = +0.0466 \ \mathrm{hPa}\), \(o_{b}^{(T)} = +0.8014 \ ^\circ\mathrm{C}\)
- YAML 键映射：
  - \(o^{(p)} \leftrightarrow\) `pressure_offset`
  - \(o^{(T)} \leftrightarrow\) `temperature_offset`

对应文件：

- `ros_barometer-main/serial_to_ros2/config/esp32_serial_baro.yaml`

备份文件：

- `esp32_serial_baro.yaml.bak.20260421_154140`

### 2.2 楼层索引参数（按当前要求）

- 每层高度：`3 m`
- 楼层数：`5 层`
- 楼层表：\(H_k = 3k \ \mathrm{m}, \ k \in \{0,1,2,3,4\}\)（即 \([0,3,6,9,12]\)）

## 3. 偏置参数是如何得到的

使用共址静态录包数据，按论文流程：

1. 30 秒重采样（每个传感器各自平均）
2. 两路 inner join 对齐
3. Eq.(4) 跳变过滤（\(|\Delta P|<=1 hPa, |\Delta T|<=1 C\)）
4. Eq.(8)(9) 估计 \(\hat{\beta}\)
5. 运行时换算 \(o = -\hat{\beta}\)

本次标定结果（paper biases）：

- \(\hat{\beta}_{m}^{(p)} = +0.046647 \ \mathrm{hPa}\)
- \(\hat{\beta}_{m}^{(T)} = +0.801433 \ ^\circ\mathrm{C}\)
- \(\hat{\beta}_{b}^{(p)} = -0.046647 \ \mathrm{hPa}\)
- \(\hat{\beta}_{b}^{(T)} = -0.801433 \ ^\circ\mathrm{C}\)

因此写入 offset（取负号）：

- \(o_{m}^{(p)} = -0.0466,\ o_{m}^{(T)} = -0.8014\)
- \(o_{b}^{(p)} = +0.0466,\ o_{b}^{(T)} = +0.8014\)

## 4. ISA 常数配置说明

当前采用默认 ISA 常数：

- `L=0.0065`
- `R=287.05`
- `g=9.80665`

这与论文结论一致：在输出相对高度差 \(\Delta h\) 的场景下，默认 `(L,R,g)` 引起的相对高度误差远小于噪声水平（论文指出在工作范围内 < 2 cm 量级），通常无需专门调 `(L,R,g)`。

## 5. 差分高度与楼层索引

### 5.1 差分高度

\[
\Delta h = h_{\mathrm{mobile}} - h_{\mathrm{base}}
\]

共址静态时理想目标：\(\Delta h \approx 0\)。

### 5.2 楼层索引（C. Floor Indexing）

\[
\ell = \arg\min_{k \in \{0,1,\dots,K-1\}} \left| \Delta h - H_k \right|
\]

其中：

- \(H_k\) 为楼层高度表（本项目当前为 `[0,3,6,9,12]`）
- \(\ell\) 为预测楼层索引（0~4）

可同时输出置信误差：

\[
e = \min_{k \in \{0,1,\dots,K-1\}} \left| \Delta h - H_k \right|
\]

`e` 越小表示越接近某个楼层高度。

## 6. 运行与测试建议

1. 先保证两块板在线，ROS 话题可见：
   - `/barometer`
   - `/base/barometer`
2. 运行实时验收脚本，检查：
   - `dp/dT/dh` 统计
   - `floor_index` 分布
   - `nearest_error`（楼层最近邻误差）
3. 共址静态时，期望：
   - `dh_mean` 接近 0
   - floor 多数在 `0 层`

## 7. 周期重标定（论文要求）

偏置会随传感器老化/磨损漂移，应周期重估。建议周期：

- 研发阶段：每次重要实验前
- 部署阶段：每 1~2 周（或温湿度环境明显变化后）

固定流程：

1. 共址静态录包
2. 跑 Eq.(4),(8),(9) 校准
3. 更新 YAML
4. 重启 ROS 节点
5. 做 2~5 分钟静态验收
