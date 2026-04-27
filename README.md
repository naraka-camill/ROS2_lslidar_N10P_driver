# ROS2 LiDAR 工作空间

本 ROS 2 工作空间集成了 **Lslidar_ROS2_driver-M10P-N10P** 驱动包，用于雷神智能系统（Leishen Intelligent System）的激光雷达传感器，支持 M10、M10_GPS、M10_P、M10_PLUS、N10、L10 和 N10_P 系列。


## 前言（开发细节）
```bash
ros2 interface show sensor_msgs/msg/LaserScan
# 来自平面激光测距仪的单次扫描数据
#
# 如果你有其他测距设备且行为不同（例如声呐阵列），请查找或创建一个不同的消息类型，
# 因为应用程序会根据此数据做出相当依赖激光特定假设的处理

std_msgs/Header header # 消息头中的时间戳是该次扫描中第一条射线的采集时间
        builtin_interfaces/Time stamp
                int32 sec        # 秒
                uint32 nanosec   # 纳秒
        string frame_id          # 坐标系ID
                                 # 扫描数据所在坐标系中的第一条射线
                                 #
                                 # 在 frame_id 坐标系中，角度围绕 Z 轴正方向测量
                                 # （若 Z 轴向上，则为逆时针方向）
                                 # 零度角对应沿 X 轴正方向（朝前）

float32 angle_min            # 扫描起始角度 [弧度]
float32 angle_max            # 扫描终止角度 [弧度]
float32 angle_increment      # 相邻测量点之间的角度增量 [弧度]

float32 time_increment       # 相邻测量点之间的时间间隔 [秒]
                             # 如果扫描仪正在移动，此参数将用于插值计算3D点的位置
float32 scan_time            # 完成一次完整扫描所需时间 [秒]

float32 range_min            # 有效测距最小值 [米]
float32 range_max            # 有效测距最大值 [米]

float32[] ranges             # 距离数据数组 [米]
                             # （注意：小于 range_min 或大于 range_max 的值应被丢弃）
float32[] intensities        # 强度数据数组 [设备相关单位]
                             # 如果设备不提供强度数据，请将此数组留空
```

## 概述

工作空间在 `src/Lslidar_ROS2_driver-M10P-N10P/` 下包含两个 ROS 2 包：

| 包名 | 描述 |
|------|------|
| `lslidar_driver` | ROS 2 激光雷达设备驱动。支持以太网（UDP）和串口通信，具备点云生成、角度裁剪、距离过滤、pcap 回放和雷达启停控制功能。 |
| `lslidar_msgs` | 雷神激光雷达数据类型的自定义 ROS 2 消息定义（包括 packet、scan、sweep、point 和 DIFOP）。 |

## 支持的雷达型号

- **M10** — 标准 M10 系列
- **M10_GPS** — 带 GPS 时间戳支持的 M10
- **M10_P** — M10-P 系列（支持高反射模式）
- **M10_PLUS** — M10-PLUS 系列
- **N10** — N10 系列
- **N10_P** — N10-P 系列
- **L10** — L10 系列

## 环境要求

### 已测试环境
- Ubuntu 20.04 + ROS 2 Foxy
- Ubuntu 22.04 + ROS 2 Humble

### 依赖项

| 依赖项 | 用途 |
|--------|------|
| `rclcpp` | ROS 2 C++ 客户端库 |
| `rclpy` | ROS 2 Python 客户端库 |
| `std_msgs` | 标准 ROS 2 消息 |
| `sensor_msgs` | 传感器数据类型（LaserScan、PointCloud2） |
| `lslidar_msgs` | 自定义雷神激光雷达消息（包含在工作空间中） |
| `pcl_conversions` / `libpcl-all-dev` | PCL 点云转换 |
| `libpcap` | 数据包捕获（PCAP）回放支持 |
| `pluginlib` | 插件库加载 |
| `diagnostic_updater` | 诊断状态发布 |
| `Boost`（特别是 `thread`） | 线程支持 |

## 编译

```bash
cd /path/to/ros2_lidar_ws
colcon build
source install/setup.bash
```

工作空间根目录也提供了便捷编译脚本 `build.sh`：

```bash
./build.sh
```

## 使用说明

### 单雷达启动

启动单个激光雷达驱动并打开 RViz 可视化：

```bash
ros2 launch lslidar_driver lslidar_launch.py
```

### 双雷达启动

适用于两台雷达的场景（分别通过 `lsx10_1.yaml` 和 `lsx10_2.yaml` 配置）：

```bash
ros2 launch lslidar_driver lslidar_double_launch.py
```

### 控制雷达启停

- **开启雷达：**
  ```bash
  ros2 topic pub -1 /lslidar_order std_msgs/msg/Int8 "data: 1"
  ```
- **关闭雷达：**
  ```bash
  ros2 topic pub -1 /lslidar_order std_msgs/msg/Int8 "data: 0"
  ```

### 发布的话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/scan` | `sensor_msgs/LaserScan` | 激光扫描数据（通过 `pubScan` 参数启用） |
| `/lslidar_point_cloud` | `sensor_msgs/PointCloud2` | 完整点云（通过 `pubPointCloud2` 参数启用） |
| `/lslidar_order` | `std_msgs/Int8` | 雷达启停命令 |

## 配置说明

主配置文件：`lslidar_driver/params/lsx10.yaml`

关键参数：

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `frame_id` | `laser_link` | 激光雷达坐标系 |
| `device_ip` | `192.168.1.200` | 雷达源 IP 地址 |
| `msop_port` | `2368` | MSOP 数据目标端口 |
| `difop_port` | `2369` | DIFOP 源端口 |
| `lidar_name` | `N10_P` | 雷达型号选择 |
| `interface_selection` | `serial` | 通信接口：`net`（网口）或 `serial`（串口） |
| `serial_port_` | `/dev/ttyACM0` | 串口设备路径 |
| `min_range` / `max_range` | `0.0` / `200.0` | 距离过滤范围（米） |
| `angle_disable_min` / `angle_disable_max` | `0.0` / `0.0` | 角度裁剪范围（度） |
| `use_gps_ts` | `false` | 启用 GPS 时间戳 |
| `high_reflection` | `false` | M10_P 高反射模式 |
| `compensation` | `false` | M10 系列角度补偿 |
| `pubScan` | `true` | 是否发布 LaserScan 话题 |
| `pubPointCloud2` | `false` | 是否发布 PointCloud2 话题 |
| `pcap` | （已注释） | PCAP 文件回放路径 |
| `in_file_name` | （已注释） | TXT 文件输入路径 |

## 自定义消息（`lslidar_msgs`）

| 消息 | 字段 | 描述 |
|------|------|------|
| `LslidarPacket.msg` | `stamp`, `data[2000]` | 原始雷达数据包 |
| `LslidarScan.msg` | `altitude`, `LslidarPoint[]` | 单帧扫描，点按方位角排序 |
| `LslidarPoint.msg` | `time`, `x`, `y`, `z`, `azimuth`, `distance`, `intensity` | 单点测量数据 |
| `LslidarSweep.msg` | `header`, `LslidarScan[16]` | 完整一圈扫描（16 帧） |
| `LslidarDifop.msg` | `temperature`, `rpm` | DIFOP 诊断数据 |

## 通信接口

### 网口（UDP）
- 默认 MSOP 端口：`2368`
- 默认 DIFOP 端口：`2369`
- 支持组播（通过 `add_multicast` 和 `group_ip` 参数配置）

### 串口
- 默认设备：`/dev/ttyACM0`
- 支持的波特率：230400、460800、500000、921600
- 可配置校验位、停止位和数据位

### PCAP 回放
- 支持从 `.pcap` 捕获文件离线回放
- 可配置回放速率和重复延迟

## 版本历史

| 版本 | 日期 | 主要变更 |
|------|------|----------|
| V2.5.2 | 2023-01-10 | 修复串口包头检测；M10 角度补偿；可配置 pointcloud2/scan 发布；支持 ROS 2 Humble；双回波雷达；TXT 文件输入 |
| V2.5.1 | 2022-11-30 | 新增 L10 支持；修复 M10_P 点云问题；修复内存泄漏；更新协议 |
| V2.5.0 | 2022-11-04 | 初始版本：支持 M10/M10_P/M10_PLUS/N10/M10_GPS；角度裁剪；距离过滤；雷达启停；PCAP 支持 |

## 许可证

本项目基于 **GNU General Public License v3.0** 开源协议。

详见 `src/Lslidar_ROS2_driver-M10P-N10P/` 下的源代码文件中的许可信息。

