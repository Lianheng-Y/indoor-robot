# ROS 2 室内自主移动机器人

项目提供两套导航模式：无需 Gazebo 的轻量点到点控制闭环，以及基于 Nav2 的地图定位、全局规划、局部避障和行为树导航。底盘节点使用差速运动学积分速度命令并发布里程计，也可以替换为真实硬件驱动。

## 功能

- 底盘安全层：线速度/角速度限幅、命令超时停车、里程计和 `odom -> base_footprint` TF
- 轻量点控制：`point_controller_node` 根据目标与里程计进行航向校正
- Nav2 导航：地图服务器、AMCL、NavFn、Regulated Pure Pursuit、双 costmap 和恢复行为
- 任务状态机：支持多点巡航、返航、停止及状态发布
- 机器人模型：底盘、左右轮和 LiDAR，可由 `robot_state_publisher` 发布完整 TF
- 工程配置：集中式 YAML 参数、可选自动巡航、Docker 和 GitHub Actions

## 环境与构建

基线环境为 ROS 2 Humble / Ubuntu 22.04，新版 ROS 2 也可构建。

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 运行

### 轻量点控制模式

启动运动学底盘、点控制器和巡航任务管理器（默认保持静止）：

```bash
ros2 launch robot_bringup simulation.launch.py
```

向点控制器发送一个 `odom` 坐标系目标：

```bash
ros2 action send_goal /navigate_to_pose robot_interfaces/action/NavigateToPose \
  "{target_pose: {header: {frame_id: odom}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}}"
```

启动预设三点巡航：

```bash
ros2 topic pub --once /task_command std_msgs/msg/String "{data: patrol}"
```

也可以启动时自动巡航：

```bash
ros2 launch robot_bringup simulation.launch.py auto_start:=true
```

任务控制命令为 `patrol`、`home`、`stop`。查看运行状态与位置：

```bash
ros2 topic echo /task_status
ros2 topic echo /odom
ros2 run tf2_ros tf2_echo map base_link
```

### Nav2 模式

Nav2 模式需要一个发布 `sensor_msgs/msg/LaserScan` 的 `/scan` 数据源，例如真实雷达或 Gazebo 插件。现有运动学底盘只发布 `/odom`，不会伪造障碍物扫描。

启动完整导航栈：

```bash
ros2 launch robot_bringup navigation.launch.py
```

默认加载 `maps/demo_map.yaml`。使用自己的地图：

```bash
ros2 launch robot_bringup navigation.launch.py map:=/absolute/path/to/map.yaml
```

启动后可通过 RViz 的 `2D Pose Estimate` 修正初始位姿，并用 `Nav2 Goal` 下发目标；也可以直接调用 Nav2 标准 Action：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

Nav2 模式由 AMCL 发布 `map -> odom`。不要同时启动 `simulation.launch.py`，否则轻量模式的静态 `map -> odom` 会与 AMCL 冲突。

`config/nav2_params.yaml` 包含：

- 全局 costmap：静态地图层、激光障碍层和膨胀层
- 局部 costmap：滚动窗口、激光障碍标记/清除和膨胀层
- NavFn A* 全局规划器和 Regulated Pure Pursuit 控制器
- BT Navigator、旋转/后退/直行/等待恢复行为
- 进度检查、目标检查、速度平滑和生命周期管理

## 主要接口

| 名称 | 类型 | 方向 | 用途 |
| --- | --- | --- | --- |
| `/task_command` | `std_msgs/msg/String` | 输入 | `patrol`、`home` 或 `stop` |
| `/task_status` | `std_msgs/msg/String` | 输出 | 当前任务状态 |
| `/navigate_to_pose` | `robot_interfaces/action/NavigateToPose` 或 `nav2_msgs/action/NavigateToPose` | 输入/输出 | 取决于启动的导航模式 |
| `/scan` | `sensor_msgs/msg/LaserScan` | Nav2 输入 | 定位和动态障碍物观测 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 内部 | 控制器速度命令 |
| `/cmd_vel_safe` | `geometry_msgs/msg/Twist` | 输出 | 限幅和超时处理后的命令 |
| `/odom` | `nav_msgs/msg/Odometry` | 输出 | 仿真里程计 |

参数集中在 `src/robot_bringup/config/robot.yaml`。修改巡航点时，`waypoint_x` 和 `waypoint_y` 必须长度相同且非空。

## 包结构

- `robot_base_driver`：安全速度处理、运动学仿真、里程计与 TF
- `robot_navigation`：独立的里程计反馈点控制器，不承担路径规划
- `robot_tasks`：巡航/返航任务状态机
- `robot_description`：URDF 机器人模型
- `robot_bringup`：轻量模式与 Nav2 模式 launch、参数和示例地图
- `docker`：ROS 2 Humble 构建环境

部署到真实机器人时，应使用编码器/IMU 融合里程计替换运动学积分，并根据实际外形、雷达量程和运动学约束调整 `nav2_params.yaml`。
