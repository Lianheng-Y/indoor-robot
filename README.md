# ROS 2 室内自主移动机器人

项目提供轻量点控制、Nav2 传感器导航、Gazebo Sim 物理仿真和 `ros2_control` 真实底盘四套运行模式。仿真模式使用差速驱动、碰撞/摩擦、轮关节、2D LiDAR、IMU 和 Gazebo 时钟。

## 功能

- 底盘安全层：线速度/角速度限幅、命令超时停车、里程计和 `odom -> base_footprint` TF
- 轻量点控制：`point_controller_node` 根据目标与里程计进行航向校正
- 控制器细节：位置/朝向两阶段控制、加减速限幅、输出平滑、里程计跳变抑制和可选倒车
- Nav2 导航：地图服务器、AMCL、NavFn、Regulated Pure Pursuit、双 costmap 和恢复行为
- 任务状态机：支持多点巡航、返航、停止及状态发布
- 机器人模型：底盘、左右轮和 LiDAR，可由 `robot_state_publisher` 发布完整 TF
- 参数化 Xacro：车体、轮径、轮距、传感器和 Gazebo 插件拆分在独立文件中
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

任务管理器的正式接口是 `robot_interfaces/action/NavigateWaypoints`，每个航点包含完整 `PoseStamped`，并可配套停留秒数：

```bash
ros2 action send_goal /navigate_waypoints robot_interfaces/action/NavigateWaypoints \
  "{waypoints: [{header: {frame_id: odom}, pose: {position: {x: 1.0}, orientation: {w: 1.0}}}, {header: {frame_id: odom}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}], dwell_times: [2.0, 0.0], loop: false}" \
  --feedback
```

任务支持 `patrol`、`pause`、`resume`、`home`、`stop` 命令。`waypoint_timeout`、`max_retries` 和 `skip_on_failure` 控制单点超时后的重试/跳过策略；`battery_state` 的电量低于 `battery_threshold` 时自动返航。启用 `resume_on_start` 后，任务索引会从 `state_file` 恢复。

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

### Gazebo Sim 物理仿真

安装 `ros_gz_sim`、`ros_gz_bridge` 和 Gazebo Sim 后启动：

```bash
ros2 launch robot_bringup gazebo.launch.py
```

该入口会加载 `robot.xacro` 和 `indoor_world.sdf`，并启动：

- `gz-sim-diff-drive-system`：差速驱动和物理里程计
- 轮关节状态发布器及 `/tf`、`/odom` 桥接
- 720 线等效 2D GPU LiDAR：`/scan`
- 100 Hz IMU：`/imu`
- `/clock` 仿真时间
- Nav2 的 AMCL、costmap、规划、控制和恢复行为

不要在 Gazebo 模式同时启动 `base_driver_node`，否则会产生重复的 `/odom` 和 `cmd_vel` 消费者。

### 真实底盘模式

真实底盘使用 `ros2_control`，不启动会对速度指令做数学积分的 `base_driver_node`：

```text
/cmd_vel -> velocity_safety_node -> /cmd_vel_safe
         -> diff_drive_controller -> hardware_interface -> serial/CAN motors
wheel encoders -> hardware_interface -> diff_drive_controller -> /odom + odom TF
```

启动控制器框架：

```bash
ros2 launch robot_bringup real_robot.launch.py \
  hardware_plugin:=your_hardware_package/SerialCanHardware \
  transport:=serial serial_port:=/dev/ttyUSB0 baud_rate:=115200 \
  encoder_cpr:=4096 gear_ratio:=30.0
```

`hardware_plugin` 必须实现 `hardware_interface::SystemInterface`，导出左右轮的 velocity command，以及 position/velocity state。launch 会将 `transport`、`serial_port`、`baud_rate`、`can_interface`、`encoder_cpr` 和 `gear_ratio` 传给插件；电机 ID、帧格式和错误码仍属于具体硬件协议，应由该插件实现，不能由通用导航节点猜测。

不传 `hardware_plugin` 时使用 `mock_components/GenericSystem`，仅用于验证 controller manager、话题和控制器配置，不能提供真实编码器里程计。

`config/nav2_params.yaml` 包含：

- 全局 costmap：静态地图层、激光障碍层和膨胀层
- 局部 costmap：滚动窗口、激光障碍标记/清除和膨胀层
- NavFn A* 全局规划器和 Regulated Pure Pursuit 控制器
- BT Navigator、旋转/后退/直行/等待恢复行为
- 进度检查、目标检查、速度平滑和生命周期管理

轻量点控制器在到达位置容差后会停止平移并原地旋转，直到目标四元数对应的 yaw 进入 `yaw_tolerance`。以下参数可调整控制动态：

```yaml
yaw_tolerance: 0.05
max_linear_accel: 0.8
max_linear_decel: 1.2
max_angular_accel: 2.0
max_angular_decel: 3.0
command_smoothing_alpha: 0.35
max_odom_jump: 1.0
odom_filter_alpha: 0.35
allow_reverse: false
```

`allow_reverse` 开启后，目标位于车后方时控制器会选择倒车；默认关闭以保持前进策略。异常里程计跳变会被拒绝，剩余数据使用一阶滤波后再参与控制。

## 主要接口

| 名称 | 类型 | 方向 | 用途 |
| --- | --- | --- | --- |
| `/task_command` | `std_msgs/msg/String` | 输入 | `patrol`、`home` 或 `stop` |
| `/task_status` | `std_msgs/msg/String` | 输出 | 当前任务状态 |
| `/navigate_waypoints` | `robot_interfaces/action/NavigateWaypoints` | 输入/输出 | 多航点任务、取消、结果和进度反馈 |
| `/navigate_to_pose` | `robot_interfaces/action/NavigateToPose` 或 `nav2_msgs/action/NavigateToPose` | 输入/输出 | 取决于启动的导航模式 |
| `/scan` | `sensor_msgs/msg/LaserScan` | Nav2 输入 | 定位和动态障碍物观测 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 内部 | 控制器速度命令 |
| `/cmd_vel_safe` | `geometry_msgs/msg/Twist` | 输出 | 限幅和超时处理后的命令 |
| `/odom` | `nav_msgs/msg/Odometry` | 输出 | 模拟、Gazebo 或编码器里程计 |

参数集中在 `src/robot_bringup/config/robot.yaml`。修改巡航点时，`waypoint_x` 和 `waypoint_y` 必须长度相同且非空。

## 包结构

- `robot_base_driver`：独立速度安全层和仅供开发使用的运动学模拟器
- `robot_navigation`：独立的里程计反馈点控制器，不承担路径规划
- `robot_tasks`：巡航/返航任务状态机
- `robot_tasks`：强类型多航点 Action、暂停/恢复、重试/跳过、持久化和低电量返航
- `robot_description`：参数化 Xacro、传感器模型、Gazebo 插件和仿真世界
- `robot_bringup`：轻量模式与 Nav2 模式 launch、参数和示例地图
- `docker`：ROS 2 Humble 构建环境

`config/ros2_control.yaml` 定义差速控制器、闭环编码器里程计和 joint state broadcaster。Xacro 文件位于 `src/robot_description/urdf/`：`robot.xacro` 负责装配及 `ros2_control` 接口，`materials.xacro` 管理材质，`sensors.xacro` 管理 LiDAR/IMU，`gazebo.xacro` 管理物理插件。部署时还应根据实际外形、雷达量程和运动学约束调整 `nav2_params.yaml`。
