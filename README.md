# ROS 2 室内自主移动机器人

这是一个可直接运行的最小自主移动机器人闭环，适合继续接入真实底盘、传感器或 Nav2。当前实现不依赖 Gazebo：底盘节点用差速运动学积分速度命令并发布里程计，因此启动后即可验证目标控制和巡航任务。

## 功能

- 底盘安全层：线速度/角速度限幅、命令超时停车、里程计和 `odom -> base_footprint` TF
- 点到点控制：订阅目标与里程计，进行航向校正并发布速度命令
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

启动全部节点（默认保持静止）：

```bash
ros2 launch robot_bringup simulation.launch.py
```

向控制器直接发送一个 `odom` 坐标系目标：

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

## 主要接口

| 名称 | 类型 | 方向 | 用途 |
| --- | --- | --- | --- |
| `/task_command` | `std_msgs/msg/String` | 输入 | `patrol`、`home` 或 `stop` |
| `/task_status` | `std_msgs/msg/String` | 输出 | 当前任务状态 |
| `/navigate_to_pose` | `robot_interfaces/action/NavigateToPose` | 输入/输出 | 带取消、结果和反馈的导航目标 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 内部 | 控制器速度命令 |
| `/cmd_vel_safe` | `geometry_msgs/msg/Twist` | 输出 | 限幅和超时处理后的命令 |
| `/odom` | `nav_msgs/msg/Odometry` | 输出 | 仿真里程计 |

参数集中在 `src/robot_bringup/config/robot.yaml`。修改巡航点时，`waypoint_x` 和 `waypoint_y` 必须长度相同且非空。

## 包结构

- `robot_base_driver`：安全速度处理、运动学仿真、里程计与 TF
- `robot_navigation`：里程计反馈点到点控制器
- `robot_tasks`：巡航/返航任务状态机
- `robot_description`：URDF 机器人模型
- `robot_bringup`：统一 launch 与参数
- `docker`：ROS 2 Humble 构建环境

当前控制器不包含避障和全局路径规划。接入真实场景时，应将该节点替换为 Nav2，并将底盘仿真积分替换为硬件编码器反馈。
