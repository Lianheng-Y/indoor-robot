# ROS 2 室内自主移动机器人系统

这是一个面向仓库配送/办公室巡检的 ROS 2 工程化项目，当前已具备基础闭环：底盘速度安全限制、超时急停、里程计与 `odom -> base_link` TF 发布、规划/任务节点扩展点，以及 Docker/CI 配置。

## 构建与运行

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_bringup simulation.launch.py
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" -1
ros2 topic echo /odom
```

## 目录

- `robot_base_driver`: C++ 差速底盘模型，限速、命令超时急停、里程计、TF
- `robot_navigation`: 规划器节点扩展点（可接 A*/Nav2）
- `robot_tasks`: 配送/巡检任务状态机扩展点
- `robot_description`: 基础 URDF
- `robot_bringup`: launch 与参数
- `docker`: ROS 2 Humble 容器构建
- `.github/workflows`: 自动构建 CI

下一阶段可接入 Gazebo、LiDAR/IMU、SLAM Toolbox、Nav2、A*、Pure Pursuit 和 launch tests。
