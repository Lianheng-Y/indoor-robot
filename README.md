# ROS 2 室内自主移动机器人系统

面向仓库配送/办公室巡检的可扩展 ROS 2 软件骨架。当前提供底盘驱动、状态发布、导航接口、任务调度、仿真启动和自动化测试的基础结构。

## 快速开始

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_bringup simulation.launch.py
```

## 包结构

- `robot_base_driver`: C++ 差速底盘驱动与安全限速接口
- `robot_navigation`: 规划/控制算法扩展点
- `robot_tasks`: 任务状态机扩展点
- `robot_description`: 机器人 URDF
- `robot_bringup`: 参数、launch 和系统集成

后续阶段接入 Nav2、SLAM Toolbox、rosbag 回归、Docker 与真实硬件。
