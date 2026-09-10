FROM ros:humble-ros-base
WORKDIR /ws
COPY . /ws
RUN . /opt/ros/humble/setup.sh && apt-get update && rosdep install --from-paths src --ignore-src -r -y && colcon build
ENTRYPOINT ["bash","-lc","source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 launch robot_bringup simulation.launch.py"]
