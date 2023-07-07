# ORB_SLAM2_ROS2
Simple ROS2 wrapper for ORB_SLAM2, reference to [orb_slam_2_ros](https://github.com/appliedAI-Initiative/orb_slam_2_ros)

## Installation
### 1. Download adn build code
This is a ros2 package, you should put it in your ROS2 workspace.
```bash
git clone https://github.com/baiyeweiguang/ORB_SLAM2_ROS2.git
```

### 2. Build
This repository contains the source code of orb_slam2, you can build the orb_slam2 and ros2_wrapper with
```bash
colcon build
```

## Run
Adjust the config/config.yaml, use **ros2 launch** to bringup the system
```bash
souce install/setup.bash
ros2 launch orb_slam2_ros2 orb_slam2_mono.launch.py
```
