# 📱 PhoneFusion‑Nav — Full ROS 2 C++ Project Documentation

## 🧾 Overview

**Author:** Pratyush  
**Timeline:** Sept 28 → Oct 10 2025  
**Goal:** Build a **ROS 2 (C++) project** that fuses smartphone IMU + camera data and drives a simulated robot (TurtleSim → Gazebo) using your own code.  
**Purpose:** Gain real ROS 2 C++ experience that demonstrates end‑to‑end robotics fundamentals for recruiters.

---

## 🚀 Motivation

As a robotics grad student (aeronautics background) seeking a strong portfolio project, I needed one compact, working demonstration that shows:

* I can **code ROS 2 nodes in C++**, build packages, manage TFs, and use launch files.  
* I can **integrate real hardware** (a phone streaming IMU + camera).  
* I understand **planning, control, and data fusion**.  

Thus → **PhoneFusion‑Nav**, a smartphone‑powered ROS 2 sensor fusion + navigation stack.

---

## 🧠 Concept Summary

**PhoneFusion‑Nav** converts a regular Android phone into a ROS 2 sensor head and runs minimal C++ nodes that:

| Node | Purpose | Topics |
|-------|----------|--------|
| `phone_imu_bridge` | UDP IMU receiver → publishes `sensor_msgs/Imu` | `/imu/data` |
| `phone_cam_bridge` | RTSP/HTTP camera stream → `sensor_msgs/Image` | `/camera/image_raw` |
| `apriltag_pose` | AprilTag detection → camera pose estimate | `/vision/pose` |
| `ekf2d` | Fuse IMU + vision using 3‑state EKF (x, y, yaw) | `/odom` |
| `tiny_planner` | 2D straight‑line/A* planner | `/plan` |
| `pure_pursuit_controller` | Pure‑pursuit path follower | `/cmd_vel` |

Each node is modular and implemented in C++ using `rclcpp`.

---

## 🧩 System Requirements

| Component | Details |
|------------|----------|
| OS | Ubuntu 22.04 LTS |
| ROS Distro | ROS 2 Humble Hawksbill |
| Compiler | g++ 11+, C++17 |
| Libraries | `libeigen3-dev`, `libapriltag-dev`, OpenCV |

Install essentials:

```bash
sudo apt update
sudo apt install -y ros-humble-desktop-full libeigen3-dev libapriltag-dev tree git
```

---

## 🏗️ Workspace Setup

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

* `mkdir -p` — creates directories recursively.  
* `ros2_ws` — ROS 2 workspace root containing:  

```
ros2_ws/
├── src/        # your packages
├── build/      # temporary build files
├── install/    # compiled executables
└── log/        # logs
```

---

## 📁 Folder Scaffold

```
phonefusion_nav/
├── README.md
├── LICENSE
├── colcon.pkg
├── config/
│   ├── camera.yaml
│   ├── controller.yaml
│   ├── planner.yaml
│   └── tags.yaml
├── launch/
│   ├── demo_turtlesim.launch.py
│   └── record.launch.py
├── src/
│   ├── phone_imu_bridge/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   └── src/main.cpp
│   ├── phone_cam_bridge/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   └── src/main.cpp
│   ├── apriltag_pose/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   └── src/main.cpp
│   ├── ekf2d/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   └── src/main.cpp
│   ├── tiny_planner/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   └── src/main.cpp
│   └── pure_pursuit_controller/
│       ├── package.xml
│       ├── CMakeLists.txt
│       └── src/main.cpp
└── test/
    ├── ekf2d_test.cpp
    └── planner_test.cpp
```

Create with:

```bash
mkdir -p phonefusion_nav/{config,launch,test,src,.github/workflows}
for p in phone_imu_bridge phone_cam_bridge apriltag_pose ekf2d tiny_planner pure_pursuit_controller; do
  mkdir -p phonefusion_nav/src/$p/src
  touch phonefusion_nav/src/$p/{CMakeLists.txt,package.xml}
  touch phonefusion_nav/src/$p/src/main.cpp
done
touch phonefusion_nav/config/{camera.yaml,controller.yaml,planner.yaml,tags.yaml}
touch phonefusion_nav/launch/{demo_turtlesim.launch.py,record.launch.py}
touch phonefusion_nav/test/{ekf2d_test.cpp,planner_test.cpp}
```

---

## 🧰 Stub Node Example

**main.cpp**

```cpp
#include <rclcpp/rclcpp.hpp>
int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("phone_imu_bridge");
  RCLCPP_INFO(node->get_logger(),"phone_imu_bridge stub running.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

**CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.16)
project(phone_imu_bridge)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
add_executable(imu_bridge src/main.cpp)
ament_target_dependencies(imu_bridge rclcpp)
install(TARGETS imu_bridge DESTINATION lib/${PROJECT_NAME})
ament_package()
```

Each other package is identical except for the names.

---

## 🧭 Launch Files

**demo_turtlesim.launch.py**
```python
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
  return LaunchDescription([
    Node(package='turtlesim',executable='turtlesim_node',name='turtlesim'),
    Node(package='phone_imu_bridge',executable='imu_bridge',name='imu_bridge'),
    Node(package='phone_cam_bridge',executable='cam_bridge',name='cam_bridge'),
    Node(package='apriltag_pose',executable='apriltag_node',name='apriltag_node'),
    Node(package='ekf2d',executable='ekf2d_node',name='ekf2d'),
    Node(package='tiny_planner',executable='tiny_planner_node',name='tiny_planner'),
    Node(package='pure_pursuit_controller',executable='pure_pursuit_node',name='pure_pursuit')
  ])
```

**record.launch.py**
```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
def generate_launch_description():
  return LaunchDescription([
    ExecuteProcess(cmd=['ros2','bag','record','-o','bags/mvp',
      '/imu/data','/camera/image_raw','/vision/pose','/odom','/plan','/cmd_vel'])
  ])
```

---

## 🧱 Building

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Check installed executables:

```bash
ros2 pkg executables phone_imu_bridge
```

Run demo:

```bash
ros2 launch phonefusion_nav demo_turtlesim.launch.py
```

Expected log:
```
[INFO] [phone_imu_bridge]: stub running
[INFO] [phone_cam_bridge]: stub running
...
```

---

## 🧠 Explanation of Key Commands

| Command | Purpose |
|----------|----------|
| `mkdir -p` | Make nested folders safely |
| `touch file` | Create empty file |
| `tree -L 3` | Visualize folder |
| `colcon build` | Build all ROS 2 packages |
| `source install/setup.bash` | Activate workspace |
| `ros2 launch` | Run multiple nodes |

---

## 🧮 Next Steps (Phase B)

1. Replace stubs with working implementations:
   * UDP IMU receiver (JSON)
   * RTSP camera (OpenCV)
   * AprilTag detection → pose calc
   * EKF 2D fusion
   * Planner + controller
2. Visualize in RViz.  
3. Record rosbag, upload to GitHub with demo video.

---

## 🎯 Key Takeaways

* You now have a **fully compiling 6‑node ROS 2 C++ project** scaffold.  
* You understand `mkdir -p`, `touch`, and the ROS 2 workspace layout.  
* You can build/run/launch nodes end‑to‑end.

---

**License:** MIT  
**Maintainer:** Pratyush  
**Language:** C++17  
**ROS 2 Distro:** Humble Hawksbill  
**Date:** October 10 2025
