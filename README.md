# Robotic Systems Design Project Team2 (UoM AERO62520)

## Introduction

Welcome to Team2's Git repository!
We're MSc Robotics postgraduate students at the University of Manchester. This is the code repository for our Robotic Systems Design Project (AERO62520), and the progress of our work is updated here in real time. The goal of this project is to use existing materials to implement a robot that explores autonomously and can recognise a target object and bring it back.

| Kit                              | Description                                                  |
| -------------------------------- | ------------------------------------------------------------ |
| LEO Core                         | Serving as the robot's brain, the LEO Core processes sensor data and controls movements, ensuring smooth and coordinated operation of all tasks. |
| Intel NUC                        | Used for handling intensive computation, the Intel NUC runs complex algorithms for object recognition and decision-making, significantly boosting the robot's autonomous capabilities. |
| Battery                          | A rechargeable battery pack powers the entire system, enabling the robot to operate independently without relying on an external power source. |
| Powerbox                         | Responsible for distributing power from the battery to all components, the Powerbox ensures stable voltage and prevents overloads, supporting safe and reliable operation. |
| RPLIDAR A2M12                    | This 2D laser scanner measures distances to map the environment and detect obstacles, playing a key role in navigation and collision avoidance. |
| Intel RealSense Depth Camera     | By providing 3D depth information, this camera helps the robot understand its surroundings, enhancing obstacle detection and navigation precision. |
| Trossen PincherX 150 Manipulator | Equipped with a robotic arm, the robot can physically interact with objects, including grasping and moving them, enabling object manipulation tasks. |
| Raspberry Pi 4 Model B           | Handling less intensive tasks like image processing and communication, the Raspberry Pi supports object recognition and task planning while ensuring smooth sensor operation. |
| Camera                           | Capturing visual data, the camera plays a critical role in recognizing objects, analyzing the environment, and aiding navigation. |
| 2.4GHz WiFi Adapter              | Enabling wireless communication, the Wi-Fi adapter allows real-time control, monitoring, and data transfer between the robot and external systems. |

## Members

- Yuliang Li https://github.com/Lyrance
- Zihan Yue https://github.com/AvidLuv
- Jiadong Hou https://github.com/hou-jd
- Zixiang He https://github.com/Hzxxxxxxx

## Getting Started

> Ubuntu 22.04 LTS; ROS2 Humble; Python 3.10.12

### Step1: Clone

```bash
mkdir -p ros2_ws/src
git clone https://github.com/Lyrance/Robotics_Team2.git ros2_ws/src
cd ros2_ws
```

### Step2: Build

Make sure you have setup the environment. Then.

```bash
colcon build
source install/setup.bash
```

### Step3: Run

We usually use `ros2 run` to run ros2 nodes and `ros2 launch` to interact with launch files. Specific names and descriptions of each node are shown below.

#### Object Detection

- Launch the Realsense camera.

```bash
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=848x480x30 rgb_camera.color_profile:=848x480x30 align_depth.enable:=true pointcloud.enable:=true
```

- Run the predictor node.

```bash
ros2 run object_dection predict
```

#### Mapping & Navigation

- Launch the SLAM.

```bash
ros2 launch team2_nav nav.launch.py
```

- Start the explorer.

```bash
ros2 run team2_nav explorer
```

#### Grasping

- Launch the controller.

```bash
ros2 launch interbotix_xsarm_control xsarm_control.launch robot_model:=px150
```

- Start to grasp.

```bash
ros2 run manipulator ServerOfPerceptionAndGrasp
```

**We do apologise that running many commands at the same time can be a bit annoying. This project is still under testing. We'll integrate it into a launch file later, after which it can be started with a single command.*

## Requirements

| NAME                             | source                                                       |
| -------------------------------- | ------------------------------------------------------------ |
| rplidar_ros                      | https://github.com/Slamtec/rplidar_ros                       |
| Intel® RealSense™ SDK 2.0        | sudo apt install ros-humble-librealsense2*                   |
| ROS servers of Intel® RealSense™ | sudo apt install ros-humble-realsense2-*                     |
| ultralytics                      | https://docs.ultralytics.com/quickstart/#install-ultralytics |
| SLAM-Toolbox                     | https://github.com/SteveMacenski/slam_toolbox                |
| nav2                             | https://docs.nav2.org                                        |
| interbotix_ros_manipulators      | https://github.com/Interbotix/interbotix_ros_manipulators    |
| imgviz                           | pip install imgviz                                           |
|                                  |                                                              |
| *To Be Continued...*             |                                                              |

