# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 (Humble) autonomous food delivery robot. The workspace is a standard colcon workspace at `~/Food_Delivery_Robot/`. Hardware target is NVIDIA Jetson (Orin Nano), with PC simulation support via Gazebo.

## Build Commands

```bash
# Build all packages
cd ~/Food_Delivery_Robot
colcon build

# Build a single package
colcon build --packages-select <package_name>

# Source after build
source install/setup.bash
```

Always `source install/setup.bash` after building before running any nodes.

## Running the Robot

**Full autonomous (real robot):**
```bash
ros2 launch rplidar_ros rplidar_a1_launch.py          # Terminal 1: Lidar
ros2 launch food_del_robot_description robot.launch.py # Terminal 2: Full stack
```

**Simulation:**
```bash
ros2 launch food_del_robot_description robot_sim_with_amcl.launch.py
```

**Individual sensors:**
```bash
ros2 run yahboom_imu_10_axis imu_driver
ros2 run zlac8015d_driver drive_and_odom_v2
```

**Display URDF only:**
```bash
ros2 launch food_del_robot_description display.launch.py
```

**SLAM (online mapping):**
```bash
ros2 launch food_del_robot_description slam_online_mapping.launch.py
```

## Architecture

The navigation stack is custom — it does **not** use the standard Nav2 controller/planner servers. Instead, two standalone C++ nodes replace them:

### Custom Nodes (src/)

| Package | Executable | Role |
|---|---|---|
| `food_del_astar_planner` | `astar_planner_node` | Global path planning. Owns its own `Costmap2DROS` (global costmap) internally. Subscribes `/amcl_pose` + `/goal_pose`, publishes `/plan`. |
| `dwb_controller` | `main` | Local controller. Owns its own `Costmap2DROS` (local costmap) internally. Subscribes `/plan` + `/odom`, publishes `/cmd_vel` at 50 Hz. |
| `zlac8015d_driver` | `drive_and_odom_v2` | ZLAC8015D dual-motor driver over CAN. Publishes `/odom/wheel` and `/cmd_vel` subscriber. |
| `food_del_robot_hardware` | (ros2_control plugin) | `ros2_control` hardware interface for the wheel motors. |
| `yahboom_imu_10_axis` | `imu_driver` | Serial IMU driver, publishes `/imu/data` at 100 Hz. |
| `robot_kinematic` | — | Differential drive kinematics library. |

**Localization:** AMCL (`nav2_amcl`) using a pre-built map. EKF (`robot_localization`) fuses `/odom/wheel` + `/imu/data` into `/odometry/filtered`.

**Sensors:** RPLidar A1 (`/scan`), Yahboom 10-axis IMU (`/imu/data`), Intel RealSense D435 depth camera (point cloud used for STVL layer).

### Costmap Architecture

Both `astar_planner_node` and `dwb_controller` own their costmap instances internally (not via separate Nav2 lifecycle nodes). Parameters are passed directly via `parameters=[config_yaml]` in the launch file.

- Global costmap config: `src/food_del_robot/config/Real_Robot/Planner_Server/A_star_config.yaml`
  - Layers: `static_layer`, `obstacle_layer` (LiDAR), `stvl_layer` (depth camera), `inflation_layer`
- Local costmap config: `src/food_del_robot/config/Real_Robot/Controller_Server/dwb_config.yaml`

### DWB Controller Implementation

`dwb_controller` is a pure C++ custom DWB implementation (not the Nav2 DWB plugin). Key parameters in `rollout_best_control()` call in `main.cpp`:
- v_max: 0.3 m/s, w_max: 0.5 rad/s
- Robot radius: 0.22 m, clearance threshold: 0.5 m
- Obstacle detection radius from costmap: 2.5 m

### Config Layout

```
src/food_del_robot/config/
├── Real_Robot/
│   ├── AMCL/             amcl_config.yaml
│   ├── BT_Navigator/     bt_navigator_config.yaml
│   ├── Controller_Server/ dwb_config.yaml
│   ├── EKF/              ekf_config.yaml
│   ├── Planner_Server/   A_star_config.yaml
│   ├── SLAM/             slam_online_mapping.yaml
│   └── Lifecycle_manager/
└── Simulation/
    ├── Costmap/          global_costmap_config.yaml, local_costmap_config.yaml
    └── ...
```

### Maps

Pre-built maps in `src/food_del_robot/maps/`:
- `Class_map/class_map.yaml` — primary map used by `robot.launch.py`
- `Outdoor_map/outdoor_map.yaml`
- `Simulation_map/Simulation_map.yaml`

### Hardware Notes

- Motor driver communicates via CAN bus (`can.cpp` in both `food_del_robot_hardware` and `zlac8015d_driver`)
- Lidar port: `/dev/lidar` (udev symlink), IMU port: `/dev/imu`
- Jetson Orin Nano requires manual CH341 serial driver installation for IMU (see README.md §0.5)
- EKF config path in README references `/home/panha/...` — update to your actual `$HOME` path

## Key Topics

| Topic | Type | Source |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | rplidar_ros |
| `/imu/data` | `sensor_msgs/Imu` | yahboom_imu_10_axis |
| `/odom/wheel` | `nav_msgs/Odometry` | zlac8015d_driver |
| `/odometry/filtered` | `nav_msgs/Odometry` | robot_localization EKF |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | nav2_amcl |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz 2D Goal Pose |
| `/plan` | `nav_msgs/Path` | food_del_astar_planner |
| `/cmd_vel` | `geometry_msgs/Twist` | dwb_controller |
