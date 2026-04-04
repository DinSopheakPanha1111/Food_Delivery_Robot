# Food Delivery Robot

BY: DIN SOPHEAK PANHA & TE TIKEA

---

## Package Overview

| Package | Role |
|---|---|
| `food_del_robot_bringup` | All launch files, configs, and maps |
| `food_del_robot_description` | URDF, meshes, RViz configs, Gazebo worlds |
| `food_del_robot_interfaces` | Custom service definitions (EmergencyStop, Reset) |
| `food_del_robot_hardware` | ros2_control hardware interface |
| `food_del_astar_planner` | Custom A* global planner |
| `dwb_controller` | Custom DWB local controller |
| `food_del_goal_bridge` | Goal bridge node |
| `food_del_robot_sim` | Python simulation tools |
| `zlac8015d_driver` | ZLAC8015D motor driver (CAN) |
| `yahboom_imu_10_axis` | Yahboom 10-axis IMU driver |
| `rplidar_ros-ros2` | RPLidar driver |

---

## Requirements

1. Ubuntu 22.04
2. ROS2 Humble
3. Jetson Orin Nano (optional — PC simulation supported via Gazebo)

---

## 0. Setup

Please complete all setup steps before using the robot. If testing on PC, only steps 0.1, 0.2, 0.3, 0.4 are needed.

**0.1 ROS2 Controllers**

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers
```

**0.2 Navigation2**

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup
sudo apt install ros-humble-spatio-temporal-voxel-layer
```

**0.3 RPLidar ROS2 Humble**

```bash
sudo chmod 777 /dev/ttyUSB0
cd src/rplidar_ros-ros2/
source scripts/create_udev_rules.sh
```

**0.4 Assign specific ports to Lidar and IMU**

1. Identify your devices:
```bash
lsusb
```

2. Plug in Lidar only, then find its port:
```bash
ls /dev/ttyUSB*
```

3. Get Lidar DEVPATH:
```bash
udevadm info -n /dev/ttyUSB0 | grep DEVPATH
```
Note down the unique number (e.g. `3-1.2`). Repeat for IMU.

4. Create udev rule file:
```bash
sudo nano /etc/udev/rules.d/99-robot-usb.rules
```

Paste the following (replace numbers with your DEVPATH values):
```
SUBSYSTEM=="tty", ENV{DEVPATH}=="*/usb3/3-2/3-2.2/*", SYMLINK+="lidar"
SUBSYSTEM=="tty", ENV{DEVPATH}=="*/usb3/3-2/3-2.1/*", SYMLINK+="imu"
```

CTRL+X → Y → Enter to save.

5. Reload udev rules:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout $USER
```
Then reboot.

6. Verify:
```bash
ls -l /dev/lidar
ls -l /dev/imu
```

**0.5 Yahboom 10-axis IMU (Jetson Orin Nano only)**

Jetson Orin Nano does not support the CH341 serial driver by default. Install it manually by following this forum thread from start to finish — DO NOT SKIP:

https://forums.developer.nvidia.com/t/ch340-serial-devices-not-enumerating-in-dev-tty/342092/3

Then run:
```bash
sudo usermod -aG dialout $USER
sudo reboot
```

Verify after reboot:
```bash
ls -l /dev/ttyUSB0
# Expected: crw-rw---- 1 root dialout 188, 0 ...
```

**0.6 Build the workspace**

```bash
cd ~/Food_Delivery_Robot
colcon build
source install/setup.bash
```

---

## 1. Launch Sensors

**1.1 RPLidar A1**

```bash
ros2 launch rplidar_ros rplidar_a1_launch.py
```

Topic: `/scan`

**1.2 IMU**

```bash
ros2 run yahboom_imu_10_axis imu_driver
```

Topic: `/imu/data` at 100 Hz

**1.3 RealSense D435 Depth Camera**

```bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```

Key topic: `/camera/camera/depth/color/points` (used for STVL costmap layer)

**1.4 Display Robot Model**

```bash
ros2 launch food_del_robot_bringup display.launch.py
```

---

## 2. Motor Driver Test

**2.1 Run the node**

```bash
ros2 run zlac8015d_driver drive_and_odom
```

Topics: `/cmd_vel`, `/odom/wheel`

**2.2 Speed test**

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

**2.3 Odometry check**

```bash
ros2 topic echo /odom/wheel
```

**2.4 Emergency stop test**

```bash
ros2 service call /emergency_stop food_del_robot_interfaces/srv/EmergencyStop "{stop: true, release: false}"
```

---

## 3. EKF Odometry Test

**3.1 Run motor node**
```bash
ros2 run zlac8015d_driver drive_and_odom
```

**3.2 Run IMU node**
```bash
ros2 run yahboom_imu_10_axis imu_driver
```

**3.3 Launch EKF**
```bash
ros2 launch food_del_robot_bringup ekf.launch.py
```

**3.4 Check filtered odometry**
```bash
ros2 topic echo /odometry/filtered
```

---

## 4. SLAM (Online Mapping)

```bash
ros2 launch food_del_robot_bringup slam.launch.py
```

---

## 5. Full Autonomous (Real Robot)

```bash
ros2 launch food_del_robot_bringup robot.launch.py
```

Uses: A* planner + custom DWB controller + AMCL localization on `Class_map`.

---

## 6. Simulation

**Full simulation with homemade A* planner:**
```bash
ros2 launch food_del_robot_bringup robot_sim_homemade_astar.launch.py
```

**Full simulation with SMAC planner:**
```bash
ros2 launch food_del_robot_bringup robot_sim_smac.launch.py
```

**Gazebo + RViz only:**
```bash
ros2 launch food_del_robot_bringup gazebo_in_restaurent_also_rviz.launch.py
```

**Simulation SLAM:**
```bash
ros2 launch food_del_robot_bringup slam_sim.launch.py
```

---

## 7. Manual Control (Joystick)

```bash
ros2 launch food_del_robot_bringup manual_control.launch.py
```

Starts joy + teleop + motor driver. Use this to drive the robot manually without any navigation stack.
