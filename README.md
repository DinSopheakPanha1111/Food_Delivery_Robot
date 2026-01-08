# Food_Delivery_Robot
BY : DIN SOPHEAK PANHA

**0. SETUP LIBRARY AND CONFIGUARTIONS**

Please do all of these before start playing around with the robot (JETSON only)
If it was testing on PC, I recommend only do 0.1, 0.2, 0.3 only

**0.1 Setup ROS2 Controllers (YOU CAN SKIP IF YOU ALREAD HAVE IT)**

```

sudo apt update
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers

```

**0.2 Setup Navigation2 (YOU CAN SKIP IF YOU ALREAD HAVE IT)**

```

sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup

```

**0.3 Setup RPLidar ROS2 Humble**

```
sudo chmod 777 /dev/ttyUSB0
cd src/rpldiar_ros-ros2/
source scripts/create_udev_rules.sh

```

**0.4 Yahboom 10 axis IMU**

For Jetson Orin Nano, it doesn't support the CH341 serial driver, so you have to install and setup manually.

Please follow the video shown in the forum from start till end, DO NOT SKIP!!!! : 

https://forums.developer.nvidia.com/t/ch340-serial-devices-not-enumerating-in-dev-tty/342092/3

Then, run 

```
sudo usermod -aG dialout $USER

```

Then, reboot 

```
sudo reboot

```
After the reboot successfully, you can check 

```
ls -l /dev/ttyUSB0

```

Expected result : 

```
crw-rw---- 1 root dialout 188, 0 Jan  8 05:26 /dev/ttyUSB0

```

**0.5 Build the workspace**

```

cd Food_Delivery_Robot
source install/setup.bash
colcon build
source install/setup.bash

```

YOU ARE READY TO GO!!!!

**1. LAUNCH SENSOR**

**1.1 Launch RPLidar A1**

```
ros2 run rplidar_ros rplidar_a1

```

topic list : 

```

/scan

```

Frequency and BW : 

```

Frequency : 

average rate: 6.659
	min: 0.145s max: 0.153s std dev: 0.00172s window: 15

BW : 

20.14 KB/s from 20 messages
	Message size mean: 2.94 KB min: 2.94 KB max: 2.94 KB

```

**1.2 LAUNCH IMU**

```
ros2 run yahboom_imu_10_axis imu_driver

```

topic list : 

```

/imu/data

```

Frequency and BW : 

```

Frequency : 

average rate: 100.012
	min: 0.010s max: 0.010s std dev: 0.00007s window: 101

BW : 

32.57 KB/s from 100 messages
	Message size mean: 0.32 KB min: 0.32 KB max: 0.32 KB

```

**1.4 LAUNCH ROBOT IN SIMULATION**

```
ros2 launch food_del_robot_description robot.launch.py 

```

**2. MOTOR DRIVE TEST**

**2.1 RUN THE NODE**

```
ros2 run zlac8015d_driver drive_and_odom 

```

topic list : 

```
/cmd_vel
/odom/wheel
/parameter_events
/rosout
```
**2.2 PUBLISH SPEED TEST**

```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"

```

**2.3 ODOMETRY CHECK TEST**

```
ros2 topic echo /odom/wheel

```

**2.4 EMERGENCY STOP/RELEASE TEST**

```
ros2 service call /emergency_stop food_del_robot/srv/EmergencyStop "{stop: true, release: false}"

```
