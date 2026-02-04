# Food_Delivery_Robot
BY : DIN SOPHEAK PANHA


**REQUIREMENT**

1. Ubuntu 22.04
2. ROS2 humble
3. Jetson (optional)

**0. SETUP LIBRARY AND CONFIGUARTIONS**

Please do all of these before start playing around with the robot (JETSON only)
If it was testing on PC, I recommend only do 0.1, 0.2, 0.3,0.4 only

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

**0.3 Assign specific port to Lidar and IMU**

1. Identify your device

```

lsusb

```

2. Assign port to Lidar and IMU

- Plug in Lidar only first

```

ls /dev/ttyUSB*

```

Expect result : /dev/ttyUSB0 or /dev/ttyUSB1

- Get Lidar DEVPATH

```

udevadm info -n /dev/ttyUSB0 | grep DEVPATH

```

Expect result : 3-1.2 or other numbers (Please note down this unique number)

- Plug out Lidar and plug in IMU only and repeat the same step for IMU

- Create udev rule file 

```

sudo nano /etc/udev/rules.d/99-robot-usb.rules

```

- Pass the script below to the file (Please changing the number according to your DEVPATH)

```

SUBSYSTEM=="tty", ENV{DEVPATH}=="*/usb3/3-2/3-2.2/*", SYMLINK+="lidar"
SUBSYSTEM=="tty", ENV{DEVPATH}=="*/usb3/3-2/3-2.1/*", SYMLINK+="imu"

```

CRTL + X -> y -> Press Enter to save

3. Reload udev rules

```

sudo udevadm control --reload-rules
sudo udevadm trigger

```

4. Dial out

```

sudo usermod -aG dialout $USER

```

Then reboot.

5. Replug both devices and verify

```

ls -l /dev/lidar
ls -l /dev/imu

```

**0.5 Yahboom 10 axis IMU**

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

**0.6 AI (optional)**

Speech to text setup:

```
pip3 install speechrecognition
sudo apt-get update
sudo apt-get install portaudio19-dev
pip3 install pyaudio

```

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

average rate: 6.932
	min: 0.135s max: 0.148s std dev: 0.00524s window: 8
average rate: 6.940
	min: 0.135s max: 0.148s std dev: 0.00526s window: 15
average rate: 6.917
	min: 0.135s max: 0.148s std dev: 0.00494s window: 22
average rate: 6.925
	min: 0.134s max: 0.148s std dev: 0.00517s window: 29

BW : 

66.06 KB/s from 7 messages
	Message size mean: 8.70 KB min: 8.70 KB max: 8.70 KB
63.37 KB/s from 14 messages
	Message size mean: 8.70 KB min: 8.70 KB max: 8.70 KB
62.52 KB/s from 21 messages
	Message size mean: 8.70 KB min: 8.70 KB max: 8.70 KB
62.11 KB/s from 28 messages
	Message size mean: 8.70 KB min: 8.70 KB max: 8.70 KB

```

Information : 

[INFO] [1767873310.778035442] [rplidar_a1_node]: RPLIDAR A1 | FW 1.29 | HW 7
[INFO] [1767873311.083058315] [rplidar_a1_node]: scan mode: Sensitivity | sample rate: 8 KHz | scan freq: 10.0 Hz | compensate x3
[INFO] [1767873311.084896206] [rplidar_a1_node]: Publishing LaserScan on: scan
  

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
**3. RAW ODOMETRY TEST**

**3.1 RUN MOTOR NODE**

```
ros2 run zlac8015d_driver drive_and_odom 

```
**3.2 RUN IMU NODE**

```
ros2 run yahboom_imu_10_axis imu_driver

```
**3.3 ODOMETRY WITH EKF TEST**

**!!NOTE : Make sure to change the params file to your own path**

```
ros2 run robot_localization ekf_node --ros-args --params-file /home/panha/Food_Delivery_Robot/src/food_del_robot/config/ekf_config.yaml

```

**3.4 Check Topic**

```
ros2 topic echo /odometry/filtered

```

**3.5 Publish some speed**

**!!NOTE : Make sure to change the params file to your own path**

```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"

```

**4. AMCL TEST**

**4.1 Launch RPLidarA1**

```
ros2 launch rplidar_ros rplidar_a1_launch.py

```

**4.2 Simply launch everything**

```
ros2 launch food_del_robot_description display.launch.py

```

Select point estimation 2D then place the arrow point to the right heading of the robot in RViz.




