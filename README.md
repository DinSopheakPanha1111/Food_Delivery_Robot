# Food_Delivery_Robot
BY : DIN SOPHEAK PANHA

**1. SETUP ROBOT**

**1.1 Setup RPLidar ROS2 Humble**

```
sudo chmod 777 /dev/ttyUSB0
cd src/rpldiar_ros/
source scripts/create_udev_rules.sh
```

**1.2 Launch RPLidar**

```
ros2 launch rplidar_ros rplidar_a1_launch.py

```

**1.3 LAUNCH ROBOT IN SIMULATION**

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

**2.5 RPLIDAR A1 TEST**

```
ros2 run rplidar_ros rplidar_a1

```
