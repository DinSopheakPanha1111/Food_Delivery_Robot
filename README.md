# Food_Delivery_Robot
BY : DIN SOPHEAK PANHA

**SETUP ROBOT**

**Setup RPLidar ROS2 Humble**

```
sudo chmod 777 /dev/ttyUSB0
cd src/rpldiar_ros/
source scripts/create_udev_rules.sh

```

**Launch RPLidar**

```

ros2 launch rplidar_ros rplidar_a1_launch.py


```

**LAUNCH ROBOT**

```

ros2 launch food_del_robot_description robot.launch.py 


```

