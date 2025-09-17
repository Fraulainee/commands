# HOW TO RUN MY LIVOX SYSTEM

## UPDATE 1.0

### Run stair detection
```
roslaunch stair_detection stair_detector.launch
```

### Run different height detection
```
roslaunch stair_detection different_height.launch
```

### Send data to Arduino
cd to servo_comm directory
```
python pixhawk_motors.py
```

======================================================================




if using rosbag
### Run Roscore and Rviz
```
roscore
rviz
```

### Run all program
```
roslaunch stair_detection stair_detector.launch
```

### Run xy-plane segmentation
```
rosrun ransac_test ransac_stair_detection.py
```
### Run stair detection
```
roslaunch stair_detection stair_detector.launch
```


if using real-time lidar
### Run livox msg
```
roslaunch livox_ros_driver2 msg_MID360.launch
```
### Run FAST-LIO mapping
```
roslaunch fast_lio mapping_mid360.launch
```

