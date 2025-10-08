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

### Run pixhawk
cd to servo_comm directory
calibrate pixhawk and input 1500 as the stop value
```
python calibrate_pixhawk.py 
```

run pixhawk motors
```
python pixhawk_motors.py
```

### Send data to arduino
```
roslaunch servo_comm servo.launch
```

### Updated launch command for different height
```
roslaunch stair_detection different_height.launch   min_points_per_band:=12   min_centroid_sep_y:=0.05   min_height_diff:=0.03   z_window:=0.20
```

### Launch the different height
```
roslaunch stair_detection different_height.launch \
  min_points_per_band:=20 \
  min_centroid_sep_x:=0.06 \
  min_height_diff:=0.06 \
  z_window:=0.15
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

