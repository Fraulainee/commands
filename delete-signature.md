# Remove the old key (optional):
```
sudo apt-key del F42ED6FBAB17C654
```

# Add the new key:
```
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

### If apt-key is deprecated on your system, you can add the key this way instead:
```
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null
```

# Then edit your ROS repo list to use this keyring:
```
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros-latest.list
```
