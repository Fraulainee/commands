# Docker commands version 2

Install Docker (If Not Installed)
```
sudo apt update
sudo apt install -y docker.io
```

Start and enable Docker service:
```
sudo systemctl start docker
sudo systemctl enable docker
```

Verify Docker installation:
```
docker --version
```

Allow running Docker without sudo (optional):
```
sudo usermod -aG docker $USER
```

Remove docker image
```
docker rmi ros:noetic
```

Run the ROS Noetic Container
```
docker run -it --rm ros:noetic bash
```
  -it → Interactive mode with a terminal.
  --rm → Automatically removes the container after exiting.

If you want to keep the container running persistently:
```
docker run -it --name ros_noetic_container ros:noetic bash
```
  --name ros_noetic_container → Names the container for easier reference.

## Find Installed Docker Images and Containers
List all downloaded Docker images:
```
docker images
```
Check running containers:
```
docker ps
```
List all containers (including stopped ones):
```
docker ps -a
```

## Uninstall or Remove ROS Noetic Docker Image
Stop any running containers using the image:
```
docker ps
docker stop <container_id>
```
Remove the container
```
docker rm ros_noetic_container
```
Remove the Docker image
```
docker rmi osrf/ros:noetic-desktop-full
```
If you want to completely remove Docker from your system:
```
sudo apt remove --purge -y docker.io
sudo rm -rf /var/lib/docker
```


