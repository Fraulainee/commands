

```
docker pull <image name>
```

getent group docker
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
ls -l /var/run/docker.sock

groups $USER
docker run -it --network=host --name=noetic_ws osrf/ros:noetic-desktop-full bash
docker exec -it noetic_ws bash
