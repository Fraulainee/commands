#Docker commands

check if docker 
```
docker pull <image name>
```

check if docker group exist
```
getent group docker
```

if it doesnt exist, create it
```
sudo groupadd docker
```

add your user to the docker group
```
sudo usermod -aG docker $USER
```


newgrp docker
ls -l /var/run/docker.sock

groups $USER
docker run -it --network=host --name=noetic_ws osrf/ros:noetic-desktop-full bash
docker exec -it noetic_ws bash
