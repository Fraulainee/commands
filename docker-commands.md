# Docker commands

check docker list
```
docker images -a
```

run docker image
```
docker run -it --name master ros:noetic
```


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

apply group changes
```
newgrp docker
```

verify docker socket permission
```
ls -l /var/run/docker.sock
```

check docker group membership
```
groups $USER
```

create docker connection
```
docker run -it --network=host --name=noetic_ws osrf/ros:noetic-desktop-full bash
```

attach to docker 
```
docker exec -it noetic_ws bash
```
