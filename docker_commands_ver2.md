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
