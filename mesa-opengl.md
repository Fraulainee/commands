
Ensure NVIDIA Drivers Are Installed Correctly
```
nvidia-smi
```

Verify NVIDIA OpenGL is Active
```
glxinfo | grep "OpenGL"
```
Install NVIDIA GL Libraries
```
sudo apt install --reinstall nvidia-driver-535 nvidia-settings nvidia-prime
```
