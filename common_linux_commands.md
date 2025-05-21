# Linux commands

### check ports
```
dmesg | grep tty
```

### Check group ownership of the device
```
ls -l /dev/ttyACM0
```
You'll probably see something like:
```
crw-rw---- 1 root dialout ... /dev/ttyACM0
```
That means only users in the dialout group can access it.

### Add your user to the dialout group
```
sudo usermod -a -G dialout $USER
```
### Make the device world-readable and writable
```
sudo chmod 666 /dev/ttyACM0
```
### Run rosserial
```
rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=9600
```
