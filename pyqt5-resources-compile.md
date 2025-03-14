# create and compile a resources_rc.py

Compile the .qrc File into resources_rc.py
```
pyrcc5 -o resources_rc.py resources.qrc
```
check version
```
pip list | grep -E "PySide2|shiboken2"
```

Uninstall PySide2 and shiboken2 --- this causes conflict with pyqt5
```
pip uninstall PySide2 shiboken2 -y
```
