# create and compile a resources_rc.py

Compile the .qrc File into resources_rc.py
```
pyrcc5 -o resources_rc.py resources.qrc
```
check version
```
pip list | grep -E "PySide2|shiboken2"
```
