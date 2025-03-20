# create and compile a resources_rc.py

Compile the .qrc file into resources_rc.py using PyQt5
```
pyrcc5 -o resources_rc.py resources.qrc
```

Compile the .qrc file into resources_rc.py using PySide2

```
pyside2-rcc resources.qrc -o resources_rc.py
```

check version
```
pip list | grep -E "PySide2|shiboken2"
```

Uninstall PySide2 and shiboken2 --- this causes conflict with pyqt5
```
pip uninstall PySide2 shiboken2 -y
```
convert ui to python pyqt5
```
pyuic5 -x splash_art.ui -o ui_splash_art.py
```

update PySide2
```
pip install --no-cache-dir PySide2 shiboken2
```
Install Missing Qt and OpenGL Dependencies
```
conda install -n pyqt5-env -c conda-forge pyqt=5.15 pyqtwebengine xorg-libx11 xorg-libxext xorg-xcb-util xorg-xcb-util-wm xorg-libxrender mesa
```
