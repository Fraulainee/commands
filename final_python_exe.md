# Creating python executable from python ui using qt designer

### Create python venv
```
python -m venv myenv
```

### Activate python venv
```
myenv\Scripts\activate
```
### Install python installer using pip and the required dependencies
```
pip install pyqt5 pyinstaller
```

### Create a .spec File (Optional)
Run this once to generate a .spec file:
```
pyinstaller yourscript.py
```
Edit yourscript.spec and add this line under datas=[]:
```
datas=[('maingui.ui', '.')],
```
or
```
datas=[
    ('maingui.ui', '.'),               # source, target path inside build
    ('assets/icon.png', 'assets')     # include images etc.
],
```

###  Build the .exe with PyInstaller
```
pyinstaller --onefile --noconsole yourscript.py
```
If you're including external files, and you edited the .spec file, use:
```
pyinstaller yourscript.spec
```
or
```
pyinstaller --onefile --add-data "maingui.ui;." rutting.py
```
