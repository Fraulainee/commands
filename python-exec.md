# Python executable 

Install PyInstaller
```
pip install pyinstaller
```
Create the Executable
```
pyinstaller --onefile --noconsole <yourfile>.py
```
Explanation:

    --onefile → Bundles everything into a single .exe file.
    --noconsole → Prevents the console/terminal from appearing when you run the app (important for GUI apps like Qt).

Handle Additional Resources (like images, icons, etc.)
```
pyinstaller --onefile --noconsole --add-data "path/to/resource;." <yourfile>.py
```

Add an Icon (Optional but recommended)
```
pyinstaller --onefile --noconsole --icon=icon.ico <yourfile>.py
```
=========================================================================================================================
Generate a .spec File
```
pyi-makespec --onefile --noconsole <yourfile>.py
```
Build Using the .spec File
```
pyinstaller <yourfile>.spec
```
