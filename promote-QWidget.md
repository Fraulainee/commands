# Promote the QWidget to MatplotlibWidget

1. Right-click the placeholder QWidget in the Object Inspector.
2. Select Promote to...
3. In the Promoted class name field, enter:
```
MatplotlibWidget
```

4. In the Header file field, enter:
```
matplotlibwidget
```
(Note: Do not include .py in the header name; Qt Designer expects a module reference, not a file path.)

5. Click Add and then Promote.

6. Regenerate Your Python File (If Needed)
```
pyuic5 -x maingui.ui -o maingui.py
```
