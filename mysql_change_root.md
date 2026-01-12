# Make root work in Workbench

### First check what plugin root is using:
```
SELECT user, host, plugin FROM mysql.user WHERE user='root';
```

If you see auth_socket, change it:
```
ALTER USER 'root'@'localhost' IDENTIFIED WITH mysql_native_password BY 'YourRootPasswordHere!';
```
```
FLUSH PRIVILEGES;
```
