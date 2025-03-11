# Creating an environment with commands

To create an environment
```
conda create --name <my-env>
```
To create an environment with a specific version of Python:
```
conda create -n myenv python=3.9
```
To create an environment with a specific package:
```
conda create -n myenv scipy
```
To create an environment with a specific version of Python and multiple packages:
```
conda create -n myenv python=3.9 scipy=0.17.3 astroid babel
```
