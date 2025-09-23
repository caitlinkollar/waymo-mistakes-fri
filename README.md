# waymo-mistakes-fri

## Install External Dependencies
After cloning, import all external dependencies:
```
vcs import src < ros2.repos
```
If `vcs` isn't on your system:
```
pip install vcstool
```
Modify the src/llama_ros/requirements.txt
```diff
- langchain-chroma==0.2.2
+ langchain-chroma==0.2.3
```
Additionally run these commands:
```
pip3 install -r src/llama_ros/requirements.txt
```