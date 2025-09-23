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
uv pip install -r requirements.txt
```
## Using llamacpp
Instructions for downloading this package can be found here:
https://github.com/abetlen/llama-cpp-python?tab=readme-ov-file#supported-backends
This ensures the that correct backend is installed (CUDA/Metal/etc).
Use `uv pip install`.