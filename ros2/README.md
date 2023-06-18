## Setup

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/AchmadFathoni/inertial-sense-sdk --recurse-submodules
cd ..
colcon build --paths src/inertial-sense-sdk/ros2
source install/setup.bash
```
