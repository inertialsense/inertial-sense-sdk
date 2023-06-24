## Setup

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/AchmadFathoni/inertial-sense-sdk --recurse-submodules --branch ros2
cd ..
colcon build --paths src/inertial-sense-sdk/ros2 --symlink-install
source install/setup.bash
ros2 launch inertial_sense_ros example.launch.py
```
