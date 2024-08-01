#!/bin/bash

##  This script is designed to setup the necessary requirements to build and run tests in a prestine environment, such as a docker container.
##  It is generally harmless, and should be fairly safe to use in a non-pristine environment as well.

# locate this scripts directory.
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# locate the catkin_ws directory in SCRIPT_DIR's parentage, and pushd to it..
CATKIN_PWD=${SCRIPT_DIR%catkin_ws*}
CATKIN_DIR=${CATKIN_PWD}/catkin_ws

apt-get update && apt -y install software-properties-common dirmngr apt-transport-https lsb-release ca-certificates libyaml-cpp-dev
add-apt-repository ppa:git-core/ppa -y && apt -y install git
source /opt/ros/jazzy/setup.bash
#apt -y install ros-${ROS_DISTRO}-tf libusb-1.0-0-dev
pushd $CATKIN_DIR
colcon build

. install/setup.bash

### Do other things, the run your tests.

