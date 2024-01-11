#!/bin/bash
function echo_blue   { echo -e "\033[1;34m$@\033[0m"; }

pushd "$(dirname "$(realpath $0)")" > /dev/null

echo_blue "==============================================="
echo_blue " GTest                                         "
echo_blue "==============================================="
sudo apt update
sudo apt install -y libgtest-dev
# Use the following to remove gtest:
# sudo apt-get autoremove -y libgtest-dev
# sudo apt-get autoremove -y --purge libgtest-dev
pushd /usr/src/googletest/googletest > /dev/null
  sudo cmake CMakeLists.txt
  # sudo make clean
  sudo make
  if [ -d "./lib/" ]; then
    sudo cp lib/*.a /usr/lib    # Ubuntu 20
  else
    sudo cp *.a /usr/lib        # Ubuntu 18
  fi
popd > /dev/null

source ~/.bashrc

popd > /dev/null
