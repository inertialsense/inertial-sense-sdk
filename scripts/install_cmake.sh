#!/bin/bash
function echo_blue   { echo -e "\033[1;34m$@\033[0m"; }

pushd "$(dirname "$(realpath $0)")" > /dev/null

echo_blue "==============================================="
echo_blue " CMake 3.26.0 (from source)                    "
echo_blue "==============================================="

sudo apt -y install wget
wget https://github.com/Kitware/CMake/releases/download/v3.26.0/cmake-3.26.0-linux-x86_64.sh \
      -q -O /tmp/cmake-install.sh \
      && chmod u+x /tmp/cmake-install.sh \
      && mkdir /usr/bin/cmake \
      && /tmp/cmake-install.sh --skip-license --prefix=/usr/bin/cmake \
      && rm /tmp/cmake-install.sh

[[ -f ~/.bashrc ]] || source ~/.bashrc
popd > /dev/null