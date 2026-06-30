#!/bin/bash
function echo_blue   { echo -e "\033[1;34m$@\033[0m"; }

pushd "$(dirname "$(realpath $0)")" > /dev/null

echo_blue "==============================================="
echo_blue " CMake 3.26.0 (from source)                    "
echo_blue "==============================================="

curl -o /tmp/cmake-install.sh -L https://github.com/Kitware/CMake/releases/download/v3.26.0/cmake-3.26.0-linux-x86_64.sh \
      && chmod u+x /tmp/cmake-install.sh \
      && /tmp/cmake-install.sh --skip-license --prefix=/usr/bin \
      && rm /tmp/cmake-install.sh

if [[ -e ~/.bashrc ]]; then source ~/.bashrc; fi
popd > /dev/null