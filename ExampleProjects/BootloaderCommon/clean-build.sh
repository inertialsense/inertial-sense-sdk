#!/usr/bin/env bash

rm -rf build && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Debug .. && make -j $(grep ^processor /proc/cpuinfo  | wc -l) && cd .. && chown -R 1000 build