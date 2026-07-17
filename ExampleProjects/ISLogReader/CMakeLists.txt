# ExampleProjects/ISLogReader/CMakeLists.txt — D-02 / SN-7893 stub.
#
# The full example program is added by D-11 / SN-7900. Keeping this
# file (mostly empty) means the parent ExampleProjects/CMakeLists.txt
# can `add_subdirectory(ISLogReader)` pre-emptively without a
# missing-file error if it ever wants to.
#
# Once D-11 lands, replace the body of this file with the actual
# add_executable + target_link_libraries lines. See the README.md in
# this directory for the example source.

cmake_minimum_required(VERSION 3.16)
project(ISLogReader_example LANGUAGES CXX)

# Intentional no-op — D-11 will turn the README.md's quickstart into
# a real binary target here.
message(STATUS "ExampleProjects/ISLogReader: stub (D-02); full example lands with D-11.")
