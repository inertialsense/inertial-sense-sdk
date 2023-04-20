CMAKE_MINIMUM_REQUIRED(VERSION 2.8.13)

project(InertialSense)

SET(BUILD_INERTIAL_SENSE_SDK 1)
SET(BUILD_LIB 1)
add_definitions(-DRTK_EMBEDDED)

# get this project built
include(CMakeCommon.text)

# build some more projects
add_subdirectory(cltool)
add_subdirectory(ExampleProjects)
