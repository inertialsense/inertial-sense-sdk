CMAKE_MINIMUM_REQUIRED(VERSION 2.8.13)

project(InertialSense)
SET(PROJECT_NAME "InertialSense")

# Denote that all executables will be linking the sdk static library
SET(BUILD_INERTIAL_SENSE_SDK 1)

# We will be building the SDK static library first
SET(BUILD_LIB 1)
add_definitions(-DRTK_EMBEDDED)

# Get this project built (the static lib of the sdk)
include(CMakeCommon.txt)

# Go back to building executables
SET(BUILD_LIB 0)

# Build the other projects
add_subdirectory(cltool)
#add_subdirectory(ExampleProjects)
