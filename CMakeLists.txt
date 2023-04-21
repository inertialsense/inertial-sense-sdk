CMAKE_MINIMUM_REQUIRED(VERSION 2.8.13)

# Required: set the project
project(InertialSense)

# Required: set PROJECT_NAME to the same name as project in case of a multi-file build
set(PROJECT_NAME "InertialSense")

# Required: set the PROJECT_DIR to let the common cmake file know where this project lives
set(PROJECT_DIR "${CMAKE_CURRENT_LIST_DIR}")

# We will be building the SDK as a static library
set(BUILD_LIB 1)

# Custom definitions
add_definitions(-DRTK_EMBEDDED)

# Required: set SOURCES_PROJECT, since we are the SDK, we don't need any files, but we add one to keep cmake happy
set(SOURCES_PROJECT "${PROJECT_DIR}/src/InertialSense.h")

# Optional: add any libraries this project needs
#set(PROJECT_LIBS "")

# Get this project built (the static lib of the sdk)
include(${PROJECT_DIR}/CMakeCommon.txt)

# Go back to building executables
set(BUILD_LIB 0)

# Build example projects if not ignored
if(NOT IGNORE_EXAMPLE_PROJECTS)

	# Denote that all executables will be linking the sdk static library and not the sources directly
	set(BUILD_INERTIAL_SENSE_SDK 1)

	# Build the other projects
	add_subdirectory(cltool)
	add_subdirectory(ExampleProjects)

endif()
