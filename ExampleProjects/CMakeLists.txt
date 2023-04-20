CMAKE_MINIMUM_REQUIRED(VERSION 2.8.13)

project(ExampleProjects)
SET(PROJECT_NAME "ExampleProjects")

# Use sdk static lib
SET(BUILD_INERTIAL_SENSE_SDK 1)

# Add all the other sub projects
add_subdirectory(Ascii)
add_subdirectory(Bootloader)
add_subdirectory(Communications)
add_subdirectory(Logger)
add_subdirectory(LogReader)
add_subdirectory(NTRIP_rover)
