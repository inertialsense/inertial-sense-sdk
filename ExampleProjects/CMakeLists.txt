CMAKE_MINIMUM_REQUIRED(VERSION 2.8.13)

project(ExampleProjects)
SET(PROJECT_NAME "ExampleProjects")

# Add all the other sub projects
add_subdirectory(Ascii)
add_subdirectory(Bootloader)
add_subdirectory(Communications)
add_subdirectory(Logger)
add_subdirectory(LogReader)
add_subdirectory(NTRIP_rover)
