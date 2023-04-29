CMAKE_MINIMUM_REQUIRED(VERSION 2.8.13)

# Required: set the project
project(cltool)

# Required: set SOURCES_PROJECT to the code in this project, the files must go on their own lines
set(SOURCES_PROJECT "${CMAKE_CURRENT_LIST_DIR}/main.cpp")

# Optional: add any libraries this project needs
#set(PROJECT_LIBS "")

# Everything is setup, use common template to build
include(../CMakeCommon.txt)
