CMAKE_MINIMUM_REQUIRED(VERSION 2.8.13)

# Required: set the project
project(ISLogReaderExample)

# Required: set PROJECT_NAME to the same name as project in case of a multi-file build
set(PROJECT_NAME "ISLogReaderExample")

# Required: set the PROJECT_DIR to let the common cmake file know where this project lives
set(PROJECT_DIR "${CMAKE_CURRENT_LIST_DIR}")

# Required: set SOURCES_PROJECT to the code in this project, the files must go on their own lines
set(SOURCES_PROJECT	"${PROJECT_DIR}/ISLogReaderExample.cpp")

# Optional: add any libraries this project needs
#set(PROJECT_LIBS "")

# Everything is setup, use common template to build
include(../../CMakeCommon.txt)
