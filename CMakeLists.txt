CMAKE_MINIMUM_REQUIRED(VERSION 3.10.0)

project(InertialSenseSDK)

file(GLOB SOURCES_SDK
    "src/*.c"
    "src/*.cpp"
    "src/*.h"
    "src/protocol/*.cpp"
    "src/protocol/*.h"
    "src/util/*.cpp"
    "src/util/*.h"
    "hw-libs/bootloader/bootloaderShared.c"
    "tests/runtime/*.cpp"
    "tests/runtime/*.h"
    "src/yaml-cpp/*.cpp"
    "src/yaml-cpp/*.h"
    "src/yaml-cpp/contrib/*.cpp"
    "src/yaml-cpp/contrib/*.h"
    "src/yaml-cpp/node/*.cpp"
    "src/yaml-cpp/node/*.h"
)

# Compile libusb from source
if (WIN32)
    file(GLOB SOURCES_LIB_USB
        "src/libusb/libusb/*.h"
        "src/libusb/libusb/*.c"
        "src/libusb/libusb/os/*windows*"
    )
else ()
    file(GLOB SOURCES_LIB_USB
        "src/libusb/libusb/*.h"
        "src/libusb/libusb/*.c"
        "src/libusb/libusb/os/*linux*"
        "src/libusb/libusb/os/*posix*"
    )
endif ()

# Ignore bootloader files
# list(FILTER SOURCES_SDK EXCLUDE REGEX "ISBootloader.*")
list(FILTER SOURCES_SDK EXCLUDE REGEX "ISBootloaderSony.*")
list(FILTER SOURCES_SDK EXCLUDE REGEX "ISBootloaderSTM32.*")

list(APPEND SOURCES_SDK ${SOURCES_LIB_USB})

# find_package(udev)
find_library(UDEV_LIB udev)

add_library(${PROJECT_NAME} STATIC ${SOURCES_SDK})

# Include paths
target_include_directories(${PROJECT_NAME} PUBLIC
    src
    src/util
    src/protocol
    src/libusb
    src/libusb/libusb
    src/yaml-cpp
    tests/runtime
    external
)

# Build using C11 and CXX20
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

# Other settings
if (WIN32)
    # yamlcpp needs to know it's a static lib.
    add_definitions(-DYAML_CPP_STATIC_DEFINE)

    # Windows specific include dir
    target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/src/libusb/msvc
    )

    # We need to ensure windows static libs are found
    target_link_libraries(${PROJECT_NAME} Ws2_32.lib)
else ()
    # Linux specific include dir
    target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/src/libusb/linux
    )

    set_source_files_properties(
        src/libusb/libusb/core.c
        PROPERTIES COMPILE_FLAGS "-Wno-unused-but-set-variable"
    )

    # Linux compiler flags
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-unused-function -Wno-unused-parameter -Wno-missing-field-initializers")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-function -Wno-unused-parameter -Wno-missing-field-initializers")

    # Set Linux compiler linker flag
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

    # Link in Linux specific packages
    target_link_libraries(${PROJECT_NAME} udev m)
endif ()
