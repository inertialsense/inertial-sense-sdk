cmake_minimum_required(VERSION 3.16)
project(rtk_reconnect_demo CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Point these at an uninstalled SDK tree to test a library you just built, e.g.
#   -DIS_SDK_INCLUDE_DIR=/path/to/inertial-sense-sdk/src
#   -DIS_SDK_LIBRARY=/path/to/inertial-sense-sdk/build/libInertialSenseSDK.a
# Left unset, the installed package is used. The include directory must be the one in which
# <inertial-sense-sdk/...> resolves, so for an installed package that is the parent directory.
# Searched without explicit PATHS so that CMAKE_PREFIX_PATH works for an SDK staged somewhere other
# than /usr. The results land in differently named variables because find_path() and find_library()
# skip the search entirely when their output variable is already in the cache.
if(NOT IS_SDK_INCLUDE_DIR)
    find_path(IS_SDK_FOUND_INCLUDE_DIR NAMES inertial-sense-sdk/NtripCorrectionService.h)
    set(IS_SDK_INCLUDE_DIR "${IS_SDK_FOUND_INCLUDE_DIR}")
endif()
if(NOT IS_SDK_LIBRARY)
    find_library(IS_SDK_FOUND_LIBRARY NAMES InertialSense InertialSenseSDK)
    set(IS_SDK_LIBRARY "${IS_SDK_FOUND_LIBRARY}")
endif()

if(NOT IS_SDK_INCLUDE_DIR OR NOT IS_SDK_LIBRARY)
    message(FATAL_ERROR
        "Inertial Sense SDK not found. Install the package, or pass "
        "-DIS_SDK_INCLUDE_DIR=<dir> -DIS_SDK_LIBRARY=<file>.")
endif()
message(STATUS "SDK headers: ${IS_SDK_INCLUDE_DIR}")
message(STATUS "SDK library: ${IS_SDK_LIBRARY}")

# An uninstalled tree has no inertial-sense-sdk/ prefix directory, so synthesise one as a symlink
# and compile against that. Keeps the demo's #include lines identical in both layouts.
if(NOT EXISTS "${IS_SDK_INCLUDE_DIR}/inertial-sense-sdk/NtripCorrectionService.h")
    set(_shim "${CMAKE_CURRENT_BINARY_DIR}/sdk-include")
    file(MAKE_DIRECTORY "${_shim}")
    if(NOT EXISTS "${_shim}/inertial-sense-sdk")
        file(CREATE_LINK "${IS_SDK_INCLUDE_DIR}" "${_shim}/inertial-sense-sdk" SYMBOLIC)
    endif()
    set(IS_SDK_ROOT_INCLUDE_DIR "${IS_SDK_INCLUDE_DIR}")
    set(IS_SDK_INCLUDE_DIR "${_shim}")
    message(STATUS "SDK headers (shimmed): ${IS_SDK_INCLUDE_DIR}")
else()
    # SDK headers include each other with paths relative to their own root, e.g. base_port.h does
    # #include "core/types.h", so that root has to be on the include path as well.
    set(IS_SDK_ROOT_INCLUDE_DIR "${IS_SDK_INCLUDE_DIR}/inertial-sense-sdk")
endif()

# In an installed package the vendored header-only dependencies are flattened next to the SDK
# headers; in a source tree they still sit in their own directories, which have to be added.
foreach(_vendored tl-expected)
    if(IS_DIRECTORY "${IS_SDK_ROOT_INCLUDE_DIR}/${_vendored}")
        list(APPEND IS_SDK_EXTRA_INCLUDE_DIRS "${IS_SDK_ROOT_INCLUDE_DIR}/${_vendored}")
    endif()
endforeach()

find_package(Threads REQUIRED)
# The SDK's public headers include "libusb.h" unqualified, so libusb's include directory has to be
# on the path of anything that includes them.
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

add_executable(rtk_reconnect_demo rtk_reconnect_demo.cpp)
target_compile_options(rtk_reconnect_demo PRIVATE -Wall -Wextra)
# SYSTEM so that the SDK's own header warnings do not drown out the demo's.
target_include_directories(rtk_reconnect_demo SYSTEM PRIVATE
    "${IS_SDK_INCLUDE_DIR}"
    "${IS_SDK_ROOT_INCLUDE_DIR}"
    ${IS_SDK_EXTRA_INCLUDE_DIRS}
    ${LIBUSB_INCLUDE_DIRS}
)
target_link_libraries(rtk_reconnect_demo PRIVATE
    "${IS_SDK_LIBRARY}"
    Threads::Threads
    usb-1.0
    yaml-cpp
    udev
    m
)
