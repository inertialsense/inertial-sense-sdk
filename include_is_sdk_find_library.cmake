# Locate a prebuilt InertialSenseSDK static library, or fall back to building
# it from source as a sub-project. Either way, downstream consumers can link
# the SDK simply by referencing the target name `InertialSenseSDK`.
#
# Strategy: mirror the consumer's build-directory NAME under ${IS_SDK_DIR}.
# So if EvalTool is being configured at `cpp/EvalTool/cmake-build-debug/`, we
# look for (and, if absent, build into) `SDK/cmake-build-debug/`. This is
# platform- and IDE-agnostic, and it lets a previous source-build be picked
# up as a "prebuilt" on subsequent configures.
#
# Convention summary:
#   manual scripts (Linux):    cpp/<proj>/build         -> SDK/build
#   manual scripts (Windows):  cpp/<proj>/build-debug   -> SDK/build-debug
#   CLion / IDE:               cpp/<proj>/cmake-build-* -> SDK/cmake-build-*
#
# Note on dependency tracking: when a prebuilt static library is found, it is
# wrapped in an IMPORTED target — CMake treats it as opaque and will NOT
# rebuild the SDK if its source changes. This is intentional for fast CI /
# automated builds. Set IS_SDK_BUILD_FROM_SOURCE=ON to force the SDK to be
# built from source as a sub-project (with full dependency tracking) — useful
# when actively iterating on SDK source.
#
# - Default (no flag set): Prebuilt library is used as IMPORTED target. Fast, suitable for CI.
# - -DIS_SDK_BUILD_FROM_SOURCE=ON: Forces add_subdirectory, full dependency tracking, SDK rebuilds when its source changes.
#
# Usage
#
# # CI / fast builds (default — uses prebuilt if available)
# cmake -S cpp/EvalTool -B cpp/EvalTool/cmake-build-debug
#
# # Active SDK development (always rebuilds SDK on source change)
# cmake -S cpp/EvalTool -B cpp/EvalTool/cmake-build-debug -DIS_SDK_BUILD_FROM_SOURCE=ON
#
# In CLion, add -DIS_SDK_BUILD_FROM_SOURCE=ON to Settings → Build, Execution, Deployment → CMake → CMake options for the relevant profile when iterating on SDK code.
#
# A note worth flagging: because IS_SDK_BUILD_FROM_SOURCE uses CMake option(), its value is cached on first configure. To toggle it later, either pass -DIS_SDK_BUILD_FROM_SOURCE=OFF explicitly or clear CMakeCache.txt.


if(TARGET InertialSenseSDK)
    # Already set up by an earlier include of this file in the same project.
    return()
endif()

option(IS_SDK_BUILD_FROM_SOURCE
    "Always build InertialSenseSDK from source (skip prebuilt detection)" OFF)

get_filename_component(_consumer_build_name "${CMAKE_BINARY_DIR}" NAME)
set(_IS_SDK_BUILD_DIR "${IS_SDK_DIR}/${_consumer_build_name}")

if(IS_SDK_BUILD_FROM_SOURCE)
    add_subdirectory(${IS_SDK_DIR} ${_IS_SDK_BUILD_DIR})
    message(STATUS "IS_SDK_BUILD_FROM_SOURCE=ON. Building SDK source into: ${_IS_SDK_BUILD_DIR}")
    return()
endif()

unset(SDK_LIBRARY_PATH CACHE)
find_library(SDK_LIBRARY_PATH InertialSenseSDK
    PATHS ${_IS_SDK_BUILD_DIR}
    NO_DEFAULT_PATH
)

if(SDK_LIBRARY_PATH)
    # Wrap the prebuilt static library in an IMPORTED target so consumers can
    # link via `InertialSenseSDK` regardless of whether it was prebuilt or
    # built from source.
    add_library(InertialSenseSDK STATIC IMPORTED GLOBAL)
    # Mirror the PUBLIC include dirs from the SDK's own CMakeLists so that
    # targets linking InertialSenseSDK get the same paths whether the library
    # was built from source (where CMake propagates them automatically via
    # target_include_directories PUBLIC) or consumed as a prebuilt IMPORTED
    # target (where they must be set explicitly). Build as a proper CMake list
    # (one entry per set() arg) so no newline/indentation leaks into a path.
    set(_is_sdk_interface_includes
        "${IS_SDK_DIR}/src"
        "${IS_SDK_DIR}/src/util"
        "${IS_SDK_DIR}/src/protocol"
        "${IS_SDK_DIR}/src/libusb"
        "${IS_SDK_DIR}/src/libusb/libusb"
        "${IS_SDK_DIR}/src/yaml-cpp"
        "${IS_SDK_DIR}/src/tl-expected"
        "${IS_SDK_DIR}/tests/runtime"
    )
    set_target_properties(InertialSenseSDK PROPERTIES
        IMPORTED_LOCATION "${SDK_LIBRARY_PATH}"
        INTERFACE_INCLUDE_DIRECTORIES "${_is_sdk_interface_includes}"
    )
    message(STATUS "Using prebuilt InertialSenseSDK at: ${SDK_LIBRARY_PATH}")
else()
    # No prebuilt library — build the SDK from source as a sub-project.
    # Place artifacts under SDK/<consumer-build-name>/ so the result is
    # discoverable as a prebuilt on subsequent configures.
    add_subdirectory(${IS_SDK_DIR} ${_IS_SDK_BUILD_DIR})
    message(STATUS "InertialSenseSDK library NOT FOUND. Building SDK source into: ${_IS_SDK_BUILD_DIR}")
endif()