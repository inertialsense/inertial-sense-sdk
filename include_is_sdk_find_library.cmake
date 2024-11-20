if(WIN32)
    set(IS_SDK_BUILD_DIR "${IS_SDK_DIR}/build/Release")
else()
    set(IS_SDK_BUILD_DIR "${IS_SDK_DIR}/build")
endif()

find_library(SDK_LIBRARY_PATH InertialSenseSDK PATHS ${IS_SDK_BUILD_DIR})

if(NOT SDK_LIBRARY_PATH AND NOT TARGET InertialSenseSDK)
    # InertialSenseSDK library not prebuilt and not already included in project.  Include it in this project.
    add_subdirectory(${IS_SDK_DIR} InertialSenseSDK)
    message("IS-SDK library is not prebuilt.  Including source in project.")
else()
    link_directories(${IS_SDK_BUILD_DIR})
    message("IS-SDK library is prebuilt at: ${SDK_LIBRARY_PATH}")
endif()