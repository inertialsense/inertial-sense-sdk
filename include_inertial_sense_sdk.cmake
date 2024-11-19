find_library(SDK_LIBRARY_PATH InertialSenseSDK PATHS ${IS_SDK_DIR}/build)
if(NOT SDK_LIBRARY_PATH AND NOT TARGET InertialSenseSDK)
    # InertialSenseSDK library not prebuilt and not already included in project.  Include it in this project.
    add_subdirectory(${IS_SDK_DIR} InertialSenseSDK)
    message("IS-SDK library not prebuilt.  Including source in project.")
else()
    link_directories(${IS_SDK_DIR}/build)
    message("IS-SDK library prebuilt.")
endif()
