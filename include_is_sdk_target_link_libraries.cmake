# Find pthread package
find_package(Threads REQUIRED)

# Add Ws2_32 for networking
if(WIN32)
    target_link_libraries(${PROJECT_NAME} Ws2_32)
elseif(APPLE)
    find_library(IOKIT_FRAMEWORK IOKit)
    find_library(COREFOUNDATION_FRAMEWORK CoreFoundation)
    find_library(SECURITY_FRAMEWORK Security)
    target_link_libraries(${PROJECT_NAME}
        ${IOKIT_FRAMEWORK}
        ${COREFOUNDATION_FRAMEWORK}
        ${SECURITY_FRAMEWORK}
    )
endif()

# Link InertialSenseSDK static library to the executable
target_link_libraries(${PROJECT_NAME} InertialSenseSDK Threads::Threads)
