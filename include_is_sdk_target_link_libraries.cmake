# Find pthread package
find_package(Threads REQUIRED)

# Add Ws2_32 for networking
if(WIN32)
    target_link_libraries(${PROJECT_NAME} Ws2_32)
endif()

# Link InertialSenseSDK static library to the executable
target_link_libraries(${PROJECT_NAME} InertialSenseSDK Threads::Threads)
