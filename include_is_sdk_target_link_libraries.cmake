# Find pthread package
find_package(Threads REQUIRED)

# Add Ws2_32 for networking
if(WIN32)
    target_link_libraries(${PROJECT_NAME} Ws2_32 Iphlpapi)
elseif(APPLE)
    # Required by libusb darwin backend (SecTask, CoreFoundation, IOKit)
    target_link_libraries(${PROJECT_NAME} "-framework CoreFoundation" "-framework IOKit" "-framework Security")
endif()

# Link InertialSenseSDK static library to the executable
target_link_libraries(${PROJECT_NAME} InertialSenseSDK Threads::Threads)
