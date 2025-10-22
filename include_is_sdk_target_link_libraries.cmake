# Find pthread package
find_package(Threads REQUIRED)

# Add Ws2_32 for networking
if(WIN32)
    target_link_libraries(${PROJECT_NAME} Ws2_32 Iphlpapi)
endif()

# Link InertialSenseSDK static library to the executable
set(IS_SDK_LIBRARIES InertialSenseSDK Threads::Threads)

if(APPLE)
    set(IS_SDK_LIBRARIES ${IS_SDK_LIBRARIES} "$<LINK_LIBRARY:FRAMEWORK,Foundation>" "$<LINK_LIBRARY:FRAMEWORK,IOKit>" "$<LINK_LIBRARY:FRAMEWORK,Security>")
endif()
target_link_libraries(${PROJECT_NAME} ${IS_SDK_LIBRARIES})
