# Find pthread package
find_package(Threads REQUIRED)

# Add Ws2_32 for networking
if(WIN32)
    target_link_libraries(${PROJECT_NAME}  libcurl.lib Ws2_32.lib Iphlpapi.lib)
    add_definitions(-DYAML_CPP_STATIC_DEFINE -DCURL_STATICLIB)
endif()

# Link InertialSenseSDK static library to the executable
target_link_libraries(${PROJECT_NAME} InertialSenseSDK Threads::Threads)
