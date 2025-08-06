# Find pthread package
find_package(Threads REQUIRED)

# Add Ws2_32 for networking
add_definitions(-DYAML_CPP_STATIC_DEFINE -DCURL_STATICLIB)
if(WIN32)
    # Link InertialSenseSDK and all of it's dependencies to the executable
    target_link_libraries(${PROJECT_NAME} libcurl.lib Ws2_32.lib Iphlpapi.lib InertialSenseSDK Threads::Threads)
else()
    # Link InertialSenseSDK and all of it's dependencies to the executable
    target_link_libraries(${PROJECT_NAME} InertialSenseSDK Threads::Threads curl idn2)
endif()



