# Find pthread package
find_package(Threads REQUIRED)

# Add Ws2_32 for networking
add_definitions(-DYAML_CPP_STATIC_DEFINE -DCURL_STATICLIB)
if(WIN32)
    # Link InertialSenseSDK and all of it's dependencies to the executable
    target_link_libraries(${PROJECT_NAME} PRIVATE libcurl.lib Ws2_32.lib Iphlpapi.lib InertialSenseSDK Threads::Threads)
else()
    # Link InertialSenseSDK and all of it's dependencies to the executable
    target_link_libraries(${PROJECT_NAME} PRIVATE InertialSenseSDK Threads::Threads curl)
    check_library_exists(idn2 idn2_free "" HAVE_LIBRARY_IDN2)
    if(HAVE_LIBRARY_IDN2)
        target_link_libraries(${PROJECT_NAME} PRIVATE idn2)
    endif()
endif()