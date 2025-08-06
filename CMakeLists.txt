CMAKE_MINIMUM_REQUIRED(VERSION 3.21.0)

project(InertialSenseSDK)

file(GLOB SOURCES_SDK
	"${CMAKE_CURRENT_LIST_DIR}/src/*.c"
	"${CMAKE_CURRENT_LIST_DIR}/src/*.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/src/*.h"
	"${CMAKE_CURRENT_LIST_DIR}/src/core/*.c"
	"${CMAKE_CURRENT_LIST_DIR}/src/core/*.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/src/core/*.h"
	"${CMAKE_CURRENT_LIST_DIR}/src/protocol/*.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/src/protocol/*.h"
	"${CMAKE_CURRENT_LIST_DIR}/src/util/*.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/src/util/*.h"
	"${CMAKE_CURRENT_LIST_DIR}/hw-libs/bootloader/bootloaderShared.c"
	"${CMAKE_CURRENT_LIST_DIR}/tests/runtime/*.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/tests/runtime/*.h"
	"${CMAKE_CURRENT_LIST_DIR}/src/yaml-cpp/*.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/src/yaml-cpp/*.h"
	"${CMAKE_CURRENT_LIST_DIR}/src/yaml-cpp/contrib/*.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/src/yaml-cpp/contrib/*.h"
	"${CMAKE_CURRENT_LIST_DIR}/src/yaml-cpp/node/*.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/src/yaml-cpp/node/*.h"
)

# Compile libusb from source
if(WIN32)
	file(GLOB SOURCES_LIB_USB
		"${CMAKE_CURRENT_LIST_DIR}/src/libusb/libusb/*.h"
		"${CMAKE_CURRENT_LIST_DIR}/src/libusb/libusb/*.c"
		"${CMAKE_CURRENT_LIST_DIR}/src/libusb/libusb/os/*windows*"
	)
else()
	file(GLOB SOURCES_LIB_USB
		"${CMAKE_CURRENT_LIST_DIR}/src/libusb/libusb/*.h"
		"${CMAKE_CURRENT_LIST_DIR}/src/libusb/libusb/*.c"
		"${CMAKE_CURRENT_LIST_DIR}/src/libusb/libusb/os/*linux*"
		"${CMAKE_CURRENT_LIST_DIR}/src/libusb/libusb/os/*posix*"
	)
endif()

# Ignore bootloader files
# list(FILTER SOURCES_SDK EXCLUDE REGEX "ISBootloader.*")
list(FILTER SOURCES_SDK EXCLUDE REGEX "ISBootloaderSony.*")
list(FILTER SOURCES_SDK EXCLUDE REGEX "ISBootloaderSTM32.*")

list(APPEND SOURCES_SDK ${SOURCES_LIB_USB})

# find_package(udev)
find_library(UDEV_LIB udev)
find_library(MDNS_LIB mdns PATHS src/libmdns)

add_library(${PROJECT_NAME} STATIC ${SOURCES_SDK})

# Add CURL for parsing URLs (cause I couldn't find any other URL parsers that supported IPv6
set(BUILD_CURL_EXE OFF)
set(CURL_DISABLE_INSTALL ON)
set(CURL_DISABLE_HTTP ON)
set(HTTP_ONLY ON)
set(CURL_DISABLE_ALTSVC ON)
set(CURL_DISABLE_SRP ON)
set(CURL_DISABLE_SRP ON)
set(CURL_DISABLE_BASIC_AUTH ON)
set(CURL_DISABLE_BEARER_AUTH ON)
set(CURL_DISABLE_DIGEST_AUTH ON)
set(CURL_DISABLE_KERBEROS_AUTH ON)
set(CURL_DISABLE_NEGOTIATE_AUTH ON)
set(CURL_DISABLE_AWS ON)
set(CURL_DISABLE_DICT ON)
set(CURL_DISABLE_DOH ON)
set(CURL_DISABLE_GETOPTIONS ON)
set(CURL_DISABLE_HEADERS_API ON)
set(CURL_DISABLE_HSTS ON)
set(CURL_DISABLE_HTTP_AUTH ON)
set(CURL_DISABLE_LIBCURL_OPTION ON)
set(CURL_DISABLE_MIME ON)
set(CURL_DISABLE_FORM_API ON)
set(CURL_DISABLE_BINDLOCAL ON)
set(CURL_DISABLE_NETRC ON)
set(CURL_DISABLE_NTLM ON)
set(CURL_DISABLE_PARSEDATE ON)
set(CURL_DISABLE_PROXY ON)
set(CURL_DISABLE_PROGRESS_METER ON)
set(CURL_DISABLE_SHA512_256 ON)
set(CURL_DISABLE_SHUFFLE_DNS ON)
set(CURL_DISABLE_SOCKETPAIR ON)
set(CURL_DISABLE_VERBOSE_STRINGS ON)
set(CURL_DISABLE_WEBSOCKETS ON)
set(BUILD_LIBCURL_DOCS OFF)
set(BUILD_MISC_DOCS OFF)
set(ENABLE_CURL_MANUAL OFF)
set(ENABLE_IPV6 ON)
set(CURL_ENABLE_SSL OFF)
set(CURL_USE_SCHANNEL OFF)
set(USE_NGHTTP2 OFF)
set(USE_LIBIDN2 ON)
set(CURL_USE_LIBPSL OFF)
set(CURL_USE_LIBSSH2 OFF)
set(ENABLE_UNIX_SOCKETS OFF)
set(CURL_ZLIB OFF)
set(ZLIB_FOUND OFF)
set(CURL_BROTLI OFF)
set(BROTLI_FOUND OFF)
set(CURL_ZSTD OFF)
set(ZSTD_FOUND OFF)
set(BUILD_SHARED_LIBS OFF)
set(BUILD_STATIC_LIBS ON)
add_subdirectory(src/curl)
target_link_libraries(${PROJECT_NAME} PRIVATE libcurl_static)

# Include paths
target_include_directories(${PROJECT_NAME} PUBLIC
        src
        src/util
        src/protocol
        src/libusb
        src/libusb/libusb
        src/yaml-cpp
        tests/runtime
        external
)

# Build using C11 and CXX20
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 17)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

# Other settings
if (WIN32)
    # yamlcpp needs to know it's a static lib.
    add_definitions(-DYAML_CPP_STATIC_DEFINE)

    # Windows specific include dir
    target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/src/libusb/msvc
    )

	add_custom_command(
			OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/src/version/repositoryInfo.h"
			COMMAND cmd /C "scripts\\lib\\update_version_files.bat ${CMAKE_CURRENT_SOURCE_DIR}/src/version/"
			DEPENDS scripts/lib/version_tools.py
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	)
	add_custom_target(gen_files
			DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/version/repositoryInfo.h"
	)
	add_dependencies(${PROJECT_NAME} gen_files)

    # We need to ensure windows static libs are found
    target_link_libraries(${PROJECT_NAME} Ws2_32.lib)
else ()
    # Linux specific include dir
    target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/src/libusb/linux
    )

	add_custom_command(
			OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/src/version/repositoryInfo.h"
			COMMAND bash -c "scripts/lib/update_version_files.sh ${CMAKE_CURRENT_SOURCE_DIR}/src/version/"
			DEPENDS scripts/lib/version_tools.py
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	)
	add_custom_target(gen_files
			DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/version/repositoryInfo.h"
	)
	add_dependencies(${PROJECT_NAME} gen_files)
    set_source_files_properties(
        src/libusb/libusb/core.c
        PROPERTIES COMPILE_FLAGS "-Wno-unused-but-set-variable"
    )

    # Linux compiler flags
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-unused-function -Wno-unused-parameter -Wno-missing-field-initializers")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-function -Wno-unused-parameter -Wno-missing-field-initializers")

    # Set Linux compiler linker flag
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

    # Link in Linux specific packages
    target_link_libraries(${PROJECT_NAME} PRIVATE udev m)
endif ()
