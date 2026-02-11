CMAKE_MINIMUM_REQUIRED(VERSION 3.16.3)

project(InertialSenseSDK)

# Define LINUX variable for use in this file
if(UNIX AND NOT APPLE)
	set(LINUX TRUE)
endif()

# set(CMAKE_FIND_DEBUG_MODE TRUE)
if(LINUX)
	find_package(UDev QUIET)	# try using find_package as a last resort (this seems to not exist on some platforms).
	if (NOT UDEV_FOUND)
		message(STATUS "`find_package` could not locate the libudev package. Trying again using 'pkg_check'.")
		find_package(PkgConfig QUIET)
		pkg_check_modules(UDEV libudev)
		if (NOT UDEV_FOUND)
			message(STATUS "'pkg_check' could not locate the libudev package. Trying again using 'find_library'.")
			find_library(UDEV_LIB udev)
			if (NOT UDEV_LIB)
				message(STATUS "'find_library' could not locate the libudev library. Trying again using 'find_library'.")
			else()
				SET(UDEV_LIBRARIES udev)
				SET(UDEV_FOUND 1)
			endif()
		endif()
	endif()

	if (UDEV_FOUND)
		message(STATUS "libudev found:\n     Libraries: ${UDEV_LIBRARIES}\n     Includes Path: ${UDEV_INCLUDE_DIRS}\n     Libraries Path: ${UDEV_LIBRARY_DIRS}")
	else()
		message(FATAL_ERROR "UDev not found. Have you ran the \"install_sdk_dependencies\" script for your platform?")
	endif()
endif()

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
	# Suppress specific warnings for libusb (third-party code)
	if(MSVC)
		set_source_files_properties(${SOURCES_LIB_USB} PROPERTIES COMPILE_FLAGS "/wd5287")
	endif()
elseif(APPLE)
	file(GLOB SOURCES_LIB_USB
		"${CMAKE_CURRENT_LIST_DIR}/src/libusb/libusb/*.h"
		"${CMAKE_CURRENT_LIST_DIR}/src/libusb/libusb/*.c"
		"${CMAKE_CURRENT_LIST_DIR}/src/libusb/libusb/os/*darwin*"
		"${CMAKE_CURRENT_LIST_DIR}/src/libusb/libusb/os/*posix*"
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

# Exclude ISDeviceCal (Qt-dependent code, moved to standard C++ but not needed for SDK builds)
# list(FILTER SOURCES_SDK EXCLUDE REGEX "ISDeviceCal.*")

list(APPEND SOURCES_SDK ${SOURCES_LIB_USB})

# find_package(udev)
add_library(${PROJECT_NAME} STATIC ${SOURCES_SDK})

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

# Build using C11 and CXX17 for all platforms
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

if(NOT WIN32)
	target_compile_options(${PROJECT_NAME} INTERFACE -Wno-deprecated-declarations)
endif()

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
			COMMAND cmd /C scripts\\lib\\update_version_files.bat -p IS_SDK_ "${CMAKE_CURRENT_SOURCE_DIR}/src/version/"
			DEPENDS scripts/lib/version_tools.py
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	)
	add_custom_target(gen_files
			DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/version/repositoryInfo.h"
	)
	add_dependencies(${PROJECT_NAME} gen_files)

    # We need to ensure windows static libs are found
    target_link_libraries(${PROJECT_NAME} Ws2_32.lib Iphlpapi.lib)
elseif(APPLE)
    # macOS specific include dir
    target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/src/libusb/Xcode
			${UDEV_INCLUDE_DIRS}
			${UDEV_LIB_INCLUDE}
    )

	add_custom_command(
			OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/src/version/repositoryInfo.h"
			COMMAND bash -c "scripts/lib/update_version_files.sh -p IS_SDK_ ${CMAKE_CURRENT_SOURCE_DIR}/src/version/"
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

    # macOS compiler flags
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-unused-function -Wno-unused-parameter -Wno-missing-field-initializers -Wno-deprecated-declarations")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-function -Wno-unused-parameter -Wno-missing-field-initializers -Wno-deprecated-declarations")

    # Set macOS compiler linker flag
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

	# Link in macOS specific packages
    target_link_libraries(${PROJECT_NAME} PUBLIC "$<LINK_LIBRARY:FRAMEWORK,Foundation>" "$<LINK_LIBRARY:FRAMEWORK,IOKit>" "$<LINK_LIBRARY:FRAMEWORK,Security>")
elseif(LINUX)
    # Linux specific include dir
    target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/src/libusb/linux
			${UDEV_INCLUDE_DIRS}
			${UDEV_LIB_INCLUDE}
    )

	add_custom_command(
			OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/src/version/repositoryInfo.h"
			COMMAND bash -c "scripts/lib/update_version_files.sh -p IS_SDK_ ${CMAKE_CURRENT_SOURCE_DIR}/src/version/"
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
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-unused-function -Wno-unused-parameter -Wno-missing-field-initializers -Wno-deprecated-declarations")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-function -Wno-unused-parameter -Wno-missing-field-initializers -Wno-deprecated-declarations")

    # Set Linux compiler linker flag
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

	# Link in Linux specific packages
    target_link_libraries(${PROJECT_NAME} ${UDEV_LIBRARIES} m)
endif ()
