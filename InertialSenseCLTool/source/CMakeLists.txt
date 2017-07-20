add_executable(cltool main.cpp)
add_subdirectory(../../ ${CMAKE_CURRENT_BINARY_DIR}/SDK)

target_link_libraries(cltool ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(cltool InertialSense)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions")

set_property(TARGET cltool PROPERTY COMPILE_DEFINITIONS
  fvisibility=hidden
  XPLM200
  XPLM210
  APL=0
  IBM=0
  LIN=1
  )
