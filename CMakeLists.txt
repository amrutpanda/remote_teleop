cmake_minimum_required(VERSION 3.15)
project(remote-telop)
set(CMAKE_BUILD_TYPE "Debug")
set(CXX_STANDARD_REQUIRED 20)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

list(APPEND CMAKE_PREFIX_PATH "/home/merai/Files/C++/chai3d/build")
find_package(chai3d REQUIRED)
include_directories(${CHAI3D_INCLUDE_DIRS})
link_directories(${CHAI3D_LIBRARY_DIRS})

include_directories(src)

find_library(HIREDIS_LIB NAMES hiredis)

add_executable(chai_haptics_redis_driver src/chai_haptics_redis_driver.cpp src/redisclient.cpp)
target_link_libraries(chai_haptics_redis_driver ${CHAI3D_LIBRARIES} ${HIREDIS_LIB})

add_executable(haptic_controller src/haptic_controller.cpp src/HapticController.cpp src/redisclient.cpp)
target_link_libraries(haptic_controller ${CHAI3D_LIBRARIES} ${HIREDIS_LIB})