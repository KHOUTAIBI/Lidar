cmake_minimum_required(VERSION 3.2)
PROJECT(Lidar)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
add_definitions(-std=c++17) # Use C++17


#Include directories
include_directories(
     ${CMAKE_SOURCE_DIR}
)


############## YDLIDAR SDK START#####################################
#find ydlidar_sdk package
find_package(ydlidar_sdk REQUIRED)

#Include directories
include_directories(
  ${YDLIDAR_SDK_INCLUDE_DIRS}
)

#link library directories
link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} lidar.cpp)


#Link your project to ydlidar_sdk library.
target_link_libraries(${PROJECT_NAME} PUBLIC ${YDLIDAR_SDK_LIBRARIES} )

############## YDLIDAR SDK END#####################################

