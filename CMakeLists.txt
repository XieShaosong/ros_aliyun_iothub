# Declare the version of the CMake API for forward-compatibility
cmake_minimum_required(VERSION 3.1)
# Declare the name of the CMake Project
project(ros_aliyun_iothub)
set(CMAKE_CXX_FLAGS "${CMAKC_CXX_FLAGS} -std=c++11 -fpermissive")

# Find and get all the information about the roscpp package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg 
)

add_message_files(
    FILES
    status.msg
)

add_service_files(
    FILES
    control.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
# Find Catkin
# Declare this project as a catkin package
catkin_package(
#  CATKIN_DEPENDS
)

# Add the headers from roscpp
include_directories(include include/iot_util iot_core/include ${roscpp_INCLUDE_DIRS})

link_libraries(${CMAKE_SOURCE_DIR}/iot_core/lib/libiot_sdk.a)
link_libraries(${CMAKE_SOURCE_DIR}/iot_core/lib/libiot_nghttp2.a)
link_libraries(${CMAKE_SOURCE_DIR}/iot_core/lib/libiot_hal.a)
link_libraries(${CMAKE_SOURCE_DIR}/iot_core/lib/libiot_tls.a)

add_executable(iothub src/iot_util/cJSON.c src/main.cpp src/ros_iot.cpp)
target_link_libraries(iothub ${catkin_LIBRARIES} ${CMAKE_SOURCE_DIR}/iot_core/lib/libiot_sdk.a ${CMAKE_SOURCE_DIR}/iot_core/lib/libiot_nghttp2.a ${CMAKE_SOURCE_DIR}/iot_core/lib/libiot_hal.a ${CMAKE_SOURCE_DIR}/iot_core/lib/libiot_tls.a -lpthread -lrt)
add_dependencies(iothub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ros_demo src/ros_demo/ros_demo.cpp)
target_link_libraries(ros_demo ${catkin_LIBRARIES})
add_dependencies(ros_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})