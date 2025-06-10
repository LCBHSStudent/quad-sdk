cmake_minimum_required(VERSION 3.8)
project(quad_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS1
find_package(catkin REQUIRED
  roscpp
  topic_tools
  roslib
  quad_utils
  sensor_msgs
  geometry_msgs
  nav_msgs
  gazebo_msgs
  grid_map_core
  grid_map_ros
  grid_map_pcl
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    quad_bridge
  CATKIN_DEPENDS
    roscpp topic_tools roslib quad_utils sensor_msgs geometry_msgs
    nav_msgs gazebo_msgs grid_map_core grid_map_ros grid_map_pcl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE SOURCE_FILES "src/*.cc")

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES})

# Unitree SDK2
target_include_directories(${PROJECT_NAME} PRIVATE
  third_party/unitree_sdk2/include
  third_party/unitree_sdk2/thirdparty/include
  third_party/unitree_sdk2/thirdparty/include/ddscxx
)
target_link_directories(${PROJECT_NAME} PRIVATE
  third_party/unitree_sdk2/lib/x86_64
  third_party/unitree_sdk2/thirdparty/lib/x86_64
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  unitree_sdk2
  ddsc
  ddscxx
  pthread
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
