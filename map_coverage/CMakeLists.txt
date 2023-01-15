cmake_minimum_required(VERSION 2.8.3)
project(map_coverage)

add_compile_options(-std=c++14)

find_package(OpenCV REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  
  image_transport
  cv_bridge
  roscpp  
  nav_msgs
  image_geometry
  move_base
  move_base_msgs
  actionlib
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
  std_msgs
  roslib
  geometry_msgs
)


catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp std_msgs  roslib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(map_coverage_node 
    src/map_coverage_ros_node.cpp
    src/MoveBaseController.cpp

)

add_dependencies(map_coverage_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(map_coverage_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
