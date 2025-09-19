cmake_minimum_required(VERSION 3.0.2)
project(dogm_ros)

add_compile_options(-std=c++14 -O2)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs tf2_ros tf2_geometry_msgs visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dogm_node
  src/dogm_node.cpp
  src/dynamic_grid_map.cpp
  src/particle_filter.cpp
)

target_link_libraries(dogm_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/fake_scan.py
  scripts/static_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS dogm_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
