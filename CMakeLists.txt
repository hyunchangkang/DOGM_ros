cmake_minimum_required(VERSION 3.0.2)
project(dogm_ros)

# [수정] C++14 표준을 명시적으로 요구하는 방식으로 변경
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# [수정] 컴파일 최적화 옵션은 별도로 관리
add_compile_options(-O2)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  # PCL 관련 의존성 추가
  pcl_ros
  pcl_conversions
)

# PCL 패키지 직접 찾기
find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs tf2_ros tf2_geometry_msgs visualization_msgs pcl_ros pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # PCL 헤더 경로 추가
  ${PCL_INCLUDE_DIRS}
)

add_executable(dogm_node
  src/dogm_node.cpp
  src/dynamic_grid_map.cpp
  src/particle_filter.cpp
)

# PCL 라이브러리 링크 추가
target_link_libraries(dogm_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS dogm_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})