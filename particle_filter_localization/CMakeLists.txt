cmake_minimum_required(VERSION 3.0.2)
project(particle_filter_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(particle_filter_localization_node src/particle_filter_localization_node.cpp)
target_link_libraries(particle_filter_localization_node ${catkin_LIBRARIES})

add_executable(evaluation_node src/evaluation_node.cpp)
target_link_libraries(evaluation_node ${catkin_LIBRARIES})
