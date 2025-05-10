cmake_minimum_required(VERSION 3.0.2)
project(particle_filter_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(particle_filter_localization_node src/particle_filter_localization_node.cpp)

target_link_libraries(particle_filter_localization_node
  ${catkin_LIBRARIES}
)
