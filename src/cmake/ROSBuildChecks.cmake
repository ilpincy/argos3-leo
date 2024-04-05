find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
link_libraries(
  ${catkin_LIBRARIES}
)