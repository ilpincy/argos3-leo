find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  ar_track_alvar_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
link_libraries(
  ${catkin_LIBRARIES}
)