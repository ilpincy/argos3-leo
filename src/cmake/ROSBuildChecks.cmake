find_package(catkin COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  ar_track_alvar_msgs
  move_base
  tf2_ros
  actionlib
)
if(catkin_FOUND)
  add_definitions(-Dcatkin_FOUND)
  include_directories(${catkin_INCLUDE_DIRS})
  link_libraries(${catkin_LIBRARIES})
endif(catkin_FOUND)
