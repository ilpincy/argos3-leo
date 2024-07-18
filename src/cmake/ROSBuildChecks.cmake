find_package(catkin COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
)
if(catkin_FOUND)
  add_definitions(-Dcatkin_FOUND)
  include_directories(${catkin_INCLUDE_DIRS})
  link_libraries(${catkin_LIBRARIES})
endif(catkin_FOUND)
