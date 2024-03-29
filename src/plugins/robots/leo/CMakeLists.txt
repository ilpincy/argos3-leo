#
# leo headers
#
# argos3/plugins/robots/leo/control_interface
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_CONTROLINTERFACE
  control_interface/ci_leo_posetwist_actuator.h
#   control_interface/ci_leo_camera_sensor.h
#   control_interface/ci_leo_encoder_sensor.h
#   control_interface/ci_leo_ground_sensor.h
#   control_interface/ci_leo_lidar_sensor.h
#   control_interface/ci_leo_light_sensor.h
#   control_interface/ci_leo_proximity_sensor.h
#   control_interface/ci_leo_ultrasound_sensor.h
)
# if(BUZZ_FOUND)
#   set(ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_CONTROLINTERFACE
#     ${ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_CONTROLINTERFACE}
#     control_interface/buzz_controller_leo.h)
# endif(BUZZ_FOUND)
# argos3/plugins/robots/leo/simulator
if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_SIMULATOR
    simulator/dynamics2d_leo_model.h
    simulator/leo_entity.h
    simulator/leo_measures.h
    simulator/leo_posetwist_default_actuator.h)
endif(ARGOS_BUILD_FOR_SIMULATOR)
# argos3/plugins/robots/leo/real_robot
# if(ARGOS_BUILD_FOR STREQUAL "leo")
#   set(ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_REALROBOT
#     real_robot/real_leo.h
#     real_robot/real_leo_device.h
#     real_robot/real_leo_differential_steering_device.h
#     real_robot/real_leo_differential_steering_actuator.h
#     real_robot/real_leo_gripper_actuator.h
#     real_robot/real_leo_leds_actuator.h
#     real_robot/real_leo_battery_sensor.h
#     real_robot/real_leo_camera_sensor.h
#     real_robot/real_leo_encoder_sensor.h
#     real_robot/real_leo_ground_sensor.h
#     real_robot/real_leo_lidar_sensor.h
#     real_robot/real_leo_proximity_sensor.h
#     real_robot/real_leo_ultrasound_sensor.h)
# endif(ARGOS_BUILD_FOR STREQUAL "leo")

#
# leo sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_CONTROLINTERFACE}
  control_interface/ci_leo_posetwist_actuator.cpp
#   control_interface/ci_leo_camera_sensor.cpp
#   control_interface/ci_leo_encoder_sensor.cpp
#   control_interface/ci_leo_ground_sensor.cpp
#   control_interface/ci_leo_lidar_sensor.cpp
#   control_interface/ci_leo_light_sensor.cpp
#   control_interface/ci_leo_proximity_sensor.cpp
#   control_interface/ci_leo_ultrasound_sensor.cpp
)
# if(BUZZ_FOUND)
#   set(ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO
#     ${ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO}
#     control_interface/buzz_controller_leo.cpp)
# endif(BUZZ_FOUND)

if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_SIMULATOR}
    simulator/dynamics2d_leo_model.cpp
    simulator/leo_entity.cpp
    simulator/leo_measures.cpp
    simulator/leo_posetwist_default_actuator.cpp)
  # Compile the graphical visualization only if the necessary libraries have been found
  if(ARGOS_QTOPENGL_FOUND)
    include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
    set(ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_SIMULATOR
      ${ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_SIMULATOR}
      simulator/qtopengl_leo.h)
    set(ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO
      ${ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO}
      simulator/qtopengl_leo.h
      simulator/qtopengl_leo.cpp)
  endif(ARGOS_QTOPENGL_FOUND)
endif(ARGOS_BUILD_FOR_SIMULATOR)
# if(ARGOS_BUILD_FOR STREQUAL "leo")
#   set(ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO
#     ${ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO}
#     ${ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_REALROBOT}
#     real_robot/real_leo.cpp
#     real_robot/real_leo_device.cpp
#     real_robot/real_leo_differential_steering_device.cpp
#     real_robot/real_leo_differential_steering_actuator.cpp
#     real_robot/real_leo_gripper_actuator.cpp
#     real_robot/real_leo_leds_actuator.cpp
#     real_robot/real_leo_battery_sensor.cpp
#     real_robot/real_leo_camera_sensor.cpp
#     real_robot/real_leo_encoder_sensor.cpp
#     real_robot/real_leo_ground_sensor.cpp
#     real_robot/real_leo_lidar_sensor.cpp
#     real_robot/real_leo_proximity_sensor.cpp
#     real_robot/real_leo_ultrasound_sensor.cpp)
# endif(ARGOS_BUILD_FOR STREQUAL "leo")

#
# Create leo plugin
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_leo SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_leo
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)
if(ARGOS_BUILD_FOR_SIMULATOR)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_leo
    argos3plugin_${ARGOS_BUILD_FOR}_dynamics2d)
  if(ARGOS_QTOPENGL_FOUND)
    target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_leo argos3plugin_${ARGOS_BUILD_FOR}_qtopengl)
  endif(ARGOS_QTOPENGL_FOUND)
else(ARGOS_BUILD_FOR_SIMULATOR)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_leo -lleo)
endif(ARGOS_BUILD_FOR_SIMULATOR)
if(BUZZ_FOUND)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_leo argos3plugin_${ARGOS_BUILD_FOR}_buzz ${BUZZ_LIBRARY})
endif(BUZZ_FOUND)

#
# Create battery monitor
#
# if(ARGOS_BUILD_FOR STREQUAL "leo")
#   add_executable(battery real_robot/battery.c)
#   target_link_libraries(battery -lleo)
# endif(ARGOS_BUILD_FOR STREQUAL "leo")

#
# Installation
#
install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_CONTROLINTERFACE} DESTINATION include/argos3/plugins/robots/leo/control_interface)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_SIMULATOR} DESTINATION include/argos3/plugins/robots/leo/simulator)
  if(ARGOS_QTOPENGL_FOUND)
    install(DIRECTORY simulator/models DESTINATION include/argos3/plugins/simulator/visualizations/qt-opengl)
  endif(ARGOS_QTOPENGL_FOUND)
else(ARGOS_BUILD_FOR_SIMULATOR)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_REALROBOT} DESTINATION include/argos3/plugins/robots/leo/real_robot)
  install(FILES real_robot/main.cpp DESTINATION include/argos3/plugins/robots/leo/real_robot)
endif(ARGOS_BUILD_FOR_SIMULATOR)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_leo
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
