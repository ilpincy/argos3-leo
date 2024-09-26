
# Simulator

set(ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_SIMULATOR
  simulator/dynamics2d_leo_model.h
  simulator/leo_entity.h
  simulator/leo_measures.h
  simulator/leo_posetwist_default_actuator.h
  simulator/leo_odometry_sensor.h
  simulator/leo_ar_tag_sensor.h
  simulator/leo_wifi_default_actuator.h
  simulator/leo_wifi_medium_sensor.h
  )

set(ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO_SIMULATOR
  simulator/dynamics2d_leo_model.cpp
  simulator/leo_entity.cpp
  simulator/leo_measures.cpp
  simulator/leo_posetwist_default_actuator.cpp
  simulator/leo_odometry_sensor.cpp
  simulator/leo_ar_tag_sensor.cpp
  simulator/leo_wifi_default_actuator.cpp
  simulator/leo_wifi_medium_sensor.cpp
  )

# Real Robot

set(ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_REALROBOT
  real_robot/real_leo.h
  real_robot/real_leo_device.h
  real_robot/real_leo_posetwist_actuator.h
  real_robot/real_leo_odometry_sensor.h
  real_robot/real_leo_ar_tag_sensor.h
  real_robot/real_leo_wifi_actuator.h
  real_robot/real_leo_wifi_sensor.h
  )

set(ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO_REALROBOT
  real_robot/real_leo.cpp
  real_robot/real_leo_device.cpp
  real_robot/real_leo_posetwist_actuator.cpp
  real_robot/real_leo_odometry_sensor.cpp
  real_robot/real_leo_ar_tag_sensor.cpp
  real_robot/real_leo_wifi_actuator.cpp
  real_robot/real_leo_wifi_sensor.cpp
  )

# Control Interfaces

set(ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_CONTROLINTERFACE
  control_interface/ci_leo_posetwist_actuator.h
  control_interface/ci_leo_odometry_sensor.h
  control_interface/ci_leo_ar_tag_sensor.h
  control_interface/ci_leo_wifi_actuator.h
  control_interface/ci_leo_wifi_sensor.h
  )

set(ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO_CONTROLINTERFACE
  control_interface/ci_leo_posetwist_actuator.cpp
  control_interface/ci_leo_odometry_sensor.cpp
  control_interface/ci_leo_ar_tag_sensor.cpp
  control_interface/ci_leo_wifi_actuator.cpp
  control_interface/ci_leo_wifi_sensor.cpp
  )

set(ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_CONTROLINTERFACE}
  )

set(ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO
  ${ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO_CONTROLINTERFACE}
  )

if(ARGOS_BUILD_FOR_SIMULATOR)

  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_SIMULATOR}
    )
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO}
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO_SIMULATOR}
    )

  # Compile the graphical visualization only if the necessary libraries have been found
  if(ARGOS_QTOPENGL_FOUND)
    include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
    set(ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO
      ${ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO}
      simulator/qtopengl_leo.h
      )
    set(ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO
      ${ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO}
      simulator/qtopengl_leo.cpp
      )
  endif(ARGOS_QTOPENGL_FOUND)

endif(ARGOS_BUILD_FOR_SIMULATOR)

if(ARGOS_BUILD_FOR STREQUAL "leo")
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_REALROBOT}
    )
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO}
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO_REALROBOT}
    )
endif(ARGOS_BUILD_FOR STREQUAL "leo")

set(ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO
  ${ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO}
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO}
)

#
# Create leo plugin
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_leo SHARED
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO}
  ${ARGOS3_SOURCES_PLUGINS_ROBOTS_LEO}
  )

target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_leo
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
  argos3plugin_${ARGOS_BUILD_FOR}_entities_leo
  argos3plugin_${ARGOS_BUILD_FOR}_media_leo
  )

if(ARGOS_BUILD_FOR_SIMULATOR)
  target_link_libraries(argos3plugin_simulator_leo
    argos3plugin_simulator_dynamics2d
    )
  if(ARGOS_QTOPENGL_FOUND)
    target_link_libraries(argos3plugin_simulator_leo
      argos3plugin_simulator_qtopengl
      )
  endif(ARGOS_QTOPENGL_FOUND)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Installation
#
install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_CONTROLINTERFACE} DESTINATION 
  include/argos3/plugins/robots/leo/control_interface
  )

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_SIMULATOR} DESTINATION
    include/argos3/plugins/robots/leo/simulator
    )
  if(ARGOS_QTOPENGL_FOUND)

    install(DIRECTORY simulator/models DESTINATION 
      include/argos3/plugins/simulator/visualizations/qt-opengl
      )

  endif(ARGOS_QTOPENGL_FOUND)

else(ARGOS_BUILD_FOR_SIMULATOR)

  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_LEO_REALROBOT} DESTINATION
    include/argos3/plugins/robots/leo/real_robot
    )

  install(FILES real_robot/main.cpp DESTINATION 
    include/argos3/plugins/robots/leo/real_robot
    )

endif(ARGOS_BUILD_FOR_SIMULATOR)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_leo
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3
  )
