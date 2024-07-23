if(ARGOS_BUILD_FOR_SIMULATOR)
  add_library(sim_test_controller MODULE sim_test_controller.h sim_test_controller.cpp)
  target_link_libraries(sim_test_controller
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_leo
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)
else(ARGOS_BUILD_FOR_SIMULATOR)
  add_executable(leo_test_communication leo_test_communication.h leo_test_communication.cpp main.cpp)
  target_link_libraries(leo_test_communication
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_leo
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)

  add_executable(leo_test_controller leo_test_controller.h leo_test_controller.cpp main.cpp)
  target_link_libraries(leo_test_controller
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_leo
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)
endif(ARGOS_BUILD_FOR_SIMULATOR)
