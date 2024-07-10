if(ARGOS_BUILD_FOR_SIMULATOR)
  add_library(sim_test_controller MODULE SimTestController.h SimTestController.cpp)
  target_link_libraries(sim_test_controller
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_leo
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)
else(ARGOS_BUILD_FOR_SIMULATOR)
  add_executable(leo_test_controller LeoTestController.h LeoTestController.cpp ../plugins/robots/leo/real_robot/main.cpp)
  target_link_libraries(leo_test_controller
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_leo
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)
endif(ARGOS_BUILD_FOR_SIMULATOR)
