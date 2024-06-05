if(ARGOS_BUILD_FOR_SIMULATOR)
  add_library(test_controller MODULE TestController.h TestController.cpp)
else(ARGOS_BUILD_FOR_SIMULATOR)
  add_executable(test_controller TestController.h TestController.cpp ../plugins/robots/leo/real_robot/main.cpp)
endif(ARGOS_BUILD_FOR_SIMULATOR)
target_link_libraries(test_controller
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_leo
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)
