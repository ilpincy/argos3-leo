add_library(test_controller MODULE TestController.h TestController.cpp)
target_link_libraries(test_controller
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)