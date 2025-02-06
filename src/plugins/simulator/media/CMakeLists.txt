#
# Headers
#
# argos3/plugins/simulator/media/
set(ARGOS3_HEADERS_PLUGINS_SIMULATOR_MEDIA_LEO
  wifi_medium.h)

#
# Source files
#
set(ARGOS3_SOURCES_PLUGINS_SIMULATOR_MEDIA_LEO
  ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_MEDIA_LEO}
  wifi_medium.cpp)

#
# Create entity plugin library
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_media_leo SHARED ${ARGOS3_SOURCES_PLUGINS_SIMULATOR_MEDIA_LEO})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_media_leo argos3core_${ARGOS_BUILD_FOR})

#
# Add plugin to ARGOS_PLUGIN_PATH
#
set(ARGOS_PLUGIN_PATH "${ARGOS_PLUGIN_PATH}:${CMAKE_CURRENT_BINARY_DIR}" CACHE INTERNAL "ARGoS plugin path")

#
# Core library installation
#
install(FILES ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_MEDIA_LEO} DESTINATION include/argos3/plugins/simulator/media/)
install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_media_leo
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
