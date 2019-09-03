if(ARGOS_BUILD_FOR_SIMULATOR)
  include_directories(${CMAKE_SOURCE_DIR}/plugins/robots/kilobot/control_interface)
  
  add_executable(aggregation aggregation.c)
  target_link_libraries(aggregation argos3plugin_simulator_kilolib)
  
  add_executable(beacon_blue beacon_blue.c)
  target_link_libraries(beacon_blue argos3plugin_simulator_kilolib)
  
  add_executable(beacon_red beacon_red.c)
  target_link_libraries(beacon_red argos3plugin_simulator_kilolib)
  
  add_executable(explorer explorer.c)
  target_link_libraries(explorer argos3plugin_simulator_kilolib)

endif(ARGOS_BUILD_FOR_SIMULATOR)
