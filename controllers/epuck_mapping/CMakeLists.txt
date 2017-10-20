add_library(epuck_mapping MODULE epuck_mapping.h epuck_mapping.cpp)
target_link_libraries(epuck_mapping
  argos3core_simulator
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot)
