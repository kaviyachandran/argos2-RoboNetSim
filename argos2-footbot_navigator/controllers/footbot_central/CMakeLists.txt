include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-fPIC)
add_library(footbot_central_controller MODULE
  footbot_central.cpp
)


target_link_libraries(footbot_central_controller
  lcm
  navigation
  #argos2_simulator_dynamic_linking
  )
