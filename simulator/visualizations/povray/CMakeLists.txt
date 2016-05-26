set(argos2_simulator_povray_renderer_HEADERS
  povray_render.h
  povray_render_visitor_draw.h
)

set(argos2_simulator_povray_renderer_IMPLEMENTATIONS
  povray_render.cpp
  povray_render_visitor_draw.cpp
)

add_library(argos2_simulator_povray_renderer SHARED ${argos2_simulator_povray_renderer_IMPLEMENTATIONS})

install(FILES ${argos2_simulator_povray_renderer_HEADERS} DESTINATION include/argos2/simulator/visualizations/povray)
install(TARGETS argos2_simulator_povray_renderer
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos2
  ARCHIVE DESTINATION lib/argos2)
