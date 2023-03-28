
#
# argos its client headers
#
set(ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_IRIDIA_TRACKING_SYSTEM_ARGOS_ITS_CLIENT
        argos_its_client/arena_state_struct.h

)
#
# argos its client sources
#
set(ARGOS3_SOURCES_PLUGINS_SIMULATOR_PHYSICS_ENGINES_IRIDIA_TRACKING_SYSTEM
    ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_IRIDIA_TRACKING_SYSTEM_ARGOS_ITS_CLIENT}
        argos_its_client/arena_state_struct.cpp
)
#
# Install location for the argos its client headers
#
install(
  FILES ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_IRIDIA_TRACKING_SYSTEM_ARGOS_ITS_CLIENT}
  DESTINATION include/argos3/plugins/simulator/physics_engines/iridia_tracking_system/argos_its_client)

#
# ADD_LIBRARY(client_thread
#   ${ARGOS3_SOURCES_PLUGINS_SIMULATOR_PHYSICS_ENGINES_IRIDIA_TRACKING_SYSTEM_ARGOS_ITS_CLIENT}
# )


#
# virtual sensor server headers
#
set(ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_IRIDIA_TRACKING_SYSTEM_VIRTUAL_SENSOR_SERVER
  virtual_sensor_server/virtual_sensor_server.h
  virtual_sensor_server/virtual_sensor_data.h
)
#
# virtual sensor server sources
#
set(ARGOS3_SOURCES_PLUGINS_SIMULATOR_PHYSICS_ENGINES_IRIDIA_TRACKING_SYSTEM
    ${ARGOS3_SOURCES_PLUGINS_SIMULATOR_PHYSICS_ENGINES_IRIDIA_TRACKING_SYSTEM}
    ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_IRIDIA_TRACKING_SYSTEM_VIRTUAL_SENSOR_SERVER}
    virtual_sensor_server/virtual_sensor_server.cpp
    virtual_sensor_server/virtual_sensor_data.cpp
)
#
# Install location for the virtual sensor server headers
#
install(
  FILES ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_IRIDIA_TRACKING_SYSTEM_VIRTUAL_SENSOR_SERVER}
  DESTINATION include/argos3/plugins/simulator/physics_engines/iridia_tracking_system/virtual_sensor_server)

# #
# # Create virtual_sensors plugin library
# #
# # Create library
# add_library(virtual_sensor_server
# ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_IRIDIA_TRACKING_SYSTEM_VIRTUAL_SENSOR_SERVER}
# )
#
# target_link_libraries(virtual_sensor_server
#   argos3core_simulator
# )

#
# Headers
#
# Headers of the Iridia Tracking System physics engine
set(ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_IRIDIA_TRACKING_SYSTEM
  iridia_tracking_system.h
  iridia_tracking_system_model.h
  iridia_tracking_system_footbot_model.h
  iridia_tracking_system_epuck_model.h
  iridia_tracking_system_rvr_model.h
  iridia_tracking_system_box_model.h
  iridia_tracking_system_cylinder_model.h
  iridia_tracking_system_rectangular_model.h
  tycho_status_thread.h
)

#
# Source files
#
# Tracking Iridia Tracking System source files
set(ARGOS3_SOURCES_PLUGINS_SIMULATOR_PHYSICS_ENGINES_IRIDIA_TRACKING_SYSTEM
  ${ARGOS3_SOURCES_PLUGINS_SIMULATOR_PHYSICS_ENGINES_IRIDIA_TRACKING_SYSTEM}
  ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_IRIDIA_TRACKING_SYSTEM}
  iridia_tracking_system.cpp
  iridia_tracking_system_footbot_model.cpp
  iridia_tracking_system_epuck_model.cpp
  iridia_tracking_system_rvr_model.cpp
  iridia_tracking_system_box_model.cpp
  iridia_tracking_system_cylinder_model.cpp
  tycho_status_thread.cpp
)


#
# Create iridia_tracking_system engine plugin library
#
# Create library
add_library(argos3plugin_simulator_iridia_tracking_system SHARED
  ${ARGOS3_SOURCES_PLUGINS_SIMULATOR_PHYSICS_ENGINES_IRIDIA_TRACKING_SYSTEM}
  ${LUA_LIBRARIES}
)

target_link_libraries(argos3plugin_simulator_iridia_tracking_system
    ${catkin_LIBRARIES}
    ${QT_LIBRARIES}
    argos3core_simulator
    argos3plugin_simulator_genericvirtualsensorsandactuators
)

install(FILES
  ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_IRIDIA_TRACKING_SYSTEM}
  DESTINATION
  include/argos3/plugins/simulator/physics_engines/iridia_tracking_system)
install(TARGETS argos3plugin_simulator_iridia_tracking_system
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
