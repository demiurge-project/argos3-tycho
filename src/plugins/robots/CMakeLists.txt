MESSAGE("Cmaking robots")


#
# generic virtual sensors and actuators control interface headers
#
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_GENERIC_CONTROLINTERFACE
  generic/control_interface/virtual_sensor_register.h
  generic/control_interface/ci_gps_virtual_sensor.h
  generic/control_interface/ci_destination_virtual_sensor.h
  generic/control_interface/ci_virtual_radiation_sensor.h
  generic/control_interface/ci_virtual_rgb_ground_sensor.h
  generic/control_interface/ci_virtual_pollutant_sensor.h
  generic/control_interface/ci_virtual_black_white_location_sensor.h
  generic/control_interface/virtual_sensor_network_data.h
  generic/control_interface/virtual_sensor_data_position2d.h
  )


#
# generic virtual sensors and actuators control interface sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_GENERIC
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_GENERIC_CONTROLINTERFACE}
  generic/control_interface/virtual_sensor_register.cpp
  generic/control_interface/ci_gps_virtual_sensor.cpp
  generic/control_interface/ci_destination_virtual_sensor.cpp
  generic/control_interface/ci_virtual_rgb_ground_sensor.cpp
  generic/control_interface/ci_virtual_radiation_sensor.cpp
  generic/control_interface/ci_virtual_pollutant_sensor.cpp
  generic/control_interface/ci_virtual_black_white_location_sensor.cpp
  )

#
# Install location for the generic virtual sensors and actuators control interface headers
#
install(
  FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_GENERIC_CONTROLINTERFACE}
  DESTINATION include/argos3/plugins/robots/generic/control_interface)


if(ARGOS_BUILD_FOR_SIMULATOR)
  #
  # generic virtual sensors and actuators simulator code
  #

  #
  # Virtual sensors library
  #
  # Headers
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_GENERIC_SIMULATOR
    generic/simulator/generic_virtual_sensor.h
    generic/simulator/ground_virtual_sensor.h
    generic/simulator/light_virtual_sensor.h
    generic/simulator/gps_virtual_sensor.h
    generic/simulator/destination_virtual_sensor.h
    generic/simulator/virtual_rgb_ground_sensor.h
    generic/simulator/virtual_radiation_sensor.h
    generic/simulator/virtual_pollutant_sensor.h
    generic/simulator/virtual_black_white_location_sensor.h)
  # Sources
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_GENERIC
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_GENERIC}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_GENERIC_SIMULATOR}
    generic/simulator/generic_virtual_sensor.cpp
    generic/simulator/ground_virtual_sensor.cpp
    generic/simulator/light_virtual_sensor.cpp
    generic/simulator/gps_virtual_sensor.cpp
    generic/simulator/destination_virtual_sensor.cpp
    generic/simulator/virtual_radiation_sensor.cpp
    generic/simulator/virtual_rgb_ground_sensor.cpp
    generic/simulator/virtual_pollutant_sensor.cpp
    generic/simulator/virtual_black_white_location_sensor.cpp)

  # Install location for the generic virtual sensors and actuators simulator headers
  install(
    FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_GENERIC_SIMULATOR}
    DESTINATION include/argos3/plugins/robots/generic/simulator)



  #
  # e-puck simulator code
  #
  # Headers
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCK_SIMULATOR
    #e-puck/simulator/epuck_ground_virtual_sensor.h # TODO whats this?
)
  # Sources
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_EPUCK
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_EPUCK}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCK_SIMULATOR}
    #e-puck/simulator/epuck_ground_virtual_sensor.cpp # TODO whats this?
)
  # Install location for the e-puck simulator headers
  install(
    FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCK_SIMULATOR}
    DESTINATION include/argos3/plugins/robots/e-puck/simulator)

else(ARGOS_BUILD_FOR_SIMULATOR)
  #
  # generic virtual sensors and actuators real robot code
  #
  # Headers
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_GENERIC_REALROBOT
     #Base
    generic/real_robot/virtual_sensor_client.h
     #Sensors
    generic/real_robot/real_virtual_sensor.h
    generic/real_robot/real_virtual_light_sensor.h
    generic/real_robot/real_virtual_ground_sensor.h
    generic/real_robot/real_gps_virtual_sensor.h
    generic/real_robot/real_destination_virtual_sensor.h
    generic/real_robot/real_virtual_radiation_sensor.h
    generic/real_robot/real_virtual_rgb_ground_sensor.h
    generic/real_robot/real_virtual_pollutant_sensor.h
    generic/real_robot/real_virtual_black_white_location_sensor.h)

  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_GENERIC
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_GENERIC}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_GENERIC_REALROBOT}
     #Base
    generic/real_robot/virtual_sensor_client.cpp
     #Sensors
    generic/real_robot/real_virtual_sensor.cpp
    generic/real_robot/real_virtual_light_sensor.cpp
    generic/real_robot/real_virtual_ground_sensor.cpp
    generic/real_robot/real_gps_virtual_sensor.cpp
    generic/real_robot/real_destination_virtual_sensor.cpp
    generic/real_robot/real_virtual_radiation_sensor.cpp
    generic/real_robot/real_virtual_rgb_ground_sensor.cpp
    generic/real_robot/real_virtual_pollutant_sensor.cpp
    generic/real_robot/real_virtual_black_white_location_sensor.cpp)

   #Install location for the real virtual sensors and actuators headers
  install(
    FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_GENERIC_REALROBOT}
    DESTINATION include/argos3/plugins/robots/generic/real_robot)


  if(ARGOS_BUILD_FOR STREQUAL "epuck")
    #
    # e-puck real robot code
    #
    # Headers
    set(ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCK_REALROBOT
        # Base
        e-puck/real_robot/real_epuck_its.h
        # Sensors
        # Actuators
    )
    set(ARGOS3_SOURCES_PLUGINS_ROBOTS_EPUCK
        ${ARGOS3_SOURCES_PLUGINS_ROBOTS_EPUCK}
        ${ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCK_REALROBOT}
        # Base
        e-puck/real_robot/real_epuck_its.cpp
        # Sensors
        # Actuators
    )
    # Install location for the real e-puck headers
    install(
        FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCK_REALROBOT}
        DESTINATION include/argos3/plugins/robots/e-puck/real_robot)
  endif(ARGOS_BUILD_FOR STREQUAL "epuck")
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Create generic virtual sensors and actuators plugin
#
# Create target
add_library(argos3plugin_${ARGOS_BUILD_FOR}_genericvirtualsensorsandactuators SHARED
  ${ARGOS3_SOURCES_PLUGINS_ROBOTS_GENERIC}
  ${ARGOS3_SOURCES_PLUGINS_ROBOTS_EPUCK}
)
# Link libraries
if(ARGOS_BUILD_FOR_SIMULATOR)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_genericvirtualsensorsandactuators
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_epuck
    argos3plugin_${ARGOS_BUILD_FOR}_footbot
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
    argos3plugin_${ARGOS_BUILD_FOR}_media
    argos3plugin_${ARGOS_BUILD_FOR}_entities
    argos3plugin_${ARGOS_BUILD_FOR}_pointmass3d
    argos3plugin_${ARGOS_BUILD_FOR}_dynamics2d
    argos3plugin_${ARGOS_BUILD_FOR}_qtopengl
)
else(ARGOS_BUILD_FOR_SIMULATOR)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_genericvirtualsensorsandactuators
    ${PTHREADS_LIBRARY}
    argos3core_${ARGOS_BUILD_FOR})

  if(ARGOS_BUILD_FOR STREQUAL "epuck")
    target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_genericvirtualsensorsandactuators
      argos3plugin_${ARGOS_BUILD_FOR}_epuck)
  endif(ARGOS_BUILD_FOR STREQUAL "epuck")
endif(ARGOS_BUILD_FOR_SIMULATOR)

# Install location for the generic virtual sensors and actuators plugin
install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_genericvirtualsensorsandactuators
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
