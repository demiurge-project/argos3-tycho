# ITS REAL ROBOT CONTROLLER

MESSAGE("Cmaking controllers")

if(ARGOS_BUILD_FOR_SIMULATOR)

#	Theses test libraries are full of warning, uncomment them if needed

  ADD_LIBRARY(its_controller MODULE
      its_controller.h
      its_controller.cpp
  )
  target_link_libraries(its_controller
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_epuck
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
    argos3plugin_${ARGOS_BUILD_FOR}_genericvirtualsensorsandactuators
  )

#  ADD_LIBRARY(ground_sensor_sampler MODULE
#      real_ground_sensor_sampler.h
#      real_ground_sensor_sampler.cpp
#  )
#  target_link_libraries(ground_sensor_sampler
#    argos3core_${ARGOS_BUILD_FOR}
#    argos3plugin_${ARGOS_BUILD_FOR}_epuck
#    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
#    argos3plugin_${ARGOS_BUILD_FOR}_genericvirtualsensorsandactuators
#  )

  ADD_LIBRARY(void SHARED
      void_controller.h
      void_controller.cpp
  )
  target_link_libraries(void
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_epuck
    argos3plugin_${ARGOS_BUILD_FOR}_rvr
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
    argos3plugin_${ARGOS_BUILD_FOR}_genericvirtualsensorsandactuators
  )

  ADD_LIBRARY(pathplanning SHARED
      path_planning_controller.h
      path_planning_controller.cpp
  )
  target_link_libraries(pathplanning
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_epuck
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
    argos3plugin_${ARGOS_BUILD_FOR}_genericvirtualsensorsandactuators
    void
  )

  ADD_LIBRARY(pathplanning_rvr SHARED
      path_planning_controller_rvr.h
      path_planning_controller_rvr.cpp
  )
  target_link_libraries(pathplanning_rvr
    ${catkin_LIBRARIES}
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_rvr
    argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
    argos3plugin_${ARGOS_BUILD_FOR}_genericvirtualsensorsandactuators
    void
  )
else()

  if(ARGOS_BUILD_FOR STREQUAL "epuck")
    add_executable(its_controller
      ../../plugins/robots/e-puck/real_robot/real_epuck_its_main.cpp
      its_controller.h
      its_controller.cpp)
    target_link_libraries(its_controller
      argos3core_${ARGOS_BUILD_FOR}
      argos3plugin_${ARGOS_BUILD_FOR}_epuck
      argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
      argos3plugin_${ARGOS_BUILD_FOR}_genericvirtualsensorsandactuators)

    add_executable(path_planning_controller
      ../../plugins/robots/e-puck/real_robot/real_epuck_its_main.cpp
      path_planning_controller.h
      path_planning_controller.cpp
      void_controller.h
      void_controller.cpp
	)
    target_link_libraries(path_planning_controller
      argos3core_${ARGOS_BUILD_FOR}
      argos3plugin_${ARGOS_BUILD_FOR}_epuck
      argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
      argos3plugin_${ARGOS_BUILD_FOR}_genericvirtualsensorsandactuators)


  else(ARGOS_BUILD_FOR STREQUAL "epuck")
    MESSAGE("TODO compilation for target != epuck and != simulator")
  endif(ARGOS_BUILD_FOR STREQUAL "epuck")


endif()
