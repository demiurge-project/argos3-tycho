link_directories(${CMAKE_BINARY_DIR}/testing/controllers)
set(simplified_pathplanning_loop_functions_SOURCES
	simplified_pathplanning_loop_functions.cpp
	simplified_pathplanning_loop_functions.h
	position_generator.h
	assignment_generator.h
	min_max_assignment_generator.h
	min_max_assignment_generator.cpp
	path_generator.h
	dijkstra_path_generator.h
	dijkstra_path_generator.cpp
	hungarian/hungarian.cpp
	hungarian/hungarian.hpp
)

if(ARGOS_COMPILE_QTOPENGL)
	MESSAGE("compile with OPENGL")
  set(simplified_pathplanning_loop_functions_SOURCES
    ${simplified_pathplanning_loop_functions_SOURCES}
    pathplanning_qt_user_functions.cpp
    pathplanning_qt_user_functions.h)
endif(ARGOS_COMPILE_QTOPENGL)


MESSAGE("#####################################################################################")
add_library(simplified_pathplanning_loop_functions MODULE ${simplified_pathplanning_loop_functions_SOURCES})
target_link_libraries(simplified_pathplanning_loop_functions
  void
  argos3core_simulator
#  argos3plugin_simulator_actuators
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot
#  argos3plugin_simulator_media
#  argos3plugin_simulator_sensors
  argos3plugin_simulator_genericvirtualsensorsandactuators)

#if(ARGOS_COMPILE_QTOPENGL)
target_link_libraries(simplified_pathplanning_loop_functions argos3plugin_simulator_qtopengl)
#endif(ARGOS_COMPILE_QTOPENGL)

link_directories(${CMAKE_BINARY_DIR}/testing/controllers)
set(simplified_pathplanning_rvr_loop_functions_SOURCES
	simplified_pathplanning_rvr_loop_functions.cpp
	simplified_pathplanning_rvr_loop_functions.h
	position_generator.h
	assignment_generator.h
	min_max_assignment_generator.h
	min_max_assignment_generator.cpp
	path_generator.h
	dijkstra_path_generator.h
	dijkstra_path_generator.cpp
	hungarian/hungarian.cpp
	hungarian/hungarian.hpp
)

if(ARGOS_COMPILE_QTOPENGL)
	MESSAGE("compile with OPENGL")
  set(simplified_pathplanning_rvr_loop_functions_SOURCES
    ${simplified_pathplanning_rvr_loop_functions_SOURCES}
    pathplanning_rvr_qt_user_functions.cpp
    pathplanning_rvr_qt_user_functions.h)
endif(ARGOS_COMPILE_QTOPENGL)

MESSAGE("#####################################################################################")
add_library(simplified_pathplanning_rvr_loop_functions MODULE ${simplified_pathplanning_rvr_loop_functions_SOURCES})
target_link_libraries(simplified_pathplanning_rvr_loop_functions
  void
  argos3core_simulator
#  argos3plugin_simulator_actuators
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_rvr
  argos3plugin_simulator_genericrobot
#  argos3plugin_simulator_media
#  argos3plugin_simulator_sensors
  argos3plugin_simulator_genericvirtualsensorsandactuators)

#if(ARGOS_COMPILE_QTOPENGL)
target_link_libraries(simplified_pathplanning_rvr_loop_functions argos3plugin_simulator_qtopengl)
#endif(ARGOS_COMPILE_QTOPENGL)
