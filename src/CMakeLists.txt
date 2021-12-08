cmake_minimum_required(VERSION 2.8)
project(argos3-iridia-tracking-system)

set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)

# Set the path where CMake will find additional scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Include path points to the base source dir
include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR})

# Find ROS packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  LIBRARIES template_controller
  CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs nav_msgs tf
  DEPENDS system_lib
)

include_directories(
		${catkin_INCLUDE_DIRS}
)

link_libraries(${catkin_LIBRARIES})

if(ARGOS_BUILD_FOR STREQUAL "simulator")
	set (ARGOS_BUILD_FOR_SIMULATOR)

	# Find the ARGoS package
	find_package(PkgConfig)
	pkg_check_modules(ARGOS REQUIRED argos3_simulator)
	set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)
	message(STATUS "CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")

	# Check whether all the necessary libs have been installed to compile the
	# code that depends on Qt and OpenGL
	include(ARGoSCheckQTOpenGL)

	# Find the GSL package
	find_package(GSL)

	# Find Lua
	find_package(Lua52)
else()
	set (ARGOS_BUILD_FOR_EPUCK)
endif(ARGOS_BUILD_FOR STREQUAL "simulator")

# Make sure we are under Unix
if(NOT UNIX)
  message(FATAL_ERROR "This plugin compiles only under UNIX, sorry!")
endif(NOT UNIX)

# Set build options
include(${CMAKE_SOURCE_DIR}/cmake/ARGoSBuildOptions.cmake)

# Set compiler flags
include(${CMAKE_SOURCE_DIR}/cmake/ARGoSBuildFlags.cmake)

# Check for libraries
include(${CMAKE_SOURCE_DIR}/cmake/ARGoSBuildChecks.cmake)


# Set up CPack for later use
#include(${CMAKE_SOURCE_DIR}/cmake/ARGoSPackaging.cmake)

# Set up ARGoS compilation information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})

add_subdirectory(plugins)
add_subdirectory(testing)
