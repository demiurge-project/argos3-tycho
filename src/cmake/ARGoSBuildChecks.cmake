#
# Find the ARGoS package
#
if(ARGOS_BUILD_FOR_SIMULATOR)
  find_package(PkgConfig)
  pkg_check_modules(ARGOS REQUIRED argos3_simulator)
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)
  #include_directories(${ARGOS_INCLUDE_DIRS})
  include_directories(${ARGOS_INCLUDE_DIRS}
   /home/ken/depots/argos3-epuck/src)
  link_directories(${ARGOS_LIBRARY_DIRS})
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Check for Lua 5.2
#
find_package(Lua52)
if(LUA52_FOUND)
  include_directories(${LUA_INCLUDE_DIR})
endif(LUA52_FOUND)

#
# Check for PThreads
#
find_package(Pthreads)
if(NOT PTHREADS_FOUND)
  message(FATAL_ERROR "Required library pthreads not found.")
endif(NOT PTHREADS_FOUND)
add_definitions(${PTHREADS_DEFINITIONS})
include_directories(${PTHREADS_INCLUDE_DIR})
