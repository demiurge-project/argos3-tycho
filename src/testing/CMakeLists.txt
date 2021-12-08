MESSAGE("Cmaking testing")

add_subdirectory(controllers)

if(ARGOS_BUILD_FOR_SIMULATOR)
	add_subdirectory(loop_functions)
endif(ARGOS_BUILD_FOR_SIMULATOR)
