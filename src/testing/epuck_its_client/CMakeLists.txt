MESSAGE('CLIEnt epuck_its_client!!!')

# EPUCK REAL ROBOT CLIENT
set(EPUCK_ITS_CLIENT_HEADERS
    epuck_its_client.h
)
set(EPUCK_ITS_CLIENT_SOURCES
    epuck_its_client.cpp
)

ADD_LIBRARY(epuck_its_client SHARED
    ${EPUCK_ITS_CLIENT_HEADERS}
    ${EPUCK_ITS_CLIENT_SOURCES}
)

add_executable(epuck_client
    epuck_client_main.cpp
    ${EPUCK_ITS_CLIENT_HEADERS}
    ${EPUCK_ITS_CLIENT_SOURCES}
)
