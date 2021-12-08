
#include <stdlib.h>

#include "epuck_its_client.h"

int main(int argc, char *argv[])
{
    std::string strArgosServerHost = "169.254.100.2";
    uint32_t unArgosServerPort = 4050;



    argos::CEpuckITSClient cEpuckITSClient(strArgosServerHost, unArgosServerPort);

    //pthread_t cThread;

    //pthread_create(&cThread, NULL, &argos::CEpuckITSClient::start, m_cClient);

    cEpuckITSClient.Run();
}
