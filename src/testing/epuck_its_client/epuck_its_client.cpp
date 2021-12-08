#include "epuck_its_client.h"
#include "stdio.h"
#include <cstring>

namespace argos {

/****************************************/
/****************************************/

CEpuckITSClient::CEpuckITSClient(std::string str_argos_server_address, uint32_t un_argos_server_port)
    :
      m_strArgosServerAddress(str_argos_server_address),
      m_unArgosServerPort(un_argos_server_port)
{
    m_tVirtualSensorTable = new std::map<uint8_t, uint32_t>();
    m_tVirtualSensorData = new std::map<uint8_t, SVirtualSensorData>();
}

/****************************************/
/****************************************/

void CEpuckITSClient::Run()
{

    InitClient();

    ReceiveVirtualSensorTable();

    PrintVirtualSensorTable();

    //SynchronizeClient();

    while(true) {
        //InitExperiment();
        ExecuteExperiment();
        //EndExperiment();
        //ResetExperiment();
    }
}

/****************************************/
/****************************************/

void CEpuckITSClient::InitClient()
{
    /*
     * Use socket
     * No Qt allowed
     *
     */

    m_nSocketDescriptor = socket(AF_INET , SOCK_STREAM , 0);

    if (m_nSocketDescriptor == -1)
    {
        printf("Could not create socket\n");
        exit(0);
    }


    m_sServer.sin_addr.s_addr = inet_addr(m_strArgosServerAddress.c_str());

    m_sServer.sin_family = AF_INET;
    m_sServer.sin_port = htons( m_unArgosServerPort );

    //Connect to remote server
    if (connect(m_nSocketDescriptor , (struct sockaddr *)&m_sServer , sizeof(m_sServer)) < 0)
    {
        printf("connect error\n");
        exit(0);
    }

    printf("Connected\n");
}

/****************************************/
/****************************************/

void CEpuckITSClient::ReceiveVirtualSensorTable()
{
    uint32_t unVirtualSensorTableBufferSize;
    if( recv(m_nSocketDescriptor, &unVirtualSensorTableBufferSize , sizeof(uint32_t) , 0) < 0)
    {
        printf("recv failed\n");
        exit(0);
    }
    unVirtualSensorTableBufferSize = ntohl(unVirtualSensorTableBufferSize);
    printf("Received virtual sensor table size: %d\n",  unVirtualSensorTableBufferSize);


    char byteVirtualSensorTableBuffer[unVirtualSensorTableBufferSize];
    uint32_t totalByteRead = 0;
    uint32_t n = 0;

    printf("Waiting for virtual sensor table\n");
    while (totalByteRead < unVirtualSensorTableBufferSize) {
        if(( n = recv(m_nSocketDescriptor, byteVirtualSensorTableBuffer , unVirtualSensorTableBufferSize - totalByteRead , 0)) < 0)
        {
            printf("recv failed\n");
            exit(0);
        }
        printf("Bytes read: %d / %d:\n", n, unVirtualSensorTableBufferSize);
        totalByteRead = totalByteRead + n;
    }

    uint32_t unBufferIndex = 0;
    uint8_t unSensorId;
    uint32_t unSensorDataSize;

    while (unBufferIndex < unVirtualSensorTableBufferSize) {

        unSensorId = byteVirtualSensorTableBuffer[unBufferIndex];
        unBufferIndex++;

        //char strSensorSingleValue[4];
        //for (uint32_t unByteIndex = 0; unByteIndex < 4; unByteIndex++) {
        //    strSensorSingleValue[unByteIndex] = byteVirtualSensorTableBuffer[unBufferIndex];
        //    unBufferIndex++;
            //printf("Bytes unserialized: %d / %d\n", unBufferIndex, unSensorDataSize);
        //}
        memcpy(&unSensorDataSize,byteVirtualSensorTableBuffer + unBufferIndex, sizeof(uint32_t));
        unBufferIndex = unBufferIndex + sizeof(uint32_t);

        printf("Sensor Entry: %d - %d\n", unSensorId, unSensorDataSize);
        m_tVirtualSensorTable->insert(std::make_pair<uint8_t, uint32_t>(unSensorId, unSensorDataSize));
    }
}

/****************************************/
/****************************************/

void CEpuckITSClient::ExecuteExperiment()
{
    uint32_t unLoopCounter = 0;

    while(true) {

        unLoopCounter++;

        printf("Waiting for data size. Loop: %d\n", unLoopCounter);
        uint32_t unSensorDataSize;
        if( recv(m_nSocketDescriptor, &unSensorDataSize , sizeof(uint32_t) , 0) < 0)
        {
            printf("recv failed\n");
            exit(0);
        }
        unSensorDataSize = ntohl(unSensorDataSize);
        printf("Received sensor data size: %d\n",  unSensorDataSize);


        char strDataBuffer[unSensorDataSize];
        uint32_t totalByteRead = 0;
        uint32_t n = 0;

        printf("Waiting for data\n");
        while (totalByteRead < unSensorDataSize) {
            if(( n = recv(m_nSocketDescriptor, strDataBuffer , unSensorDataSize - totalByteRead , 0)) < 0)
            {
                printf("recv failed\n");
                exit(0);
            }
            printf("Bytes read: %d / %d:\n", n, unSensorDataSize);
            totalByteRead = totalByteRead + n;
        }

        uint32_t unBufferIndex = 0;
        uint8_t unSensorId;

        while (unBufferIndex < unSensorDataSize) {

            unSensorId = strDataBuffer[unBufferIndex];
            printf("Sensor Id: %d\n", unSensorId);
            unBufferIndex++;
            //printf("Bytes unserialized: %d / %d\n", unBufferIndex, unSensorDataSize);

            TVirtualSensorData::iterator itVirtualSensorData = m_tVirtualSensorData->find(unSensorId);
            if (itVirtualSensorData == m_tVirtualSensorData->end()) {
                SVirtualSensorData sVirtualSensorData = SVirtualSensorData();
                m_tVirtualSensorData->insert(std::make_pair<uint32_t, SVirtualSensorData>(unSensorId, sVirtualSensorData));
            }

            TVirtualSensorTable::iterator itVirtualSensorTable = m_tVirtualSensorTable->find(unSensorId);
            if (itVirtualSensorTable != m_tVirtualSensorTable->end()) {
                uint32_t unSensorDataBufferSize = (*itVirtualSensorTable).second;

                itVirtualSensorData = m_tVirtualSensorData->find(unSensorId);
                (*itVirtualSensorData).second.SetPreparingBuffer(strDataBuffer + unBufferIndex, unSensorDataBufferSize);
                unBufferIndex = unBufferIndex + unSensorDataBufferSize;

                (*itVirtualSensorData).second.SwitchBuffers();

            }
            else {
                printf("Virtual Sensor %d not in VST\n", unSensorId);
                exit(0);
            }
        }

        PrintVirtualSensorData();
    }

}


/****************************************/
/****************************************/

void CEpuckITSClient::PrintVirtualSensorTable()
{
    printf("***************************\n");
    printf("Virtual Sensor Table:\n");
    TVirtualSensorTable::iterator itVirtualSensorTable;
    for (itVirtualSensorTable = m_tVirtualSensorTable->begin(); itVirtualSensorTable != m_tVirtualSensorTable->end(); itVirtualSensorTable++)
    {
        //printf("\tSensor ID: %d\tData size: %d\n", (*itVirtualSensorTable).first, (*itVirtualSensorTable).second.GetDataSize());
        printf("\tSensor ID: %d\tData size: %d\n", (*itVirtualSensorTable).first, (*itVirtualSensorTable).second);
    }
    printf("***************************\n");
}

/****************************************/
/****************************************/

void CEpuckITSClient::PrintVirtualSensorData()
{
    printf("***************************\n");
    printf("Virtual Sensor Data: sensors availble %d\n", m_tVirtualSensorData->size());
    TVirtualSensorData::iterator itVirtualSensorData;
    for (itVirtualSensorData = m_tVirtualSensorData->begin(); itVirtualSensorData != m_tVirtualSensorData->end(); itVirtualSensorData++)
    {

        uint32_t unBufferIndex = 0;
        std::vector<char> strBuffer;

        uint8_t unSensorId = (*itVirtualSensorData).first;

        if (!(*itVirtualSensorData).second.IsDataReady()) {
            printf("NO DATA READY: displaying preparing data\n");
            GetPreparingData(strBuffer, unSensorId);
        }
        else {
            printf("SOME DATA READY\n");
            GetReadyData(strBuffer, unSensorId);
        }
        uint32_t unBufferSize = strBuffer.size();
        if(unBufferSize == 0) {
            printf("buffer size == 0\n");
            continue;
        }

        printf("buffer size = %d\n", unBufferSize);


        printf("\tSensor ID: %d\n", unSensorId);

        while (unBufferIndex < unBufferSize) {

            //char strSensorId = strBuffer[unBufferIndex];
            //unBufferIndex++;
            //uint8_t unSensorId = strSensorId;
            //printf("\tSensor ID: %d\n", unSensorId);
            //uint32_t unSensorDataSize = GetDataSizeBySensorId(unSensorId);
            //printf("Data size: %d\n", unSensorDataSize);

            //uint32_t unSensorIndex = 0;
            //while(unSensorIndex < unSensorDataSize) {

                char strSensorSingleValue[8];
                for (uint32_t unByteIndex = 0; unByteIndex < 8; unByteIndex++) {
                    strSensorSingleValue[unByteIndex] = strBuffer[unBufferIndex];
                    unBufferIndex++;
                    //unSensorIndex++;
                }
                double fSensorSingleValue;
                memcpy(&fSensorSingleValue,strSensorSingleValue,sizeof(double));
                //printf("buffer index: %d / %d\n", unBufferIndex, unBufferSize);
                //printf("sensor index: %d / %d\n", unSensorIndex, unSensorDataSize);

                printf("double Value = %f\n", fSensorSingleValue);
            //}

            //printf("**************\n");
        }
        printf("**************\n");


    }
    printf("***************************\n");

}

/****************************************/
/****************************************/

void CEpuckITSClient::GetReadyData(std::vector<char> & byte_data_buffer, uint8_t un_sensor_id)
{
    TVirtualSensorData::iterator itVirtualSensorData = m_tVirtualSensorData->find(un_sensor_id);
    if (itVirtualSensorData != m_tVirtualSensorData->end()) {

        (*itVirtualSensorData).second.GetReadyBuffer(byte_data_buffer);

    }
}

/****************************************/
/****************************************/

void CEpuckITSClient::GetPreparingData(std::vector<char> & byte_data_buffer, uint8_t un_sensor_id)
{
    TVirtualSensorData::iterator itVirtualSensorData = m_tVirtualSensorData->find(un_sensor_id);
    if (itVirtualSensorData != m_tVirtualSensorData->end()) {

         (*itVirtualSensorData).second.GetPreparingBuffer(byte_data_buffer);

    }
}

/****************************************/
/****************************************

uint32_t CEpuckITSClient::GetDataSizeBySensorId(uint8_t un_sensor_id)
{
    TVirtualSensorTable::iterator itVirtualSensorTable = m_tVirtualSensorTable->find(un_sensor_id);
    if (itVirtualSensorTable != m_tVirtualSensorTable->end()) {
        //DEBUG_FUNCTION_EXIT;
        return (*itVirtualSensorTable).second;
    }
    return uint32_t(0);
}


/****************************************/
/****************************************/



}

