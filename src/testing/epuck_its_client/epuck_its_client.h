#ifndef EPUCK_ITS_CLIENT
#define EPUCK_ITS_CLIENT

#include <string>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <map>
#include <vector>


namespace argos {

class CEpuckITSClient
{

    struct SVirtualSensorData {
        std::vector<char> vecByteVirtualSensorReadyData;
        std::vector<char> vecByteVirtualSensorPreparingData;
        pthread_mutex_t cMutex;
        bool bIsDataReady;

        SVirtualSensorData()
            : bIsDataReady(false)
        {
            pthread_mutex_init(&cMutex, NULL);
        }

        inline void GetReadyBuffer(std::vector<char> & vec_ready_buffer) {

            vec_ready_buffer = vecByteVirtualSensorReadyData;
        }

        void SetPreparingBuffer(char* str_preparing_data, uint32_t un_num_bytes) {

            pthread_mutex_lock(&cMutex);

            for (uint32_t unCharBufferIndex = 0; unCharBufferIndex < un_num_bytes; unCharBufferIndex++) {
                vecByteVirtualSensorPreparingData.push_back(str_preparing_data[unCharBufferIndex]);
            }

            pthread_mutex_unlock(&cMutex);
        }

        inline void * GetPreparingBuffer(std::vector<char>& vec_preparing_buffer) {
            vec_preparing_buffer = vecByteVirtualSensorPreparingData;
        }

        inline bool IsDataReady() {
            return bIsDataReady;
        }

        void SwitchBuffers() {

            pthread_mutex_lock(&cMutex);

            vecByteVirtualSensorPreparingData.swap(vecByteVirtualSensorReadyData);
            vecByteVirtualSensorPreparingData.clear();
            bIsDataReady = true;

            pthread_mutex_unlock(&cMutex);
        }
    };

public:
    typedef std::map<uint8_t, uint32_t> TVirtualSensorTable;
    typedef std::map<uint8_t, SVirtualSensorData> TVirtualSensorData;

public:

    CEpuckITSClient(std::string str_argos_server_address, uint32_t un_argos_server_port);

    void Run();

private:

    void InitClient();

    void ReceiveVirtualSensorTable();

    void ExecuteExperiment();

    void PrintVirtualSensorTable();

    void PrintVirtualSensorData();

    void GetReadyData(std::vector<char> & byte_data_buffer, uint8_t un_sensor_id);

    void GetPreparingData(std::vector<char> & byte_data_buffer, uint8_t un_sensor_id);

    //uint32_t GetDataSizeBySensorId(uint8_t un_sensor_id);

private:

    int16_t m_nSocketDescriptor;
    struct sockaddr_in m_sServer;

    std::string m_strArgosServerAddress;
    uint32_t m_unArgosServerPort;

    TVirtualSensorTable * m_tVirtualSensorTable;
    TVirtualSensorData * m_tVirtualSensorData;

};

}

#endif
