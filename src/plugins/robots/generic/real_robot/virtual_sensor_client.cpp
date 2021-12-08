#include "virtual_sensor_client.h"

namespace argos {

/****************************************/
/****************************************/

CVirtualSensorClient::CVirtualSensorClient() :
    m_strArgosServerAddress(""),
    m_unArgosServerPort(0),
    m_bIsArgosPlaying(false),
    m_bErrorOnSocket(false),
    m_bVSTReceived(false)
{
    m_tVirtualSensorTable = new TVirtualSensorTable();
    m_tVirtualSensorData = new TVirtualSensorData();

    // Create thread to host the Virtual Sensor Client
    pthread_t cThread;
    pthread_create(&cThread, NULL, &Run, this);

}

/****************************************/
/****************************************/

void CVirtualSensorClient::Destroy()
{
    delete m_tVirtualSensorTable;
    delete m_tVirtualSensorData;
}

/****************************************/
/****************************************/

void CVirtualSensorClient::SetServerAddresAndPort(const std::string &str_server_address, uint32_t un_server_port)
{
    m_strArgosServerAddress = str_server_address;
    m_unArgosServerPort = un_server_port;
}

/****************************************/
/****************************************/

void *CVirtualSensorClient::Run(void* p)
{
    // Life cycle of the Client
    CVirtualSensorClient *cClientThread = reinterpret_cast<CVirtualSensorClient*>(p);
    try{
		cClientThread->InitClient();
		cClientThread->ReceiveVirtualSensorTable();
		cClientThread->PrintVirtualSensorTable();
		cClientThread->WaitForArgosToStart();
		cClientThread->ExecuteExperiment();
	}
    catch(CARGoSException& ex){
    	cClientThread->m_bErrorOnSocket = true;
    	DEBUG("Socket error\n")
    }
    return p;
}



/****************************************/
/****************************************/

void CVirtualSensorClient::InitClient()
{
    // Wait for any Virtual Sensor to set the Argos server address
    while (m_strArgosServerAddress.empty() || !m_unArgosServerPort) {
    	pthread_yield();
    }
    LOG << "Server Address: "<< m_strArgosServerAddress.c_str() <<"\nServer Port: "<< m_unArgosServerPort <<"\n";
    Connect(m_strArgosServerAddress,m_unArgosServerPort);
}

/****************************************/
/****************************************/

void CVirtualSensorClient::ReceiveVirtualSensorTable()
{
    CByteArray cBuffer;
    if(!ReceiveByteArray(cBuffer)){
    	//Closed socket
    	THROW_ARGOSEXCEPTION("Socket closed before receiving the VST");
    }

    UInt8 unSensorId;
    UInt32 unSensorDataSize;

    // Parse the byte buffer according to the model:
    // 1 byte for sensor ID + 4 bytes for sensor data size
    while (cBuffer.Size()) {
        // Get the sensor ID & sensor data size
    	cBuffer >> unSensorId >> unSensorDataSize;

        LOG << "Sensor Entry:" << unSensorId << " - " << unSensorDataSize << "\n";

        // Insert the Sensor Table Entry in the Virtual Sensor Table
        m_tVirtualSensorTable->insert(std::make_pair<UInt8, UInt32>(unSensorId, unSensorDataSize));
    }

    m_bVSTReceived = true;
}

/****************************************/
/****************************************/

void CVirtualSensorClient::WaitForArgosToStart()
{
    // Receive a START signal by the Argos VSS
    UInt32 unStartSignal;
    do {
    	if(!ReceiveBuffer(reinterpret_cast<UInt8*>(&unStartSignal),sizeof(UInt32))){
			//Closed socket
			THROW_ARGOSEXCEPTION("Socket closed before receiving the VST");
		}
        unStartSignal = ntohl(unStartSignal);
    } while (unStartSignal != 1);

    m_bIsArgosPlaying = true;
}

/****************************************/
/****************************************/

void CVirtualSensorClient::ExecuteExperiment()
{
    //UInt32 unLoopCounter = 0;
    CByteArray cBuffer;
    UInt8 unSensorId;
    // Infinite loop in which the client waits for a 32 bit integer that carries the size of the overall Virtual Sensor Data
    while(true) {
        //unLoopCounter++;
        //DEBUG("Waiting for data size. Loop: %d\n", unLoopCounter);

		UInt32 unSizeNBO;
		if(!ReceiveBuffer(reinterpret_cast<UInt8*>(&unSizeNBO), sizeof(unSizeNBO))) {
			THROW_ARGOSEXCEPTION("Socket closed");
		}
		unSizeNBO = ntohl(unSizeNBO);
		// Close signal is zero and received
		if (!unSizeNBO){
			m_bIsArgosPlaying = false;
			break;
		}
		// Else it is size
		/* Receive the actual data */
		cBuffer.Resize(unSizeNBO);
		if(!ReceiveBuffer(cBuffer.ToCArray(), cBuffer.Size())) {
			THROW_ARGOSEXCEPTION("Socket closed");
		}
        // The byte buffer is parsed to retrieve the values of the virtual sensors sent by the Virtual Sensor Server
        while (cBuffer.Size()) {

            // Get the sensor ID (one byte)
        	cBuffer >> unSensorId;

            // If a Virtual Sensor Data Entry is not present in the VSD the create it
            TVirtualSensorData::iterator itVirtualSensorData = m_tVirtualSensorData->find(unSensorId);
            if (itVirtualSensorData == m_tVirtualSensorData->end()) {
                SVirtualSensorData sVirtualSensorData = SVirtualSensorData();
                m_tVirtualSensorData->insert(std::make_pair<uint32_t, SVirtualSensorData>(unSensorId, sVirtualSensorData));
            }

            // Look for the Virtual Sensor Table Entry for this sensor
            TVirtualSensorTable::iterator itVirtualSensorTable = m_tVirtualSensorTable->find(unSensorId);
            if (itVirtualSensorTable != m_tVirtualSensorTable->end()) {
                // Get the corresponding Virtual Sensor Data size from the Virtual Sensor Table Entry
                UInt32 unSensorDataBufferSize = (*itVirtualSensorTable).second;

                // Look again for the Virtual Sensor Data Entry (now it must exist)
                itVirtualSensorData = m_tVirtualSensorData->find(unSensorId);

                UInt8* punBufferContent = new UInt8[unSensorDataBufferSize];
                cBuffer.FetchBuffer(punBufferContent, unSensorDataBufferSize);
                // Set the writing buffer of the Virtual Sensor Data with the corresponding sub buffer in the byte buffer
                itVirtualSensorData->second.SetPreparingBuffer(punBufferContent, unSensorDataBufferSize);
                delete punBufferContent;
                // Make the written buffer available for readings
                (*itVirtualSensorData).second.SwitchBuffers();

            }
            else {
                THROW_ARGOSEXCEPTION("Virtual Sensor not in VST\n");
            }
        }
    }
}


/****************************************/
/****************************************/

void CVirtualSensorClient::PrintVirtualSensorTable()
{
    LOG << "***************************\n";
    LOG << "Virtual Sensor Table:\n";
    TVirtualSensorTable::iterator itVirtualSensorTable;
    for (itVirtualSensorTable = m_tVirtualSensorTable->begin(); itVirtualSensorTable != m_tVirtualSensorTable->end(); itVirtualSensorTable++)
    {
        LOG << "\tSensor ID: " << (*itVirtualSensorTable).first << "\tData size: " << (*itVirtualSensorTable).second << "\n";
    }
    LOG <<"***************************\n";
}

/****************************************/
/****************************************/

void CVirtualSensorClient::GetReadyData(CVirtualSensorNetworkData& c_object, const UInt8& un_sensor_id)
{
    TVirtualSensorData::iterator itVirtualSensorData = m_tVirtualSensorData->find(un_sensor_id);
    if (itVirtualSensorData != m_tVirtualSensorData->end()) {
    	(*itVirtualSensorData).second.GetReadyBuffer(c_object);
    }

}

/****************************************/
/****************************************/

void CVirtualSensorClient::GetPreparingData(CVirtualSensorNetworkData& c_object,const UInt8& un_sensor_id)
{
    TVirtualSensorData::iterator itVirtualSensorData = m_tVirtualSensorData->find(un_sensor_id);
    if (itVirtualSensorData != m_tVirtualSensorData->end()) {
    	(*itVirtualSensorData).second.GetPreparingBuffer(c_object);
    }

}

/****************************************/
/****************************************/

void CVirtualSensorClient::SetInitData(const CByteArray& c_preparing_data, const UInt8& un_sensor_id){
    // If a Virtual Sensor Data Entry is not present in the VSD the create it
    TVirtualSensorData::iterator itVirtualSensorData = m_tVirtualSensorData->find(un_sensor_id);
    if (itVirtualSensorData == m_tVirtualSensorData->end()) {
        SVirtualSensorData sVirtualSensorData = SVirtualSensorData();
        m_tVirtualSensorData->insert(std::make_pair<uint32_t, SVirtualSensorData>(un_sensor_id, sVirtualSensorData));
    }

    // Look for the Virtual Sensor Table Entry for this sensor
    TVirtualSensorTable::iterator itVirtualSensorTable = m_tVirtualSensorTable->find(un_sensor_id);

    if (itVirtualSensorTable != m_tVirtualSensorTable->end()) {
        // Get the corresponding Virtual Sensor Data size from the Virtual Sensor Table Entry
        if (itVirtualSensorTable->second != c_preparing_data.Size())
        	THROW_ARGOSEXCEPTION("SetInitData: Incorrect buffer size, incorrect init of a sensor\n");

        // Look again for the Virtual Sensor Data Entry (now it must exist)
        itVirtualSensorData = m_tVirtualSensorData->find(un_sensor_id);
        // Set the writing buffer of the Virtual Sensor Data with the corresponding sub buffer in the byte buffer
        itVirtualSensorData->second.SetPreparingBuffer(c_preparing_data);
        // Make the written buffer available for readings
        itVirtualSensorData->second.SwitchBuffers();

    }
    else {
        THROW_ARGOSEXCEPTION("Virtual Sensor not in VST\n");
    }

}

/****************************************/
/****************************************/



}

