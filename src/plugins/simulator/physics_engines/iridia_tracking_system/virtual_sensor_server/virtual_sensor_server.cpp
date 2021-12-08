/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/virutal_sensor_server/virutal_sensor_server.cpp>
 *
 *
 * @author Mattia Salvaro
 */

#include "virtual_sensor_server.h"

namespace argos {

/****************************************/
/****************************************/

CVirtualSensorServer::CVirtualSensorServer()
    : CTCPSocket(),
      m_unVirtualSensorServerPort(0),
      m_cVirtualSensorData(CVirtualSensorData::GetInstance()),
      m_bEnabled(false)
{
    m_tHashRobotSocketId = new THashRobotSocketFD();
    m_tTempHashRobotSocketId = new THashRobotSocketFD();
}

/****************************************/
/****************************************/

CVirtualSensorServer::~CVirtualSensorServer()
{
    delete m_tHashRobotSocketId;
    delete m_tTempHashRobotSocketId;
}


/****************************************/
/****************************************/

void CVirtualSensorServer::Launch()
{
    // Create thread to host the Virtual Sensor Server
    pthread_t cThread;
    pthread_create(&cThread, NULL, &Run, this);
}

/****************************************/
/****************************************/

void *CVirtualSensorServer::Run(void* p)
{
	try{
		CVirtualSensorServer *cServerThread = reinterpret_cast<CVirtualSensorServer*>(p);

		cServerThread->SessionOpened();

		while (true) {
			// Inifite loop of accept
			UInt32 unRobotId = cServerThread->AcceptClientConnection();

			// And init connected client
			cServerThread->SendVirtualSensorTable(unRobotId);
		}
	}
	catch(CARGoSException& ex){
		LOG << "Incomming connection thread exiting\n";
	}
    return p;
}

/****************************************/
/****************************************/

void CVirtualSensorServer::SessionOpened()
{
    // Wait for the physics engine to set the value of the server port
    while(m_unVirtualSensorServerPort == 0) {
    	;
    }
    Listen(m_unVirtualSensorServerPort, 1000);
}

/****************************************/
/****************************************/

UInt32 CVirtualSensorServer::AcceptClientConnection()
{

    DEBUG("Accepting incoming connection\n");

    CTCPSocket* cNewSocket = new CTCPSocket();
    Accept(*cNewSocket);
    UInt32 unRobotId = GetRobotIdFromIP(cNewSocket->GetAddress());

    // Create the entry in the robot ID / socket destcriptor table
    // VERY IMPORTANT: delete previous entry for corrupted socket id
    THashRobotSocketFD::iterator itHash = m_tTempHashRobotSocketId->find(unRobotId);
    if (itHash != m_tTempHashRobotSocketId->end()) {
    	delete itHash->second;
        m_tTempHashRobotSocketId->erase(itHash);
    }
    //m_tTempHashRobotSocketId->insert(std::make_pair<UInt32, CTCPSocket*>(unRobotId, cNewSocket));
    m_tTempHashRobotSocketId->insert(std::make_pair(unRobotId, cNewSocket));
    m_cVirtualSensorData.ClearVirtualSensorData(unRobotId);
    DEBUG("Robot %d connected\n",unRobotId);
    return unRobotId;
}

/****************************************/
/****************************************/

void CVirtualSensorServer::SendVirtualSensorTable(UInt32 un_robot_id)
{
    THashRobotSocketFD::iterator itHashRobotSocketId = m_tTempHashRobotSocketId->find(un_robot_id);
    if (itHashRobotSocketId == m_tTempHashRobotSocketId->end()) {
        THROW_ARGOSEXCEPTION("Robot not connected");
    }

    // Fill the byte array with the serialized Virtual Sendsor table
    const CByteArray* cSerializedTable= m_cVirtualSensorData.GetSerializedVirtualSensorTable();

    DEBUG("Sending virtual sensor table to robot: %d \tsize: %d\n",
    		itHashRobotSocketId->first,cSerializedTable->Size());
    try {
    	itHashRobotSocketId->second->SendByteArray(*cSerializedTable);
    }
    catch(CARGoSException& ex){
    	DEBUG("Send of Virtual Sensor Table to robot %d failed\n",itHashRobotSocketId->first);
    	delete itHashRobotSocketId->second;
		m_tTempHashRobotSocketId->erase(itHashRobotSocketId);
    }
    delete cSerializedTable;
}

/****************************************/
/****************************************/

UInt32 CVirtualSensorServer::GetRobotIdFromIP(const std::string& un_robot_ip)
{
    // Tokenize the string and select the last tocken
    std::vector<std::string> vecTokens;
    std::stringstream cStringStream(un_robot_ip);
    std::string strToken;

    while (std::getline(cStringStream, strToken, '.')) {
        vecTokens.push_back(strToken);
    }

    // Convert the string to int
    return FromString<UInt32>(vecTokens[vecTokens.size()-1]);
}


/****************************************/
/****************************************/

void CVirtualSensorServer::SendArgosSignal(UInt32 n_signal)
{
	//Signal 1 mean start of experiment
	//Signal 0 mean close of experiment
	//No other signals for now
	if (n_signal > 1){
		THROW_ARGOSEXCEPTION("Unknown ARGoS signal to send to the robots");
	}
    THashRobotSocketFD::iterator itHashRobotSocketId;
    if (m_tHashRobotSocketId->empty()) {
        DEBUG("SendArgosSignal\n");
    }

    // For each entry in the hash robot socket id send the same signal
    UInt32 nNetworkSignal = htonl(n_signal);
    UInt8* punBuffer = reinterpret_cast<UInt8*> (&nNetworkSignal);

    for (itHashRobotSocketId = m_tHashRobotSocketId->begin(); itHashRobotSocketId != m_tHashRobotSocketId->end(); itHashRobotSocketId++)
    {
    	itHashRobotSocketId->second->SendBuffer(punBuffer, sizeof(UInt32));
    }
}

/****************************************/
/****************************************/

void CVirtualSensorServer::ExperimentStarted()
{
	//Close all old sockets
	THashRobotSocketFD::iterator itHash;
	for (itHash = m_tHashRobotSocketId->begin(); itHash != m_tHashRobotSocketId->end(); itHash++)
	{
		delete itHash->second;
		m_tHashRobotSocketId->erase(itHash);
	}
	if (!m_tHashRobotSocketId->empty()){
		THROW_ARGOSEXCEPTION("[VirtualSensorServer] Hash not empty after cleaning it !")
	}
	//Insert new sockets
    /*THashRobotSocketFD::iterator itTempHash;
    for (itTempHash = m_tTempHashRobotSocketId->begin(); itTempHash != m_tTempHashRobotSocketId->end(); itTempHash++)
    {
        m_tHashRobotSocketId->insert(*itTempHash);
        itHash = m_tHashRobotSocketId->find((*itTempHash).first);
    }
    m_tTempHashRobotSocketId->clear();*/
	m_tHashRobotSocketId->swap(*m_tTempHashRobotSocketId);
}

/****************************************/
/****************************************/

void CVirtualSensorServer::SendAllVirtualSensorData()
{
    THashRobotSocketFD::iterator itHashRobotSocketId;
    if (m_tHashRobotSocketId->empty()) {
        DEBUG("SendAllVirtualSensorData\n");
    }

    // For each entry in the hash robot socket id send the corresponding data on the socket
    for (itHashRobotSocketId = m_tHashRobotSocketId->begin(); itHashRobotSocketId != m_tHashRobotSocketId->end(); itHashRobotSocketId++)
    {
        // Get the robot ID
        UInt32 unRobotId = (*itHashRobotSocketId).first;
        DEBUG("Sending virtual sensor data to robot: %d\n",unRobotId);
        try {
			// Get the ready data buffer for the given robot
			const CByteArray& cData = m_cVirtualSensorData.GetReadyData(unRobotId);

			//If there is no data skip this robot
			if (!cData.Size()) {
				DEBUG("No data to send for robot %d\n",unRobotId);
			}
			else {
				itHashRobotSocketId->second->SendByteArray(cData);
			}
        }
		catch(CARGoSException &ex){
			DEBUG("Send failed\n");
			delete itHashRobotSocketId->second;
			m_tHashRobotSocketId->erase(itHashRobotSocketId);
        }
    }
}

/****************************************/
/****************************************/

void CVirtualSensorServer::SwapBuffers(UInt32 un_robot_id)
{
    m_cVirtualSensorData.SwapBuffers(un_robot_id);
}

/****************************************/
/****************************************/

}
