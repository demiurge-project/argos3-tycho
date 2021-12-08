/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/virutal_sensor_server/virutal_sensor_server.h>
 *
 * @brief This file provides the Virtual Sensor Server as a singleton.
 * The Virtual Sensor Server is a server running within the Iridia Tracking System physics engine.
 * It allows the communication between the physics engine in ARGoS and the clients deployed on the robots.
 *
 *
 * @author Mattia Salvaro
 */


#ifndef VIRTUAL_SENSOR_SERVER_H
#define VIRTUAL_SENSOR_SERVER_H

#include <arpa/inet.h>
#include <unistd.h>
#include <memory>
// #ifndef __APPLE__
// #include <auto_ptr.h>
// #endif

#include <argos3/core/utility/configuration/argos_exception.h>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/string_utilities.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/networking/tcp_socket.h>

#include <argos3/plugins/simulator/physics_engines/iridia_tracking_system/virtual_sensor_server/virtual_sensor_data.h>

namespace argos {

class CVirtualSensorServer : public CTCPSocket
{

public:

    /**
     * @brief THashRobotSocketFD <Robot ID (IP), Socket descriptor>
     */
    typedef std::map<UInt32, CTCPSocket*> THashRobotSocketFD;

public:

    // Pattern Singleton
    /**
     * @brief GetInstance The method to access the CVirtualSensorServer singleton
     * @return  A reference to the singleton CVirtualSensorServer
     */
    static CVirtualSensorServer & GetInstance()
    {
        static std::auto_ptr<CVirtualSensorServer> pcVirtualSensorServerInstance(new CVirtualSensorServer());
        return *(pcVirtualSensorServerInstance.get());
    }

    /**
      * Destructor
      */
    ~CVirtualSensorServer();

    /**
     * @brief Launch Creates the thread that hosts the Virtual Sensor Server
     */
    void Launch();

    /**
     * @brief SendArgosSignal Sends 32 bit signal when Argos starts and stops
     */
    void SendArgosSignal(UInt32 n_signal);

    /**
     * @brief ExperimentStarted To handle Temp Hash Robot / Socket ID
     */
    void ExperimentStarted();

    /**
     * @brief SendAllVirtualSensorData Sends the ready sensor data to all the robots
     */
    void SendAllVirtualSensorData();

    /**
     * @brief SwapBuffers Swaps the virtual sensor data double buffer for a given robot
     * @param un_robot_id The given robot
     */
    void SwapBuffers(UInt32 un_robot_id);

    /**
     * @brief SetVirtualSensorServerPort Sets the port on which the servre must open the socket
     * @param un_virtual_sensor_server_port The given port
     */
    inline void SetVirtualSensorServerPort(UInt32 un_virtual_sensor_server_port)
    {
        m_unVirtualSensorServerPort = un_virtual_sensor_server_port;
    }


private:

    /**
     * @brief m_unArgosServerPort The port on which the Virtual Sensor Server is running
     */
    UInt32 m_unVirtualSensorServerPort;

    /**
     * @brief m_tHashRobotSocketId Pointer to the robot ID / socket descriptor table
     */
    THashRobotSocketFD * m_tHashRobotSocketId;

    /**
     * @brief m_tTempHashRobotSocketId To handle disconnection and reconnection of clients
     */
    THashRobotSocketFD * m_tTempHashRobotSocketId;

    /**
     * @brief m_cVirtualSensorData Reference to the singleton Virtual Sensor Data Structure
     */
    CVirtualSensorData & m_cVirtualSensorData;

    // NEW
    bool m_bEnabled;

private:

    /* Pattern Singleton *******/
    /**
     * @brief CVirtualSensorServer Constructor for singleton
     */
    CVirtualSensorServer();

    /**
     * @brief CVirtualSensorServer Copy constructor for singleton
     */
    CVirtualSensorServer(const CVirtualSensorServer &);

    /**
     * @brief operator = Overridden not implemented for singleton
     */
    void operator=(const CVirtualSensorServer &);

    /**
     * @brief Run Entry point for server thread
     */
    static void *Run(void *p);

    /**
     * @brief SessionOpened Opens the server socket
     */
    void SessionOpened();

    /**
     * @brief AcceptClientConnection Accepts connections from clients
     */
    UInt32 AcceptClientConnection();

    /**
     * @brief SendVirtualSensorTable Sends the serialized Virtual Sensor Table to all the clients
     */
    void SendVirtualSensorTable(UInt32 un_robot_id);

    /**
     * @brief GetRobotIdFromIP Returns the robot ID (i.e. the last octet of the IP address) given the robot IP
     * @param unRobotIP The robot IP in network form
     * @return The robot ID, i.e. the last octet of the IP address
     */
    UInt32 GetRobotIdFromIP(const std::string& un_robot_ip);
};
}

#endif
