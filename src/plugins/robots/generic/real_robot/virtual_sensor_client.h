/**
 * @file <argos3/plugins/robots/generic/control_interface/virtual_sensor_client.h>
 *
 * @brief This file provides the Virtual Sensor data structure as a singleton
 *
 *
 * @author Mattia Salvaro
 */

#ifndef VIRTUAL_SENSOR_CLIENT_H
#define VIRTUAL_SENSOR_CLIENT_H

#include <arpa/inet.h>
#include <cerrno>
#include <map>
#include <vector>
#include <memory>
// #ifndef __APPLE__
// #include <auto_ptr.h>
// #endif
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/configuration/argos_exception.h>
#include <argos3/core/utility/datatypes/byte_array.h>
#include <argos3/core/utility/networking/tcp_socket.h>
#include <argos3/plugins/robots/generic/control_interface/virtual_sensor_network_data.h>


namespace argos {

class CVirtualSensorClient : public CTCPSocket
{
	/**
	     * @brief The SVirtualSensorData struct Defines the Virtual Sensor data structure for one robot.
	     * It uses double byte buffer in form of UInt8 vector to store and update the data.
	     * Operation on the buffers must be done within a critic session granted by mutex.
	     */
	    struct SVirtualSensorData{

	        /**
	         * @brief unRobotId Robot ID
	         */
	        UInt32 unRobotId;

	        /**
	         * @brief vecVirtualSensorReadyData Data byte buffer for readable ready data
	         */
	        CByteArray vecVirtualSensorReadyData;

	        /**
	         * @brief vecVirtualSensorPreparingData Data byte buffer for preparing writable data
	         */
	        CByteArray vecVirtualSensorPreparingData;

	        /**
	         * @brief cMutex Mutex for critic sessions
	         */
	        pthread_mutex_t cMutex;

	        SVirtualSensorData()
	            : unRobotId(0)
	        {
	            pthread_mutex_init(&cMutex, NULL);
	        }

	        /*
	         * Accessors to the structure fields
	         */

	        inline void GetReadyBuffer(CVirtualSensorNetworkData& c_object) {
	        	if (vecVirtualSensorReadyData.Size())
	        		c_object.Deserialize(vecVirtualSensorReadyData);
	        }

	        inline void GetPreparingBuffer(CVirtualSensorNetworkData& c_object) {
	        	if (vecVirtualSensorPreparingData.Size())
	        		c_object.Deserialize(vecVirtualSensorPreparingData);
	        }

	        /*
	         * Modifiers of the structure fields
	         */

	        /**
	         * @brief SetPreparingBuffer Writes the number of bytes given by un_num_bytes,
	         * taken from pun_preparing_data, to the preparing buffer.
	         * @param str_preparing_data The input buffer
	         * @param un_num_bytes The amount of bytes to write
	         */
	        void SetPreparingBuffer(const UInt8* pun_preparing_data, UInt32 un_num_bytes) {
				pthread_mutex_lock(&cMutex);
				vecVirtualSensorPreparingData.AddBuffer(pun_preparing_data, un_num_bytes);
				pthread_mutex_unlock(&cMutex);
			}
	        void SetPreparingBuffer(const CByteArray& c_preparing_data) {
				pthread_mutex_lock(&cMutex);
				vecVirtualSensorPreparingData.AddBuffer(c_preparing_data.ToCArray(),c_preparing_data.Size());
				pthread_mutex_unlock(&cMutex);
			}

	        /**
			 * @brief SwitchBuffer swap the two buffers, and clear the preparing data.
			 */
	        void SwitchBuffers() {
	            pthread_mutex_lock(&cMutex);
	            vecVirtualSensorPreparingData.Swap(vecVirtualSensorReadyData);
	            vecVirtualSensorPreparingData.Clear();
	            pthread_mutex_unlock(&cMutex);
	        }
	    };

public:

    /**
     * @brief TVirtualSensorTable <Sensor ID, Sensor Data Size>
     */
    typedef std::map<UInt8, UInt32> TVirtualSensorTable;

    /**
     * @brief TVirtualSensorData <Sensor ID, Sensor Data>
     */
    typedef std::map<UInt8, SVirtualSensorData> TVirtualSensorData;

    // Pattern Singleton
    /**
     * @brief GetInstance The method to access the CVirtualSensorClient singleton
     * @return A reference to the singleton CVirtualSensorClient
     */
    static CVirtualSensorClient& GetInstance()
    {
        static std::auto_ptr<CVirtualSensorClient> pcVirtualSensorClientInstance(new CVirtualSensorClient());
        return *(pcVirtualSensorClientInstance.get());
    }

    /**
     * @brief SetServerAddresAndPort Set the proper IP address and port after the XML file had been parsed
     * @param str_server_address The given Iridia Tracking System Physics Engine server address
     * @param un_server_port The given Iridia Tracking System Physics Engine server port
     */
    void SetServerAddresAndPort(const std::string & str_server_address, uint32_t un_server_port);

private:

    // Pattern Singleton

    /**
     * @brief CVirtualSensorClient Private constructor for singleton
     */
    //CVirtualSensorClient(const std::string & str_argos_server_address, uint32_t un_argos_server_port);

    CVirtualSensorClient();

    /**
     * @brief CVirtualSensorClient Private copy constructor for singleton
     */
    CVirtualSensorClient(const CVirtualSensorClient& c_virtual_sensor_client):
    	m_strArgosServerAddress(""),
    	m_unArgosServerPort(0),
    	m_tVirtualSensorTable(NULL),
    	m_tVirtualSensorData(NULL),
    	m_bIsArgosPlaying(false),
    	m_bErrorOnSocket(false),
    	m_bVSTReceived(false){
    }

    /**
     * @brief operator = Overridden not implemented for singleton
     */
    CVirtualSensorClient& operator=(const CVirtualSensorClient& c_virtual_sensor_client) {return *this;}
    // ------------------

    /**
     * @brief Run Entry point for client thread
     */
    static void *Run(void *p);

    /**
     * @brief InitClient Establishes connection with server
     */
    void InitClient();

    /**
     * @brief ReceiveVirtualSensorTable Receives and initializes VST containing sensor information
     */
    void ReceiveVirtualSensorTable();

    /**
     * @brief WaitForArgosToStart Waits until PLAY button is pressed on Argos interface
     */
    void WaitForArgosToStart();

    /**
     * @brief ExecuteExperiment Body of the client. Waits for data updates from the server
     */
    void ExecuteExperiment();

    /**
     * @brief PrintVirtualSensorTable Prints the VST
     */
    void PrintVirtualSensorTable();

public:

    /**
     * @brief GetReadyData Gets the buffer containing the ready data for a given sensor
     * @param un_sensor_id The given sensor id
     * @return The ByteArray containing the ready data
     */
    void GetReadyData(CVirtualSensorNetworkData& c_object, const UInt8& un_sensor_id);

    /**
     * @brief GetPreparingData Gets the buffer containing the soon-to-be ready data for a given sensor
     * @param un_sensor_id The given sensor id
     * @return The ByteArray containing the preparing data
     */
    void GetPreparingData(CVirtualSensorNetworkData& c_object, const UInt8& un_sensor_id);

    /**
     * @brief SetInitData Initializes the data structure with given values for a given sensor
     * @param str_preparing_data The input buffer
     */
    void SetInitData(const CByteArray& c_preparing_data, const UInt8& un_sensor_id);

    /**
	 * @brief IsArgosPlaying return the playing state received from the simulator
	 * @return un_num_bytesthe playing state received from the simulator
	 */
    inline bool IsArgosPlaying()
    {
        return m_bIsArgosPlaying;
    }

    /**
	 * @brief ErrorOnSocket return if there was an error on the socket till start
	 * @return if there was an error on the socket till start
	 */
    inline bool ErrorOnSocket(){
    	return m_bErrorOnSocket;
    }


    /**
	 * @brief ForceStartOfExperiment force the experiment to start if a connection is established
	 */
    inline void ForceStartOfExperiment(){
		m_bIsArgosPlaying = true;
	}

    /**
	 * @brief ForceEndOfConnection force the experiment to stop and close the thread
	 */
    inline void ForceEndOfConnection(){
    	m_bIsArgosPlaying = false;
    }

    void Destroy();

    /**
	 * @brief return if the Virtual Sensor Table has been received
	 * @return Virtual Sensor Table is received or not
	 */
    inline bool IsVSTReceived() {
    	return m_bVSTReceived;
    }

    //uint32_t GetDataSizeBySensorId(uint8_t un_sensor_id);

private:

    // Virtual Sensor Server address and port
    std::string m_strArgosServerAddress;
    u_int32_t m_unArgosServerPort;

    TVirtualSensorTable * m_tVirtualSensorTable;
    TVirtualSensorData * m_tVirtualSensorData;

    // Flags
    bool m_bIsArgosPlaying;
    bool m_bErrorOnSocket;
    bool m_bVSTReceived;

};

}

#endif
