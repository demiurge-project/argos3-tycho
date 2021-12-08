/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/virutal_sensor_server/virutal_sensor_data.h>
 *
 * @brief This file provides the Virtual Sensor data structure as a singleton
 *
 *
 * @author Mattia Salvaro
 */

#ifndef VIRTUAL_SENSOR_DATA_H
#define VIRTUAL_SENSOR_DATA_H


#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/configuration/argos_exception.h>
#include <argos3/core/utility/datatypes/byte_array.h>
#include <argos3/plugins/robots/generic/control_interface/virtual_sensor_network_data.h>

#include <map>
#include <vector>
#include <algorithm>
#include <memory>
// #ifndef __APPLE__
// #include <auto_ptr.h>
// #endif

namespace argos {

class CVirtualSensorData
{

public:

    /**
     * @brief The SVirtualSensorData struct Defines the Virtual Sensor data structure for one robot.
     * It uses double byte buffer in form of UInt8 vector to store and update the data.
     * Operation on the buffers must be done within a critic session granted by mutex.
     */
    struct SVirtualSensorData {

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

        /**
		 * @brief m_mapLastSentData Last data registered per sensor for the current robot
		 */
        std::map<UInt8,CByteArray> m_mapLastSentData;

        /**
         * @brief bIsDataReady Flag that tells if the readable buffer has been written
         */
        bool bIsDataReady;

        SVirtualSensorData():
        	unRobotId(0),
            bIsDataReady(false)
        {
            pthread_mutex_init(&cMutex, NULL);
        }

        /*
         * Accessors to the structure fields
         */

        inline const CByteArray& GetReadyBuffer() {
            return vecVirtualSensorReadyData;
        }

        inline const CByteArray& GetPreparingBuffer() {
            return vecVirtualSensorPreparingData;
        }

        inline const bool IsDataReady() {
            return bIsDataReady;
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
        void SetPreparingBuffer(const CVirtualSensorNetworkData& c_preparing_data, UInt8 un_sensor_id) {
        	CByteArray cPreparingBuffer = c_preparing_data.Serialize();
        	//Last data were exactly the same, should not send it
        	if (m_mapLastSentData[un_sensor_id] == cPreparingBuffer){
        		return;
        	}
        	//Else data differ, send it
        	m_mapLastSentData[un_sensor_id] = cPreparingBuffer;
            pthread_mutex_lock(&cMutex);
            vecVirtualSensorPreparingData.AddBuffer(&un_sensor_id, sizeof(un_sensor_id));
            vecVirtualSensorPreparingData.AddBuffer(cPreparingBuffer.ToCArray(), cPreparingBuffer.Size());
            pthread_mutex_unlock(&cMutex);
        }

        /**
		 * @brief SwitchBuffer swap the two buffers, and clear the preparing data.
		 */
        void SwitchBuffers() {
            pthread_mutex_lock(&cMutex);
            vecVirtualSensorPreparingData.Swap(vecVirtualSensorReadyData);
            vecVirtualSensorPreparingData.Clear();
            bIsDataReady = true;
            pthread_mutex_unlock(&cMutex);
        }
    };


public:

    /**
     * @brief TVirtualSensorTable <Virtual Sensor ID, Virtual Sensor Data Size>
     */
    typedef std::map<UInt8, UInt32> TVirtualSensorTable;

    /**
     * @brief TVirtualSensorData <Robot Id (IP), Virtual Sensor Data Struct>
     */
    typedef std::map<UInt32, SVirtualSensorData> TVirtualSensorData;



public:

    // Pattern Singleton
    /**
     * @brief GetInstance The method to access the CVirtualSensorData singleton
     * @return A reference to the singleton CVirtualSensorData
     */
    static CVirtualSensorData& GetInstance()
    {
        static std::auto_ptr<CVirtualSensorData> pcVirtualSensorDataInstance(new CVirtualSensorData);
        return *(pcVirtualSensorDataInstance.get());
    }

    /**
      * Destructor
      */
    ~CVirtualSensorData();

    /*
     * Accessors to the strucure fields that call internal accessors
     */

    /**
     * @brief GetReadyData Gives the ready readable data buffer for a given robot
     * @param byte_data_buffer Output parameter. The requested buffer
     * @param un_robot_id The given robot id
     */
    const CByteArray& GetReadyData(UInt32 un_robot_id);

    /**
     * @brief GetPreparingData Gives the preparing writeble data buffer for a given robot
     * @param byte_data_buffer Output parameter. The requested buffer
     * @param un_robot_id The given robot id
     */
    const CByteArray& GetPreparingData(UInt32 un_robot_id);

    /*
     * Modifiers of the structure fields that call internal modifiers with mutex session
     */

	/**
	 * @brief AppendVirtualSensorData Appends a given number of bytes to the preparing writable data buffer for a given robot
	 * @param c_virtual_sensor_data The buffer to append to the preparing writable buffer
	 * @param un_robot_id The given robot ID
	 */
	void AppendVirtualSensorData(const CVirtualSensorNetworkData& c_virtual_sensor_data, UInt32 un_robot_id, UInt8 un_sensor_id);

    /**
     * @brief SwapBuffers Swaps the ready readable buffer with the preparing writable buffer for a given robot
     * when all the writings have been performed
     * @param un_robot_id The given robot ID
     */
    void SwapBuffers(UInt32 un_robot_id);


    /*
     * Virtual Sensor Table accessors
     */

    /**
     * @brief GetSerializedVirtualSensorTable Gives the Virtual Sensor Table in form of serialized byte buffer
     * for transmission purpose.
     * @param byte_virtual_sensor_table_buffer Output parameter. The requested byte buffer
     */
    //void GetSerializedVirtualSensorTable(UInt8 * byte_virtual_sensor_table_buffer);
    const CByteArray* GetSerializedVirtualSensorTable();

    /**
     * @brief IsSensorAlreadyInTable Checks if a given sensor is already registered to the Virtual Sensor Table
     * @param un_sensor_id The given sensor ID
     * @return True if the sensor is registered already, false otherwise
     */
    bool IsSensorAlreadyInTable(UInt8 un_sensor_id);
\
    /**
     * @brief PrintVirtualSensorTable Prints all the Virtual Sensors defined in the XML configuration file
     */
    void PrintVirtualSensorTable();

    /**
     * @brief IsAtLeastOneVirtualSensorDefined Tells whether at least one Virtual Sensor had been defined in
     * the XML configuaration file. This is useful to the physics engine to trigger the Virtual Sensor Server
     * @return True if at lest one Virtual Sensor had been defined in the XML configuaration file,
     * false otherwise
     */
    bool IsAtLeastOneVirtualSensorDefined();


    /*
     * Virtual Sensor Table modifiers
     */

    /**
     * @brief AddVirtualSensorEntry Adds a new Virtual Sensor in the Virtual Sensor Table
     * @param un_virtual_sensor_id The ID of the virtual sensor
     * @param un_virtual_sensor_data_size The size in bytes of the data of the given Virtual Sensor
     */
    void AddVirtualSensorEntry(const UInt8 un_virtual_sensor_id, const UInt32 un_virtual_sensor_data_size);

    /**
     * @brief SetVirtualSensorDefined Set the flag if at least one Virtual Sensor had been defined in the XML configuaration file
     */
    void SetVirtualSensorDefined();

    /**
	 * @brief ClearVirtualSensorData Clear the buffers of data for a robot, create it if it does not exist
	 */
    void ClearVirtualSensorData(UInt32 un_robot_id){
    	(*m_tVirtualSensorData)[un_robot_id] = SVirtualSensorData();
    }

private:

    /* Pattern Singleton */

    /**
     * @brief CVirtualSensorData Constructor for singleton
     */
    CVirtualSensorData();

    /**
     * @brief CVirtualSensorData Copy constructor for singleton
     */
    CVirtualSensorData(const CVirtualSensorData &);

    /**
     * @brief operator = Overridden not implemented for singleton
     */
    void operator=(const CVirtualSensorData &);

    /******************/

    /**
     * @brief GetDataSizeBySensorId Returns the size in bytes of the data of the given Virtual Sensor
     * @param un_sensor_id The given Virtual Sensor
     * @return The size in bytes of the data of the given Virtual Sensor
     */
    UInt32 GetDataSizeBySensorId(UInt8 un_sensor_id);

private:


    TVirtualSensorTable * m_tVirtualSensorTable;
    TVirtualSensorData * m_tVirtualSensorData;

    /*
     * Flag useful to the physics engine for enabling the Virtual Sensor Server
     * only if at least one Virtual Sensor exists in the XML configuration file.
     */
    bool m_bVirtualSensorDefined;

};

}

#endif
