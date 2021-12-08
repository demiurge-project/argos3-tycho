/**
 * @file <argos3/plugins/robots/e-puck/real_robot/real_epuck_its.h>
 *
 * This class extends the real e-puck robot implementation in order to
 * make the real robot work together with the Iridia Tracking System
 * plugin
 *
 * @author Mattia Salvaro
 */

#ifndef REAL_EPUCK_ITS_H
#define REAL_EPUCK_ITS_H


namespace argos {
	class CRealEPuck;
	class CRealEPuckITS;
}



#include <argos3/plugins/robots/e-puck/real_robot/real_epuck.h>
#include <argos3/plugins/robots/generic/real_robot/virtual_sensor_client.h>
#include <argos3/plugins/robots/generic/real_robot/real_virtual_sensor.h>
#include <argos3/plugins/robots/generic/real_robot/real_virtual_ground_sensor.h>
#include <argos3/plugins/robots/generic/real_robot/real_virtual_light_sensor.h>
#include <argos3/plugins/robots/generic/real_robot/real_gps_virtual_sensor.h>
#include <argos3/plugins/robots/generic/real_robot/real_destination_virtual_sensor.h>
#include <argos3/plugins/robots/generic/real_robot/real_virtual_rgb_ground_sensor.h>
#include <argos3/plugins/robots/generic/real_robot/real_virtual_radiation_sensor.h>
#include <argos3/plugins/robots/generic/real_robot/real_virtual_pollutant_sensor.h>
#include <argos3/plugins/robots/generic/real_robot/real_virtual_black_white_location_sensor.h>
namespace argos {

class CRealEPuckITS : public CRealEPuck {

protected:

   CRealEPuckITS();
   CRealEPuckITS& operator=(const CRealEPuckITS& c_epuck) { return *this; }

public:

   /**
    * Initializes the e-puck state (sensors/actuator and params)
    *
    * @param str_config_file_name path to the xml config file
    *
    * @param str_controller_id the controller id to select in the
    * xml file if there are severals.
    */
   void Init(const std::string& str_config_file_name,
             const std::string& str_controller_id);

   /**
    * Insert a new Sensor
    *
    * @param str_sensor_type the type of the sensor ("epuck_light"
    * for example)
    */
   virtual CCI_Sensor* InsertSensor(const std::string& str_sensor_type);

   /**
    * Returns the instance to the CRealEPuck class Since CRealEPuck
    * is a singleton, the instance is created at the first call of
    * this function and returned thereafter.
    *
    * @return The instance to the CRealEPuck class.
    */
   static CRealEPuckITS& GetInstance();

   /**
    * Class destructor, ensure free/clean/reset of all stuff in e-puck
    */
   virtual ~CRealEPuckITS();

    /**
     * Called at the end of every ReceiveData. Allows the sensors to
     * copy their specific data in a private structure. Only for
     * i2c sensors this function allows the directed data reading.
     */
    virtual void UpdateValues();

    // Updates virtual sensors
    void UpdateVirtualSensors();

    inline bool Start(){
    	return m_cVirtualSensorClient.IsArgosPlaying();
    }

    inline bool Stop(){
    	return !m_cVirtualSensorClient.IsArgosPlaying();
    }

    inline bool ErrorOnSocket(){
    	return m_cVirtualSensorClient.ErrorOnSocket();
    }

   private:

    CVirtualSensorClient& m_cVirtualSensorClient;

    /**
     * List of Virtual Sensors that communicate with the Virtual Sensor Client
     */
    std::vector<CRealVirtualSensor*> m_vecVirtualSensors;


    // NEW
    std::string m_strVirtualSensorServerAddress;
    UInt32 m_unVirtualSensorServerPort;


};

}
#endif

