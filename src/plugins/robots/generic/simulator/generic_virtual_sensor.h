#ifndef GENERIC_VIRTUAL_SENSOR_H
#define GENERIC_VIRTUAL_SENSOR_H

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/string_utilities.h>
#include <argos3/core/utility/configuration/argos_configuration.h>

#include <argos3/plugins/simulator/physics_engines/iridia_tracking_system/virtual_sensor_server/virtual_sensor_data.h>
#include <argos3/plugins/robots/generic/control_interface/virtual_sensor_register.h>

namespace argos {

class CGenericVirtualSensor
{

public:

    CGenericVirtualSensor();

    virtual ~CGenericVirtualSensor() {}

    //virtual const std::string& GetSensorType() const = 0;

protected:

    void GetITSAndRobotIdFromArgosId(std::pair<UInt32, UInt32> & pair_its_androbot_id, const std::string & str_argos_id);

protected:

    /**
     * @brief m_pairITSAndRobotId <ITS ID (tag), robot ID>
     */
   std::pair<UInt32, UInt32> m_pairITSAndRobotId;

   /**
    * @brief m_cVirtualSensorData Reference to the singleton Virtual Sensor Data Struct
    */
   CVirtualSensorData & m_cVirtualSensorData;

   /**
    * @brief m_unVirtualSensorDataSize Size of the data of this particular virtual sensor
    */
   UInt32 m_unVirtualSensorDataSize;

   /**
    * @brief m_strRobotArgosId The robot ID defined in the XML configuration file
    */
   std::string m_strRobotArgosId;

   /**
    * @brief m_bRealExperiment Flag that tell if the experiment is real or simulated
    */
   bool m_bRealExperiment;

   /**
   	* @brief m_unSensorId The ID that represents the sensor in the Virtual Sensor Register
   	*/
   UInt8 m_unSensorId;

};


}

#endif
