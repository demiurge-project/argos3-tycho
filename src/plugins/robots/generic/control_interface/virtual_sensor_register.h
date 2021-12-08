/**
 * @file <argos3/plugins/robots/generic/control_interface/virtual_sensor_register.h>
 *
 * @brief This file provides a register for all the implemented virutal sensors.
 * Every virutal sensor must be included in the Virtual Sensor Map together with its ID,
 * which can be arbitrary but unique. A good practice is to assign progressive values to the IDs.
 *
 *
 * @author Mattia Salvaro
 */

#ifndef VIRTUAL_SENSOR_REGISTER_H
#define VIRTUAL_SENSOR_REGISTER_H

#include <map>
#include <string>
// #ifndef __APPLE__
// #include <auto_ptr.h>
// #endif

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/configuration/argos_exception.h>
#include <argos3/core/utility/logging/argos_log.h>

namespace argos {

class CVirtualSensorRegister
{

private:

    /**
     * @brief m_mapVirtualSensor List of all the implemented sensor with their own ID
     */
    static std::map<std::string, UInt8> m_mapVirtualSensor;
     /**
     * @brief CVirtualSensorRegister Constructor
     */
    CVirtualSensorRegister();


    /**
     * @brief CVirtualSensorRegister Private copy constructor for singleton
     */
    CVirtualSensorRegister(const CVirtualSensorRegister &);

    /**
     * @brief operator = Overridden operator for singleton
     */
    void operator=(const CVirtualSensorRegister &);

    /**
     * @brief GenerateVirtualSensorMap The method to instantiate the Virtual Sensor Register singleton
     * @return A reference to the list of implemented sensors
     */
    static void GenerateVirtualSensorMap();

public:

    /**
     * Destructor
     */
    //~CVirtualSensorRegister();

    /**
     * @brief GetVirtualSensorId Retrieves the ID for a given sensor
     * @param str_virtual_sensor_class_name The name of the class of the given Virtual Sensor
     * @return
     */
    static UInt8 GetVirtualSensorId(const std::string & str_virtual_sensor_class_name);
};


}

#endif
