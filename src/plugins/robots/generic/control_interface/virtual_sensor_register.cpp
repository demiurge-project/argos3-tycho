#include "virtual_sensor_register.h"
#include <memory>

namespace argos {

/****************************************/
/****************************************/

std::map<std::string, UInt8> CVirtualSensorRegister::m_mapVirtualSensor = std::map<std::string, UInt8>();

/****************************************/
/****************************************/

CVirtualSensorRegister::CVirtualSensorRegister()
{
    CVirtualSensorRegister::GenerateVirtualSensorMap();
}

/****************************************/
/****************************************/

UInt8 CVirtualSensorRegister::GetVirtualSensorId(const std::string &str_virtual_sensor_class_name)
{
    // If the singleton Virtual Sensor Register does not exists, then create it.
    static std::auto_ptr<CVirtualSensorRegister> pcVirtualSensorServerInstance(new CVirtualSensorRegister());
    LOG << "Looking for ID for virtual sensor class: " << str_virtual_sensor_class_name.c_str() << "\n" ;
    // If the Virtual Sensor Class Name exists in the Virtual Sensor Map...
    std::map<std::string, UInt8>::iterator itMapVirtualSensor = CVirtualSensorRegister::m_mapVirtualSensor.find(str_virtual_sensor_class_name);

    if (itMapVirtualSensor != m_mapVirtualSensor.end()) {
        // ... then return the Virtual Sensor ID associated with the Virtual Sensor Class Name
        //LOG << "Virtual Sensor Type: " << str_virtual_sensor_class_name.c_str() << "\t Virtual Sensor ID: " << (*itMapVirtualSensor).second << "\n";
        return (*itMapVirtualSensor).second;
    }
    else {
        THROW_ARGOSEXCEPTION("Class " << str_virtual_sensor_class_name << " is not declared as a Virtual Sensor!\n");
        return 0;
    }
}

/****************************************/
/****************************************/

void CVirtualSensorRegister::GenerateVirtualSensorMap()
{
	// Create the Virtual Sensor Map and fill it with the implemented available Virtual Sensors.
	// IMPORTANT: each brand new Virtual Sensor implementation must be added here with incremetal number ID.
    if (m_mapVirtualSensor.size() == 0){
		m_mapVirtualSensor.insert(std::make_pair<std::string, UInt8>("GroundVirtualSensor", 1));
		m_mapVirtualSensor.insert(std::make_pair<std::string, UInt8>("LightVirtualSensor", 2));
		m_mapVirtualSensor.insert(std::make_pair<std::string, UInt8>("GPSVirtualSensor", 3));
		m_mapVirtualSensor.insert(std::make_pair<std::string, UInt8>("DestinationVirtualSensor", 4));
		m_mapVirtualSensor.insert(std::make_pair<std::string, UInt8>("VirtualRGBGroundSensor", 5));
		m_mapVirtualSensor.insert(std::make_pair<std::string, UInt8>("VirtualRadiationSensor", 6));
		m_mapVirtualSensor.insert(std::make_pair<std::string, UInt8>("VirtualPollutantSensor", 7));
        m_mapVirtualSensor.insert(std::make_pair<std::string, UInt8>("VirtualBlackWhiteLocationSensor", 8));
    }
    else
    	THROW_ARGOSEXCEPTION("CVirtualSensorRegister should only be created once")

}


}
