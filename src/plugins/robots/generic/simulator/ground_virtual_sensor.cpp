/**
 * @file <argos3/plugins/robot/generic/simulator/ground_virtual_sensor.cpp>
 *
 * Provides the Virutal Ground Sensor
 *
 * @author Mattia Salvaro
 */


#include "ground_virtual_sensor.h"



namespace argos {

UInt32 CGroundVirtualSensor::CGroundVirtualSensorReading::unVectorSize=0;

/****************************************/
/****************************************/

CGroundVirtualSensor::CGroundVirtualSensor()
    : CGenericVirtualSensor(), CGroundRotZOnlySensor()
{
	m_unSensorId = CVirtualSensorRegister::GetVirtualSensorId("GroundVirtualSensor");

}

/****************************************/
/****************************************/

void CGroundVirtualSensor::Init(TConfigurationNode &t_tree)
{
    CGroundRotZOnlySensor::Init(t_tree);
    // Calculate the data size of this particular Virtual Sensor
    m_cReading.SetVectorSize(m_pcGroundSensorEntity->GetNumSensors());
    m_unVirtualSensorDataSize = m_cReading.Serialize().Size();

    // If the Virtual Sensor is not included in the Virtual Sensor Table yet...
    if (!m_cVirtualSensorData.IsSensorAlreadyInTable(m_unSensorId)) {

        // Add the Virtual Sensor Entry in the Virtual Sensor Table
        m_cVirtualSensorData.AddVirtualSensorEntry(m_unSensorId, m_unVirtualSensorDataSize);
    }

    Update();
}

/****************************************/
/****************************************/

void CGroundVirtualSensor::SetRobot(CComposableEntity &c_entity)
{
    // Bind the instance of the Virtual Sensor with the robot
    CGroundRotZOnlySensor::SetRobot(c_entity);
    // Set the pair of ITS and Robot ID starting from the Argos ID
    m_strRobotArgosId = c_entity.GetId();
    GetITSAndRobotIdFromArgosId(m_pairITSAndRobotId, m_strRobotArgosId);
}

/****************************************/
/****************************************/

void CGroundVirtualSensor::Update()
{
    CGroundRotZOnlySensor::Update();
    m_cReading = CGroundVirtualSensorReading(m_tReadings);

	// Append the Virtual Sensor data of the sensor in the Virtual Sensor Data Struct
	m_cVirtualSensorData.AppendVirtualSensorData(m_cReading, m_pairITSAndRobotId.second, m_unSensorId);

}

REGISTER_SENSOR(CGroundVirtualSensor,
                "virtual_ground_sensor", "default",
                "Mattia Salvaro",
                "1.0",
                "The general robot virtual ground sensor.",
                "This sensor accesses the general virtual ground sensor.\n",
                "Usable"
       );

}
