/**
 * @file <argos3/plugins/robots/generic/real_robot/destination_virtual_sensor.cpp>
 *
 * @author Bernard Mayeur [bmayeur@ulb.ac.be]
 */

#include "destination_virtual_sensor.h"



namespace argos {

/****************************************/
/****************************************/

CDestinationVirtualSensor::CDestinationVirtualSensor()
	  : CSimulatedSensor(), CGenericVirtualSensor(), CCI_DestinationVirtualSensor(),
	    m_pcEmbodiedEntity(NULL) {
	m_unSensorId = CVirtualSensorRegister::GetVirtualSensorId("DestinationVirtualSensor");
	m_unVirtualSensorDataSize = m_cReading.Serialize().Size();
    m_cVirtualSensorData.SetVirtualSensorDefined();
}

/****************************************/
/****************************************/

void CDestinationVirtualSensor::Init(TConfigurationNode &t_tree)
{
    CCI_DestinationVirtualSensor::Init(t_tree);
    // If the Virtual Sensor is not included in the Virtual Sensor Table
    if (!m_cVirtualSensorData.IsSensorAlreadyInTable(m_unSensorId)) {
    	/* The data of gps sensor is position(X,Y) and direction(angle)*/
        m_cVirtualSensorData.AddVirtualSensorEntry(m_unSensorId, m_unVirtualSensorDataSize);
    }
    Update();
}

/****************************************/
/****************************************/

void CDestinationVirtualSensor::SetRobot(CComposableEntity &c_entity)
{
    m_strRobotArgosId = c_entity.GetId();
    try{
		m_pcEmbodiedEntity = &(dynamic_cast<CFootBotEntity&>(c_entity).GetEmbodiedEntity());
	}
	catch (const std::bad_cast& e){
		try{
			m_pcEmbodiedEntity = &(dynamic_cast<CEPuckEntity&>(c_entity).GetEmbodiedEntity());
		}
		catch (const std::bad_cast& e){
			try{
                m_pcEmbodiedEntity = &(dynamic_cast<CRVREntity&>(c_entity).GetEmbodiedEntity());
            }
            catch (const std::bad_cast& e){
                throw CARGoSException("Unknown robot");
            }
		}
	}
    GetITSAndRobotIdFromArgosId(m_pairITSAndRobotId, m_strRobotArgosId);
}

/****************************************/
/****************************************/

void CDestinationVirtualSensor::Update()
{
	m_cVirtualSensorData.AppendVirtualSensorData(m_cReading, m_pairITSAndRobotId.second, m_unSensorId);
}

/****************************************/
/****************************************/


const std::string CDestinationVirtualSensor::ClassName() const
{
    return "CDestinationVirtualSensor";
}


REGISTER_SENSOR(/* CLASSNAME      */ CDestinationVirtualSensor,
                /* LABEL          */ "destination_virtual_sensor" ,
                /* IMPLEMENTATION */ "default",
                /* AUTHOR         */ "Bernard Mayeur [bmayeur@ulb.ac.be]",
                /* VERSION        */ "1.0",
                /* BRIEF_DESCRIPTION */ "Sensor sending a location in the arena",
                /* LONG_DESCRIPTION  */ "This sensor send a location and direction in the arena (direction to look from this point)",
                /* STATUS            */ "Working");

}
