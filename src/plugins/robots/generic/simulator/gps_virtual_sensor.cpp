/**
 * @file <argos3/plugins/robots/generic/real_robot/gps_virtual_sensor.cpp>
 *
 * @author Bernard Mayeur [bmayeur@ulb.ac.be]
 */

#include "gps_virtual_sensor.h"



namespace argos {

/****************************************/
/****************************************/

CGPSVirtualSensor::CGPSVirtualSensor():
	    CSimulatedSensor(), CGenericVirtualSensor(), CCI_GPSVirtualSensor(),
        m_pcEmbodiedEntity(NULL) {
	m_unSensorId = CVirtualSensorRegister::GetVirtualSensorId("GPSVirtualSensor");
	m_unVirtualSensorDataSize=0;
    m_cVirtualSensorData.SetVirtualSensorDefined();

}

/****************************************/
/****************************************/

void CGPSVirtualSensor::Init(TConfigurationNode &t_tree)
{
    CCI_GPSVirtualSensor::Init(t_tree);
    m_unVirtualSensorDataSize = m_cReading.Serialize().Size();
    // If the Virtual Sensor is not included in the Virtual Sensor Table
    if (!m_cVirtualSensorData.IsSensorAlreadyInTable(m_unSensorId)) {
    	/* The data of gps sensor is position(X,Y) and direction(angle)*/
        m_cVirtualSensorData.AddVirtualSensorEntry(m_unSensorId, m_unVirtualSensorDataSize);
    }
    Update();
}

/****************************************/
/****************************************/

void CGPSVirtualSensor::SetRobot(CComposableEntity &c_entity)
{
	// Bind the instance of the Virtual Sensor with the robot
	//CCI_GPSVirtualSensor::SetRobot(c_entity);
    m_strRobotArgosId = c_entity.GetId();
    try{
		m_pcEmbodiedEntity = &(dynamic_cast<CFootBotEntity&>(c_entity).GetEmbodiedEntity());
	}
	catch (const std::bad_cast& e){
		try{
			m_pcEmbodiedEntity = &(dynamic_cast<CEPuckEntity&>(c_entity).GetEmbodiedEntity());
		}
		catch (const std::bad_cast& e){
			throw CARGoSException("Unknown robot");
		}
	}
    GetITSAndRobotIdFromArgosId(m_pairITSAndRobotId, m_strRobotArgosId);
}

/****************************************/
/****************************************/

void CGPSVirtualSensor::Update()
{
	m_cReading.XRange = m_pcEmbodiedEntity->GetOriginAnchor().Position.GetX();
	m_cReading.YRange = m_pcEmbodiedEntity->GetOriginAnchor().Position.GetY();
	CRadians cBearing, cUselessAngle;
	m_pcEmbodiedEntity->GetOriginAnchor().Orientation.ToEulerAngles(cBearing,cUselessAngle,cUselessAngle);
	m_cReading.Bearing = cBearing.GetValue();
	m_cVirtualSensorData.AppendVirtualSensorData(m_cReading, m_pairITSAndRobotId.second, m_unSensorId);
}

/****************************************/
/****************************************/


const std::string CGPSVirtualSensor::ClassName() const
{
    return "CGPSVirtualSensor";
}

/****************************************/
/****************************************/

REGISTER_SENSOR(/* CLASSNAME      */ CGPSVirtualSensor,
                /* LABEL          */ "gps_virtual_sensor" ,
                /* IMPLEMENTATION */ "default",
                /* AUTHOR         */ "Bernard Mayeur [bmayeur@ulb.ac.be]",
                /* VERSION        */ "1.0",
                /* BRIEF_DESCRIPTION */ "A virtual Global positioning sensor.",
                /* LONG_DESCRIPTION  */ "This sensor accesses a virtual GPS sensor.",
                /* STATUS            */ "Working");

}
