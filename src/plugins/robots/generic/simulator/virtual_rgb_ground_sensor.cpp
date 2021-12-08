/**
 * @file <argos3/plugins/robots/generic/real_robot/virtual_rgb_ground_sensor.cpp>
 *
 * @author Andreagiovanni Reina <areina@ulb.ac.be>
 */

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/floor_entity.h>

#include "virtual_rgb_ground_sensor.h"

namespace argos {

/****************************************/
/****************************************/

CVirtualRGBGroundSensor::CVirtualRGBGroundSensor()
	  : CSimulatedSensor(), CGenericVirtualSensor(), CCI_VirtualRGBGroundSensor(),
	    m_pcEmbodiedEntity(NULL), m_cSpace(CSimulator::GetInstance().GetSpace()) {
	m_unSensorId = CVirtualSensorRegister::GetVirtualSensorId("VirtualRGBGroundSensor");
	m_unVirtualSensorDataSize = m_cReading.Serialize().Size();
    m_cVirtualSensorData.SetVirtualSensorDefined();
}

/****************************************/
/****************************************/

void CVirtualRGBGroundSensor::Init(TConfigurationNode &t_tree)
{
	CCI_VirtualRGBGroundSensor::Init(t_tree);
    // If the Virtual Sensor is not included in the Virtual Sensor Table
    if (!m_cVirtualSensorData.IsSensorAlreadyInTable(m_unSensorId)) {
    	/* The data of gps sensor is position(X,Y) and direction(angle)*/
        m_cVirtualSensorData.AddVirtualSensorEntry(m_unSensorId, m_unVirtualSensorDataSize);
    }
    Update();
}

/****************************************/
/****************************************/

void CVirtualRGBGroundSensor::SetRobot(CComposableEntity &c_entity)
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
			throw CARGoSException("Unknown robot");
		}
	}
    GetITSAndRobotIdFromArgosId(m_pairITSAndRobotId, m_strRobotArgosId);
    m_pcFloorEntity = &m_cSpace.GetFloorEntity();
}

/****************************************/
/****************************************/

void CVirtualRGBGroundSensor::Update()
{
	/* Get robot position and orientation */
	const CVector3& cEntityPos = m_pcEmbodiedEntity->GetOriginAnchor().Position;
	/* Get the color */
	const CColor& cColor = m_pcFloorEntity->GetColorAtPoint(cEntityPos.GetX(), cEntityPos.GetY());
	m_cReading.R = cColor.GetRed();
	m_cReading.G = cColor.GetGreen();
	m_cReading.B = cColor.GetBlue();
	//std::printf("%u , %u , %u \n",m_cReading.R,m_cReading.G,m_cReading.B);

	m_cVirtualSensorData.AppendVirtualSensorData(m_cReading, m_pairITSAndRobotId.second, m_unSensorId);
}

/****************************************/
/****************************************/


const std::string CVirtualRGBGroundSensor::ClassName() const
{
    return "CVirtualRGBGroundSensor";
}


REGISTER_SENSOR(/* CLASSNAME      */ CVirtualRGBGroundSensor,
                /* LABEL          */ "virtual_rgb_ground_sensor" ,
                /* IMPLEMENTATION */ "default",
                /* AUTHOR         */ "Andreagiovanni Reina [areina@ulb.ac.be]",
                /* VERSION        */ "1.0",
                /* BRIEF_DESCRIPTION */ "Ground sensor capable to perceive RGB colors",
                /* LONG_DESCRIPTION  */ "This sensor returns a RBG color corresponding to the ground under the center of the robot",
                /* STATUS            */ "Beta");

}
