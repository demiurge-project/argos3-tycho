/**
 * @file <argos3/plugins/robots/generic/real_robot/virtual_black_white_location_sensor.cpp>
 *
 * @author Andreagiovanni Reina <areina@ulb.ac.be>
 */

#include <argos3/core/simulator/simulator.h>

#include "virtual_black_white_location_sensor.h"

namespace argos {

/****************************************/
/****************************************/

CVirtualBlackWhiteLocationSensor::CVirtualBlackWhiteLocationSensor()
	  : CSimulatedSensor(), CGenericVirtualSensor(), CCI_VirtualBlackWhiteLocationSensor(),
	    m_pcEmbodiedEntity(NULL), m_pcTargetEntity(NULL), m_cSpace(CSimulator::GetInstance().GetSpace()) {
	m_unSensorId = CVirtualSensorRegister::GetVirtualSensorId("VirtualBlackWhiteLocationSensor");
	m_unVirtualSensorDataSize = m_cReading.Serialize().Size();
    m_cVirtualSensorData.SetVirtualSensorDefined();
}

/****************************************/
/****************************************/

void CVirtualBlackWhiteLocationSensor::Init(TConfigurationNode &t_tree)
{
	CCI_VirtualBlackWhiteLocationSensor::Init(t_tree);
    // If the Virtual Sensor is not included in the Virtual Sensor Table
    if (!m_cVirtualSensorData.IsSensorAlreadyInTable(m_unSensorId)) {
    	/* The data of gps sensor is position(X,Y) and direction(angle)*/
        m_cVirtualSensorData.AddVirtualSensorEntry(m_unSensorId, m_unVirtualSensorDataSize);
    }

    Real x,y;

    GetNodeAttribute(t_tree, "white_min_x", x);
    GetNodeAttribute(t_tree, "white_min_y", y);
    m_vWhiteMin.Set(x,y);
    GetNodeAttribute(t_tree, "white_max_x", x);
    GetNodeAttribute(t_tree, "white_max_y", y);
    m_vWhiteMax.Set(x,y);

    GetNodeAttribute(t_tree, "black_min_x", x);
    GetNodeAttribute(t_tree, "black_min_y", y);
    m_vBlackMin.Set(x,y);
    GetNodeAttribute(t_tree, "black_max_x", x);
    GetNodeAttribute(t_tree, "black_max_y", y);
    m_vBlackMax.Set(x,y);

    m_vCenterBlack.Set((m_vBlackMin.GetX()+m_vBlackMax.GetX())/2,
                       (m_vBlackMin.GetY()+m_vBlackMax.GetY())/2);
    m_vCenterWhite.Set((m_vWhiteMin.GetX()+m_vWhiteMax.GetX())/2,
            (m_vWhiteMin.GetY()+m_vWhiteMax.GetY())/2);

    Update();
}

/****************************************/
/****************************************/

void CVirtualBlackWhiteLocationSensor::SetRobot(CComposableEntity &c_entity)
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

    IdentifyTargetEntity();
}

/****************************************/
/****************************************/

void CVirtualBlackWhiteLocationSensor::IdentifyTargetEntity(){
	try{
		/* Get the map of all epucks from the space */
		CSpace::TMapPerType& tEPMap = m_cSpace.GetEntitiesByType("epuck");
		/* Go through them */
		for(CSpace::TMapPerType::iterator it = tEPMap.begin(); it != tEPMap.end(); ++it) {
			/* Set some data in the epuck controller */
			CEPuckEntity* pcEPentity = any_cast<CEPuckEntity*>(it->second);
			std::string strRobotID = pcEPentity->GetId();

			//printf("Robot ID is %s \n",strRobotID.c_str());
			if (strRobotID == "epuck_1"){
				m_pcTargetEntity = &(pcEPentity->GetEmbodiedEntity());
				break;
			}
		}
	} catch (CARGoSException e){
	}
}

/****************************************/
/****************************************/

void CVirtualBlackWhiteLocationSensor::Update()
{
	/* Get robot position */
    const CVector3& cRobotPos3 = m_pcEmbodiedEntity->GetOriginAnchor().Position;
    const CVector2 vRobotPos2(cRobotPos3.GetX(), cRobotPos3.GetY());

    CRadians angleX, angleY, angleZ;
    m_pcEmbodiedEntity->GetOriginAnchor().Orientation.ToEulerAngles(angleZ, angleY, angleX);

    CVector2 vToBlackLocation, vToWhiteLocation;
    vToBlackLocation = m_vCenterBlack - vRobotPos2;
    vToWhiteLocation = m_vCenterWhite - vRobotPos2;

    CVector2 vToBlackInRobotFoR, vToWhiteInRobotFoR;
    vToBlackInRobotFoR.FromPolarCoordinates(1, vToBlackLocation.Angle()-angleZ);
    vToWhiteInRobotFoR.FromPolarCoordinates(1, vToWhiteLocation.Angle()-angleZ);

    m_cReading.vBlackLocation = vToBlackInRobotFoR;
    m_cReading.vWhiteLocation = vToWhiteInRobotFoR;

    m_cVirtualSensorData.AppendVirtualSensorData(m_cReading, m_pairITSAndRobotId.second, m_unSensorId);
}

/****************************************/
/****************************************/


const std::string CVirtualBlackWhiteLocationSensor::ClassName() const
{
    return "CVirtualBlackWhiteLocationSensor";
}


REGISTER_SENSOR(/* CLASSNAME      */ CVirtualBlackWhiteLocationSensor,
                /* LABEL          */ "virtual_black_white_location_sensor" ,
                /* IMPLEMENTATION */ "default",
                /* AUTHOR         */ "Gaëtan Podevijn [gpodevij@ulb.ac.be]",
                /* VERSION        */ "1.0",
                /* BRIEF_DESCRIPTION */ "Virtual BlackWhiteLocation sensor to ... ||ADD DESCRIPTION HERE||",
                /* LONG_DESCRIPTION  */ "",
                /* STATUS            */ "Beta");

}
