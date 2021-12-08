/**
 * @file <argos3/plugins/robots/generic/real_robot/virtual_radiation_sensor.cpp>
 *
 * @author Andreagiovanni Reina <areina@ulb.ac.be>
 */

#include <argos3/core/simulator/simulator.h>

#include "virtual_radiation_sensor.h"

namespace argos {

/****************************************/
/****************************************/

CVirtualRadiationSensor::CVirtualRadiationSensor()
	  : CSimulatedSensor(), CGenericVirtualSensor(), CCI_VirtualRadiationSensor(),
	    m_pcEmbodiedEntity(NULL), m_pcRadioactiveMaterialEntity(NULL), m_cSpace(CSimulator::GetInstance().GetSpace()) {
	m_unSensorId = CVirtualSensorRegister::GetVirtualSensorId("VirtualRadiationSensor");
	m_unVirtualSensorDataSize = m_cReading.Serialize().Size();
    m_cVirtualSensorData.SetVirtualSensorDefined();
}

/****************************************/
/****************************************/

void CVirtualRadiationSensor::Init(TConfigurationNode &t_tree)
{
	CCI_VirtualRadiationSensor::Init(t_tree);
    // If the Virtual Sensor is not included in the Virtual Sensor Table
    if (!m_cVirtualSensorData.IsSensorAlreadyInTable(m_unSensorId)) {
    	/* The data of gps sensor is position(X,Y) and direction(angle)*/
        m_cVirtualSensorData.AddVirtualSensorEntry(m_unSensorId, m_unVirtualSensorDataSize);
    }

    Update();
}

/****************************************/
/****************************************/

void CVirtualRadiationSensor::SetRobot(CComposableEntity &c_entity)
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

    IdentifyRadioactiveMaterial();
}

/****************************************/
/****************************************/

void CVirtualRadiationSensor::IdentifyRadioactiveMaterial(){
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
				m_pcRadioactiveMaterialEntity = &(pcEPentity->GetEmbodiedEntity());
				break;
			}
		}
	} catch (CARGoSException e){
	}
}

/****************************************/
/****************************************/

void CVirtualRadiationSensor::Update()
{
	/* Get robot position */
	const CVector3& cRobotPos = m_pcEmbodiedEntity->GetOriginAnchor().Position;
	/* Get radioactive material position  */
	/* If m_pcRadioactiveMaterialEntity has not been assigned I try to assign it now */
	CVector3 cRadioMatPos;
	if (m_pcRadioactiveMaterialEntity == NULL){
		IdentifyRadioactiveMaterial();
	}
	if (m_pcRadioactiveMaterialEntity != NULL){
		cRadioMatPos = m_pcRadioactiveMaterialEntity->GetOriginAnchor().Position;
	}
	/* Get distance from Radioactive Material */
	Real fDistance = (CVector2(cRobotPos.GetX(), cRobotPos.GetY()) - CVector2(cRadioMatPos.GetX(), cRadioMatPos.GetY())).Length();

	/* Compute radioactive intensity */
	m_cReading.Intensity = 1/(fDistance*fDistance);

	m_cVirtualSensorData.AppendVirtualSensorData(m_cReading, m_pairITSAndRobotId.second, m_unSensorId);
}

/****************************************/
/****************************************/


const std::string CVirtualRadiationSensor::ClassName() const
{
    return "CVirtualRadiationSensor";
}


REGISTER_SENSOR(/* CLASSNAME      */ CVirtualRadiationSensor,
                /* LABEL          */ "virtual_radiation_sensor" ,
                /* IMPLEMENTATION */ "default",
                /* AUTHOR         */ "Andreagiovanni Reina [areina@ulb.ac.be]",
                /* VERSION        */ "1.0",
                /* BRIEF_DESCRIPTION */ "Virtual Radiation sensor to detect the radiation intensity",
                /* LONG_DESCRIPTION  */ "",
                /* STATUS            */ "Beta");

}
