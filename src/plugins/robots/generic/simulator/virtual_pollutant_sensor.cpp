/**
 * @file <argos3/plugins/robots/generic/real_robot/virtual_pollutant_sensor.cpp>
 *
 * @author Andreagiovanni Reina <areina@ulb.ac.be>
 */

#include <argos3/core/simulator/simulator.h>

#include "virtual_pollutant_sensor.h"

namespace argos {

/****************************************/
/****************************************/

CVirtualPollutantSensor::CVirtualPollutantSensor()
	  : CSimulatedSensor(), CGenericVirtualSensor(), CCI_VirtualPollutantSensor(),
	    m_pcEmbodiedEntity(NULL), m_pcTargetEntity(NULL), m_cSpace(CSimulator::GetInstance().GetSpace()),
		m_fTriangleMatrixDet(1) {
	m_unSensorId = CVirtualSensorRegister::GetVirtualSensorId("VirtualPollutantSensor");
	m_unVirtualSensorDataSize = m_cReading.Serialize().Size();
    m_cVirtualSensorData.SetVirtualSensorDefined();
}

/****************************************/
/****************************************/

void CVirtualPollutantSensor::Init(TConfigurationNode &t_tree)
{
	CCI_VirtualPollutantSensor::Init(t_tree);

	/* Parse XML file */
	GetNodeAttributeOrDefault(t_tree, "direction_angle", m_cParam.cDirectionAngle, CDegrees(0));
	GetNodeAttributeOrDefault(t_tree, "wide_angle", m_cParam.cWideAngle, CDegrees(90));
	GetNodeAttributeOrDefault(t_tree, "max_distance", m_cParam.fMaxDistance, 1.0);
	GetNodeAttributeOrDefault(t_tree, "rotation_timestep", m_cParam.uRotationTimestep, UInt32(0));
	GetNodeAttributeOrDefault(t_tree, "rotation_angle", m_cParam.cRotationAngle, CDegrees(90));

	LOG << "m_cDirectionAngle " << m_cParam.cDirectionAngle << "m_cWideAngle " << m_cParam.cWideAngle << " m_fMaxDistance: " << m_cParam.fMaxDistance << " m_uRotationTimestep: " << m_cParam.uRotationTimestep << " m_cRotationAngle: " << m_cParam.cRotationAngle << std::endl;

    // If the Virtual Sensor is not included in the Virtual Sensor Table
    if (!m_cVirtualSensorData.IsSensorAlreadyInTable(m_unSensorId)) {
    	/* The data of gps sensor is position(X,Y) and direction(angle)*/
        m_cVirtualSensorData.AddVirtualSensorEntry(m_unSensorId, m_unVirtualSensorDataSize);
    }

    Update();
}

/****************************************/
/****************************************/

void CVirtualPollutantSensor::SetRobot(CComposableEntity &c_entity)
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

    IdentifyPollutantEntity();
}

/****************************************/
/****************************************/

void CVirtualPollutantSensor::IdentifyPollutantEntity(){
	try{
		/* Get the map of all epucks from the space */
		CSpace::TMapPerType& tEPMap = m_cSpace.GetEntitiesByType("epuck");
		/* Go through them */
		for(CSpace::TMapPerType::iterator it = tEPMap.begin(); it != tEPMap.end(); ++it) {
			/* Set some data in the epuck controller */
			CEPuckEntity* pcEPentity = any_cast<CEPuckEntity*>(it->second);
			std::string strRobotID = pcEPentity->GetId();

            //printf("Robot ID is %s \n",strRobotID.c_str());
			if (strRobotID == "epuck_22_1"){
				m_pcTargetEntity = &(pcEPentity->GetEmbodiedEntity());
				break;
			}
		}
	} catch (CARGoSException e){
	}
	ComputePollutedCone();
}

void CVirtualPollutantSensor::ComputePollutedCone(){
	if (m_pcTargetEntity != NULL){
		/* Compute the triangle of influence */
		CVector3 cSourcePos3 = m_pcTargetEntity->GetOriginAnchor().Position;
		m_cVertex1.Set(cSourcePos3.GetX(), cSourcePos3.GetY());
		m_cVertex2.FromPolarCoordinates(m_cParam.fMaxDistance, ToRadians(-m_cParam.cWideAngle/2));
		m_cVertex2.Rotate(ToRadians(m_cParam.cDirectionAngle));
		m_cVertex2 += m_cVertex1;
		m_cVertex3.FromPolarCoordinates(m_cParam.fMaxDistance, ToRadians(+m_cParam.cWideAngle/2));
		m_cVertex3.Rotate(ToRadians(m_cParam.cDirectionAngle));
		m_cVertex3 += m_cVertex1;

		/* Compute the determinant of Triangle Matrix (y2 - y3)(x1 - x3) + (x3 - x2)(y1 - y3) */
		m_fTriangleMatrixDet = (m_cVertex2.GetY() - m_cVertex3.GetY())*(m_cVertex1.GetX() - m_cVertex3.GetX()) +
				(m_cVertex3.GetX() - m_cVertex2.GetX())*(m_cVertex1.GetY() - m_cVertex3.GetY());

		//LOG << "V1: " << m_cVertex1 << " V2: " << m_cVertex2 << " V3: " << m_cVertex3 << " matDet: " << m_fTriangleMatrixDet << std::endl;
	}
}

/****************************************/
/****************************************/

void CVirtualPollutantSensor::ConvertPointInBarycentricCoordinates(CVector2 &cPoint, const CVector2 &cV1, const CVector2 &cV2, const CVector2 &cV3, Real fMatrixDet){
	Real x = cPoint.GetX();
	Real y = cPoint.GetY();
	/* r1 = (y2 - y3)(x - x3) + (x3 - x2)(y - y3) all divided by MatDet */
	cPoint.SetX( ((cV2.GetY() - cV3.GetY())*(x - cV3.GetX()) + (cV3.GetX() - cV2.GetX())*(y - cV3.GetY()))/fMatrixDet );
	/* r2 = (y3 - y1)(x - x3) + (x1 - x3)(y - y3) all divided by MatDet */
	cPoint.SetY( ((cV3.GetY() - cV1.GetY())*(x - cV3.GetX()) + (cV1.GetX() - cV3.GetX())*(y - cV3.GetY()))/fMatrixDet );
}

/****************************************/
/****************************************/

void CVirtualPollutantSensor::Update()
{
	/* Default virtual sensor Value is ZER0 */
	m_cReading.Value = 0;

	/* Get robot position */
	const CVector3& cRobotPos3 = m_pcEmbodiedEntity->GetOriginAnchor().Position;
	CVector2 cRobotPos(cRobotPos3.GetX(),cRobotPos3.GetY());
	/* Get target entity position  */
	/* If m_pcTargetEntity has not been assigned I try to assign it now */
	if (m_pcTargetEntity == NULL){
		IdentifyPollutantEntity();
	}
	if (m_pcTargetEntity != NULL){
		if (m_cParam.uRotationTimestep == m_cSpace.GetSimulationClock()){
			//LOG << "ROTATION!" << std::endl;
			m_cParam.cDirectionAngle += m_cParam.cRotationAngle;
//			ComputePollutedCone();
		}
		//if (m_fTriangleMatrixDet == 0)
		ComputePollutedCone();
		//LOG << "X= " << cRobotPos.GetX() << " Y=" << cRobotPos.GetY() << std::endl;
		ConvertPointInBarycentricCoordinates(cRobotPos, m_cVertex1, m_cVertex2, m_cVertex3, m_fTriangleMatrixDet);
		//LOG << "X= " << cRobotPos.GetX() << " Y=" << cRobotPos.GetY() << std::endl;
		/* if robot inside polluted triangle set Value to 1 --- that is 0 <= r1 <= 1 and 0 <= r2 <= 1 and r1 + r2 <= 1*/
		if (cRobotPos.GetX() >= 0 && cRobotPos.GetX() <= 1 && cRobotPos.GetY() >= 0 && cRobotPos.GetY() <= 1 && (cRobotPos.GetX()+cRobotPos.GetY()) <= 1){
			m_cReading.Value = 1.0f;
		}
	}

	m_cVirtualSensorData.AppendVirtualSensorData(m_cReading, m_pairITSAndRobotId.second, m_unSensorId);
}

/****************************************/
/****************************************/


const std::string CVirtualPollutantSensor::ClassName() const
{
    return "CVirtualPollutantSensor";
}


REGISTER_SENSOR(/* CLASSNAME      */ CVirtualPollutantSensor,
                /* LABEL          */ "virtual_pollutant_sensor" ,
                /* IMPLEMENTATION */ "default",
                /* AUTHOR         */ "Andreagiovanni Reina [areina@ulb.ac.be]",
                /* VERSION        */ "1.0",
                /* BRIEF_DESCRIPTION */ "Virtual Pollutant sensor to sense a Value of pollutant in the environment.",
                /* LONG_DESCRIPTION  */ "Virtual Pollutant sensor to sense a Value of pollutant in the environment. "
                		"The pollutant has a diffusion cone with focus in the tag #1 and other parameters specified by the user as follow:\n"
                		"direction_angle : angle in absolute ref frame in which the pollutant expands \n"
                		"max_distance: maximum distance until the pollutant is perceived\n"
                		"wide_angle: angle of the cone (triangle) that expands from the tag#1 \n"
                		"rotation_timestep: number of timesteps after which the cone rotates of rotation_angle degrees clockwise"
                		"rotation_angle: degrees of rotation counterclockwise",
                /* STATUS            */ "Beta");

}
