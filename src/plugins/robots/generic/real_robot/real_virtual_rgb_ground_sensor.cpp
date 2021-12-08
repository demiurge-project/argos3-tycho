/**
 * @file <argos3/plugins/robots/generic/real_robot/real_virtual_rgb_ground_sensor.cpp>
 *
 * @author Andreagiovanni Reina <areina@ulb.ac.be>
 */

#include "real_virtual_rgb_ground_sensor.h"

namespace argos {

/****************************************/
/****************************************/

CRealVirtualRGBGroundSensor::CRealVirtualRGBGroundSensor() :
	CCI_VirtualRGBGroundSensor::CCI_VirtualRGBGroundSensor(),
	CRealVirtualSensor::CRealVirtualSensor(){
	m_unSensorId = CVirtualSensorRegister::GetVirtualSensorId("VirtualRGBGroundSensor");
}

/****************************************/
/****************************************/

CRealVirtualRGBGroundSensor::~CRealVirtualRGBGroundSensor() {
}

/****************************************/
/****************************************/

void CRealVirtualRGBGroundSensor::Init(TConfigurationNode& t_node){
}


/****************************************/
/****************************************/

void CRealVirtualRGBGroundSensor::UpdateValues() {
   m_cVirtualSensorClient.GetReadyData(m_cReading, m_unSensorId);
}

/****************************************/
/****************************************/

}
