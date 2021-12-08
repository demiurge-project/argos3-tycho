/**
 * @file <argos3/plugins/robots/generic/real_robot/real_virtual_black_white_location_sensor.cpp>
 *
 * @author Andreagiovanni Reina <areina@ulb.ac.be>
 */

#include "real_virtual_black_white_location_sensor.h"

namespace argos {

/****************************************/
/****************************************/

CRealVirtualBlackWhiteLocationSensor::CRealVirtualBlackWhiteLocationSensor() :
	CCI_VirtualBlackWhiteLocationSensor::CCI_VirtualBlackWhiteLocationSensor(),
	CRealVirtualSensor::CRealVirtualSensor(){
	m_unSensorId = CVirtualSensorRegister::GetVirtualSensorId("VirtualBlackWhiteLocationSensor");
}

/****************************************/
/****************************************/

CRealVirtualBlackWhiteLocationSensor::~CRealVirtualBlackWhiteLocationSensor() {
}

/****************************************/
/****************************************/

void CRealVirtualBlackWhiteLocationSensor::Init(TConfigurationNode& t_node){
}


/****************************************/
/****************************************/

void CRealVirtualBlackWhiteLocationSensor::UpdateValues() {
   m_cVirtualSensorClient.GetReadyData(m_cReading, m_unSensorId);
}

/****************************************/
/****************************************/

}
