/**
 * @file <argos3/plugins/robots/generic/real_robot/real_virtual_pollutant_sensor.cpp>
 *
 * @author Andreagiovanni Reina <areina@ulb.ac.be>
 */

#include "real_virtual_pollutant_sensor.h"

namespace argos {

/****************************************/
/****************************************/

CRealVirtualPollutantSensor::CRealVirtualPollutantSensor() :
	CCI_VirtualPollutantSensor::CCI_VirtualPollutantSensor(),
	CRealVirtualSensor::CRealVirtualSensor(){
	m_unSensorId = CVirtualSensorRegister::GetVirtualSensorId("VirtualPollutantSensor");
}

/****************************************/
/****************************************/

CRealVirtualPollutantSensor::~CRealVirtualPollutantSensor() {
}

/****************************************/
/****************************************/

void CRealVirtualPollutantSensor::Init(TConfigurationNode& t_node){
}


/****************************************/
/****************************************/

void CRealVirtualPollutantSensor::UpdateValues() {
   m_cVirtualSensorClient.GetReadyData(m_cReading, m_unSensorId);
}

/****************************************/
/****************************************/

}
