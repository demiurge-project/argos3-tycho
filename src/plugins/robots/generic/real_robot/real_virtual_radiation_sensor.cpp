/**
 * @file <argos3/plugins/robots/generic/real_robot/real_virtual_radiation_sensor.cpp>
 *
 * @author Andreagiovanni Reina <areina@ulb.ac.be>
 */

#include "real_virtual_radiation_sensor.h"

namespace argos {

/****************************************/
/****************************************/

CRealVirtualRadiationSensor::CRealVirtualRadiationSensor() :
	CCI_VirtualRadiationSensor::CCI_VirtualRadiationSensor(),
	CRealVirtualSensor::CRealVirtualSensor(){
	m_unSensorId = CVirtualSensorRegister::GetVirtualSensorId("VirtualRadiationSensor");
}

/****************************************/
/****************************************/

CRealVirtualRadiationSensor::~CRealVirtualRadiationSensor() {
}

/****************************************/
/****************************************/

void CRealVirtualRadiationSensor::Init(TConfigurationNode& t_node){
}


/****************************************/
/****************************************/

void CRealVirtualRadiationSensor::UpdateValues() {
   m_cVirtualSensorClient.GetReadyData(m_cReading, m_unSensorId);
}

/****************************************/
/****************************************/

}
