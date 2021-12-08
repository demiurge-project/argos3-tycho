/**
 * @file <argos3/plugins/robots/generic/real_robot/real_destination_virtual_sensor.cpp>
 *
 * @author Bernard Mayeur <bmayeur@ulb.ac.be>
 */

#include "real_destination_virtual_sensor.h"

namespace argos {

/****************************************/
/****************************************/

CRealDestinationVirtualSensor::CRealDestinationVirtualSensor() :
	CCI_DestinationVirtualSensor::CCI_DestinationVirtualSensor(),
	CRealVirtualSensor::CRealVirtualSensor(){
	m_unSensorId = CVirtualSensorRegister::GetVirtualSensorId("DestinationVirtualSensor");
}

/****************************************/
/****************************************/

CRealDestinationVirtualSensor::~CRealDestinationVirtualSensor() {
}

/****************************************/
/****************************************/

void CRealDestinationVirtualSensor::Init(TConfigurationNode& t_node){
}


/****************************************/
/****************************************/

void CRealDestinationVirtualSensor::UpdateValues() {
   m_cVirtualSensorClient.GetReadyData(m_cReading, m_unSensorId);
}

/****************************************/
/****************************************/

}
