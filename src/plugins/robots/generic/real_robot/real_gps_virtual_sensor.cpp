/**
 * @file <argos3/plugins/robots/generic/real_robot/real_gps_virtual_sensor.cpp>
 *
 * @author Bernard Mayeur <bmayeur@ulb.ac.be>
 */

#include "real_gps_virtual_sensor.h"


namespace argos {

   /****************************************/
   /****************************************/

   CRealGPSVirtualSensor::CRealGPSVirtualSensor() :
		CRealVirtualSensor::CRealVirtualSensor(),
		CCI_GPSVirtualSensor::CCI_GPSVirtualSensor(){
		m_unSensorId = CVirtualSensorRegister::GetVirtualSensorId("GPSVirtualSensor");
   }

   /****************************************/
   /****************************************/

   CRealGPSVirtualSensor::~CRealGPSVirtualSensor() {
   }

   /****************************************/
   /****************************************/

   void CRealGPSVirtualSensor::Init(TConfigurationNode& t_node) {
   }

   /****************************************/
   /****************************************/

   void CRealGPSVirtualSensor::UpdateValues() {
	   m_cVirtualSensorClient.GetReadyData(m_cReading, m_unSensorId);
	   if (m_cReading == m_cLastReading){
    	   ++m_unTimeTillUpdate;
       }
       else {
    	   m_unTimeTillUpdate = 0;
    	   m_cLastReading = m_cReading;
       }
   }

   /****************************************/
   /****************************************/
}
