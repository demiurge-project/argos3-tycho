/**
 * @file <argos3/plugins/robots/generic/real_robot/real_virtual_light_sensor.cpp>
 *
 * @author Mattia Salvaro
 */

#include "real_virtual_light_sensor.h"


namespace argos {

UInt32 CRealVirtualLightSensor::CLightVirtualSensorReading::unVectorSize = 0;

   /****************************************/
   /****************************************/

   CRealVirtualLightSensor::CRealVirtualLightSensor() :
	   CRealVirtualSensor()
   {
       m_unSensorId = CVirtualSensorRegister::GetVirtualSensorId("LightVirtualSensor");
   }

   /****************************************/
   /****************************************/

   CRealVirtualLightSensor::~CRealVirtualLightSensor() {
   }

   /****************************************/
   /****************************************/

   void CRealVirtualLightSensor::Init(TConfigurationNode& t_node) {
   }

   /****************************************/
   /****************************************/

   void CRealVirtualLightSensor::UpdateValues() {
       m_cVirtualSensorClient.GetReadyData(m_cReadings,m_unSensorId);
       m_tReadings = m_cReadings.GetReadings();
   }
}
