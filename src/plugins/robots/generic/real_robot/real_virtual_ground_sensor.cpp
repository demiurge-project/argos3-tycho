/**
 * @file <argos3/plugins/robots/generic/real_robot/real_virtual_ground_sensor.cpp>
 *
 * @author Mattia Salvaro
 */

#include "real_virtual_ground_sensor.h"


namespace argos {

UInt32 CRealVirtualGroundSensor::CGroundVirtualSensorReading::unVectorSize = 0;

   /****************************************/
   /****************************************/

   CRealVirtualGroundSensor::CRealVirtualGroundSensor() :
	   CRealVirtualSensor()
   {
       m_unSensorId = CVirtualSensorRegister::GetVirtualSensorId("GroundVirtualSensor");
   }

   /****************************************/
   /****************************************/

   CRealVirtualGroundSensor::~CRealVirtualGroundSensor() {
   }

   /****************************************/
   /****************************************/

   void CRealVirtualGroundSensor::Init(TConfigurationNode& t_node){
   }

   /****************************************/
   /****************************************/

   void CRealVirtualGroundSensor::UpdateValues() {
       m_cVirtualSensorClient.GetReadyData(m_cReadings, m_unSensorId);
       m_tReadings = m_cReadings.GetReadings();
   }
}
